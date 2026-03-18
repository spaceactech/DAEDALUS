#ifndef STM32SERVO_H
#define STM32SERVO_H

#if !defined(ARDUINO_ARCH_STM32)
#  error "This library only supports boards with STM32 processor."
#endif

#include <cmath>
#include <Arduino.h>
#include <HardwareTimer.h>

#if !defined(HAL_TIM_MODULE_ENABLED) || defined(HAL_TIM_MODULE_ONLY)
#  error "HAL_TIM_MODULE is not enabled!"
#endif

constexpr uint32_t SERVO_MIN_PULSE_WIDTH     = 544;   // shortest pulse (µs)
constexpr uint32_t SERVO_MAX_PULSE_WIDTH     = 2400;  // longest pulse (µs)
constexpr uint32_t SERVO_CEN_PULSE_WIDTH     = (SERVO_MIN_PULSE_WIDTH + SERVO_MAX_PULSE_WIDTH) / 2;
constexpr uint32_t SERVO_DEFAULT_PULSE_WIDTH = 1500;   // default center pulse (µs)
constexpr uint32_t SERVO_REFRESH_INTERVAL    = 20000;  // frame period (µs), ~50 Hz

constexpr uint32_t SERVO_MAX_PER_TIMER = 10;  // max servos per timer

namespace detail {
  struct servo_t {
    uint32_t      pin{};
    GPIO_TypeDef *port{};
    uint32_t      mask{};
    bool          active{};
    uint16_t      pulse_us{SERVO_DEFAULT_PULSE_WIDTH};
    uint32_t      min_pulse{};
    uint32_t      max_pulse{};

    void writeMicroseconds(uint16_t us) {
      if (us < min_pulse) us = min_pulse;
      if (us > max_pulse) us = max_pulse;
      pulse_us = us;
    }
    void write(float deg) {
      if (deg < 0.f) deg = 0.f;
      if (deg > 180.f) deg = 180.f;
      const float span = static_cast<float>(max_pulse - min_pulse);
      pulse_us         = static_cast<uint16_t>(
        std::lround(static_cast<float>(min_pulse) + (deg / 180.f) * span));
    }
  };
}  // namespace detail

class STM32ServoList {
private:
  HardwareTimer          timer_;  // constructed from TIMx in ctor
  bool                   timer_active_{false};
  static STM32ServoList *s_active_;

  // Registered servos
  detail::servo_t servos_[SERVO_MAX_PER_TIMER]{};
  size_t          size_{0};

  // Per-frame working state
  size_t   order_[SERVO_MAX_PER_TIMER]{};           // indices of active servos, sorted by width
  uint16_t pulse_us_frame_[SERVO_MAX_PER_TIMER]{};  // snapshot of pulse widths (aligned to order_)
  size_t   n_active_{0};
  size_t   next_idx_{0};
  uint16_t last_edge_us_{0};

  enum class phase_t : uint8_t { FALLS,
                                 TAIL };
  phase_t phase_{phase_t::FALLS};

public:
  explicit STM32ServoList(TIM_TypeDef *timer) : timer_(timer) {}

  // Register a servo pin; returns true on success
  bool attach(uint32_t pin,
              uint32_t min_pulse = SERVO_MIN_PULSE_WIDTH,
              uint32_t max_pulse = SERVO_MAX_PULSE_WIDTH,
              uint32_t value     = SERVO_CEN_PULSE_WIDTH);

  void enable(const size_t channel) {
    if (channel < size_) servos_[channel].active = true;
  }
  void disable(const size_t channel) {
    if (channel < size_) servos_[channel].active = false;
  }

  detail::servo_t &operator[](const size_t index) {
    return servos_[index];
  }

  [[nodiscard]] size_t size() const { return size_; }

private:
  void        initTimer();           // 1 MHz base; delta scheduling
  void        frameStartPrepare_();  // latch/sort/start frame
  inline void setDeltaUs_(uint16_t delta_us);

  static void servoElapsedCallback_() {
    if (s_active_) s_active_->onTimerISR_();
  }
  void onTimerISR_();
};

#endif  // STM32SERVO_H
