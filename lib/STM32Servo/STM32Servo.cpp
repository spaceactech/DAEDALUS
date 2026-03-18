#include "./STM32Servo.h"

STM32ServoList *STM32ServoList::s_active_ = nullptr;

bool STM32ServoList::attach(const uint32_t pin,
                            const uint32_t min_pulse,
                            const uint32_t max_pulse,
                            const uint32_t value) {
  if (size_ >= SERVO_MAX_PER_TIMER) return false;

  pinMode(pin, OUTPUT);
  auto          *port = digitalPinToPort(pin);
  const uint32_t mask = digitalPinToBitMask(pin);

  servos_[size_] = detail::servo_t{
    .pin       = pin,
    .port      = port,
    .mask      = mask,
    .active    = true,
    .pulse_us  = static_cast<uint16_t>(SERVO_DEFAULT_PULSE_WIDTH),
    .min_pulse = static_cast<uint16_t>(min_pulse),
    .max_pulse = static_cast<uint16_t>(max_pulse)};
  servos_[size_].writeMicroseconds(static_cast<uint16_t>(value));

  // Publish before enabling timer to avoid ISR racing an incomplete entry
  ++size_;

  if (!timer_active_) {
    initTimer();
  }
  return true;
}

void STM32ServoList::initTimer() {
  s_active_ = this;

  // 1 MHz timebase ⇒ 1 tick = 1 µs
  uint32_t psc = timer_.getTimerClkFreq() / 1'000'000u;
  if (psc < 1) psc = 1;
  timer_.setPrescaleFactor(psc);

  // Any placeholder; we'll program the first Δt before starting
  timer_.setOverflow(1000, TICK_FORMAT);
  timer_.attachInterrupt(servoElapsedCallback_);
  timer_.setPreloadEnable(false);

  // Program first frame timing *before* starting the counter
  frameStartPrepare_();

  timer_.setCount(0);
  timer_.resume();

  timer_active_ = true;
}

void STM32ServoList::frameStartPrepare_() {
  // 1) Build active list
  n_active_ = 0;
  for (size_t i = 0; i < size_; ++i) {
    if (servos_[i].active) order_[n_active_++] = i;
  }

  // 2) Insertion sort by width (ascending)
  for (size_t i = 1; i < n_active_; ++i) {
    const size_t   k  = order_[i];
    const uint16_t wk = servos_[k].pulse_us;
    int            j  = static_cast<int>(i) - 1;
    while (j >= 0 && servos_[order_[static_cast<size_t>(j)]].pulse_us > wk) {
      order_[static_cast<size_t>(j) + 1] = order_[static_cast<size_t>(j)];
      --j;
    }
    order_[static_cast<size_t>(j) + 1] = k;
  }

  // 3) Snapshot pulse widths for this frame in sorted order
  for (size_t i = 0; i < n_active_; ++i) {
    pulse_us_frame_[i] = servos_[order_[i]].pulse_us;
  }

  // 4) Drive all active pins HIGH together
  for (size_t i = 0; i < n_active_; ++i) {
    const auto &s = servos_[order_[i]];
    s.port->BSRR  = s.mask;  // set
  }

  // 5) Reset frame state & schedule first fall (or full frame)
  next_idx_     = 0;
  last_edge_us_ = 0;

  if (n_active_ > 0) {
    const uint16_t t0 = pulse_us_frame_[0];
    setDeltaUs_(t0);  // Δt to first fall
    phase_ = phase_t::FALLS;
  } else {
    setDeltaUs_(SERVO_REFRESH_INTERVAL);
    phase_ = phase_t::TAIL;
  }
}

inline void STM32ServoList::setDeltaUs_(uint16_t delta_us) {
  if (delta_us == 0) delta_us = 1;            // avoid zero-period
  timer_.setOverflow(delta_us, TICK_FORMAT);  // 1 tick = 1 µs
  // No refresh() here — avoid immediate UG inside ISR
}

void STM32ServoList::onTimerISR_() {
  if (phase_ == phase_t::FALLS) {
    // Current scheduled fall time:
    const uint16_t t_curr = pulse_us_frame_[next_idx_];

    // Drop all servos that share this time in one go
    do {
      const size_t id = order_[next_idx_];
      const auto  &s  = servos_[id];
      s.port->BSRR    = (s.mask << 16);  // reset
      ++next_idx_;
    } while (next_idx_ < n_active_ && pulse_us_frame_[next_idx_] == t_curr);

    if (next_idx_ < n_active_) {
      const uint16_t t_next = pulse_us_frame_[next_idx_];
      setDeltaUs_(static_cast<uint16_t>(t_next - t_curr));  // Δt to next fall
    } else {
      last_edge_us_       = t_curr;
      const uint16_t tail = static_cast<uint16_t>(SERVO_REFRESH_INTERVAL - last_edge_us_);
      setDeltaUs_(tail);
      phase_ = phase_t::TAIL;
    }
    return;
  }

  // Frame tail elapsed -> start next frame
  frameStartPrepare_();
}
