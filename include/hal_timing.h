#ifndef HAL_TIMING_HPP
#define HAL_TIMING_HPP

#include <Arduino.h>

namespace hal {
  inline auto millis() {
    return ::millis();
  }

  inline auto micros() {
    return ::micros();
  }

  inline void delay_ms(const uint32_t n) {
    delay(n);
  }

  inline void delay_us(const uint32_t n) {
    delayMicroseconds(n);
  }
}  // namespace hal

#endif  //HAL_TIMING_HPP
