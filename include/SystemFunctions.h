#ifndef ROCKET_AVIONICS_TEMPLATE_SYSTEMFUNCTIONS_H
#define ROCKET_AVIONICS_TEMPLATE_SYSTEMFUNCTIONS_H

#include <Arduino.h>

extern void ReadIMU();

extern void ReadAltimeter();

extern void ReadGNSS();

extern void ActivateDeployment(size_t index);

/**
 * Retain the state and auto-cutoff
 */
extern void RetainDeployment();

extern void AutoZeroAlt();

namespace internal {
  inline int32_t read_vref() {
    return __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_16B);
  }

  inline int32_t read_cpu_temp(const int32_t VRef) {
    return __LL_ADC_CALC_TEMPERATURE(VRef, analogRead(ATEMP), LL_ADC_RESOLUTION_16B);
  }
}  // namespace internal

inline int32_t ReadCPUTemp() {
  return internal::read_cpu_temp(internal::read_vref());
}

#endif  //ROCKET_AVIONICS_TEMPLATE_SYSTEMFUNCTIONS_H
