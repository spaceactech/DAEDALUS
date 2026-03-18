#include <Arduino.h>

#if __has_include("STM32FreeRTOS.h")
#  define USE_FREERTOS 1
#endif

extern "C" void __real_delay(uint32_t ms);

#if defined(USE_FREERTOS) && USE_FREERTOS
#  include "hal_rtos.h"
extern "C" {
void __wrap_delay(const uint32_t ms) {
  osKernelGetState() == osKernelRunning
    ? (void) osDelay(ms)
    : __real_delay(ms);
}

static void __attribute__((used)) vPortNoOptimize() {
  vTaskSwitchContext();
}
}
#else  // NO FREERTOS
extern "C" void __wrap_delay(const uint32_t ms) {
  __real_delay(ms);
}
#endif