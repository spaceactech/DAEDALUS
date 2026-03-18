#ifndef HAL_RTOS_HPP
#define HAL_RTOS_HPP

#include <STM32FreeRTOS.h>
#include "./hal_timing.h"

#ifndef pdTICKS_TO_MS
#  define pdTICKS_TO_MS(xTicks) ((TickType_t) ((uint64_t) (xTicks) * 1000 / configTICK_RATE_HZ))
#endif

namespace hal {
  // UNUSED
  template<typename Exp>
  [[maybe_unused]] void unused(Exp &&exp) {
    static_cast<void>(exp);
  }
}  // namespace hal

namespace hal::rtos {
  namespace mon {
    constexpr size_t    MAX_MON = 32;
    inline osThreadId_t handles[MAX_MON];
    inline size_t       stacks[MAX_MON];
    inline size_t       num_handles = 0;
  }  // namespace mon

  // --- Tick/Time helpers -----------------------------------------------------

  inline uint32_t tick_hz() {
    const uint32_t f = osKernelGetTickFreq();
    return f ? f : 1000u;
  }

  constexpr uint32_t max_delay = osWaitForever;

  inline uint32_t to_tick(const uint32_t ms) {
    // Round up to avoid 0 for small ms when freq < 1000
    const uint64_t t = (static_cast<uint64_t>(ms) * tick_hz() + 999ull) / 1000ull;
    return static_cast<uint32_t>(t);
  }

  inline uint32_t to_ms(const uint32_t ticks) {
    const uint64_t ms = (static_cast<uint64_t>(ticks) * 1000ull) / tick_hz();
    return static_cast<uint32_t>(ms);
  }

  inline uint32_t max_delay_ms = to_ms(max_delay);

  /**
   * Delays the execution of the current task for a specified number of milliseconds.
   *
   * @param n The number of milliseconds to delay.
   */
  __attribute__((always_inline)) inline void delay_ms(const uint32_t n) {
    osDelay(n);
  }

  /**
   * Delays the execution for a specified number of microseconds.
   * Maybe FreeRTOS-safe
   *
   * @param n The duration of the delay in microseconds.
   */
  __attribute__((always_inline)) inline void delay_us(const uint32_t n) {
    if (n >= 1000000ul / tick_hz()) {
      delay_ms(n / 1000ul);
      hal::delay_us(n % 1000u);
      return;
    }
    hal::delay_us(n);
  }

  /**
   * Triggers a task yield operation.
   *
   * This function forces the scheduler to select a different task to run.
   * It should be used to explicitly yield the processor to other tasks.
   */
  __attribute__((always_inline)) inline void yield() {
    osThreadYield();
  }

  /**
   * Triggers a context switch from within an Interrupt Service Routine (ISR).
   *
   * This function ensures that a task switch occurs immediately after the
   * ISR completes if a higher priority task is ready to run.
   */
  __attribute__((always_inline)) inline void isr_yield([[maybe_unused]] const BaseType_t pxHigherPriorityTaskWoken) {
    // NOP
  }

  struct mutex_t {
    mutable osMutexId_t handle = nullptr;

    mutex_t() {
      handle = osMutexNew(nullptr);
    }

    bool acquire(const uint32_t wait_ms = max_delay_ms) const {
      const uint32_t timeout = (wait_ms == max_delay_ms) ? osWaitForever : wait_ms;
      return osMutexAcquire(handle, timeout) == osOK;
    }

    void vAcquire(const uint32_t wait_ms = max_delay_ms) const {
      unused(acquire(wait_ms));
    }

    bool release() const {
      return osMutexRelease(handle) == osOK;
    }

    void vRelease() const {
      unused(release());
    }

    template<typename Func>
    void exec(Func &&func) {
      if (acquire()) {
        func();
        vRelease();
      }
    }
  };

  /**
   * Represents a periodic delay utility for FreeRTOS tasks,
   * maintaining consistent timing intervals.
   */
  struct interval_delay {
  private:
    TickType_t freq;

  public:
    /**
     * Constructor
     *
     * @param interval_ms
     */
    explicit interval_delay(const TickType_t interval_ms)
        : freq(to_tick(interval_ms)) {}

    /**
     * Sets the interval in milliseconds.
     *
     * @param new_interval_ms The new interval in milliseconds to set.
     */
    void set_interval(const TickType_t new_interval_ms) {
      freq = to_tick(new_interval_ms);
    }

    /**
     * Retrieves the interval in ticks.
     *
     * @return The interval in ticks as a TickType_t value.
     */
    [[nodiscard]] constexpr TickType_t interval_ticks() const {
      return freq;
    }

    /**
     * FreeRTOS vTaskDelayUntil Wrapper
     *
     * Delays the task execution for a specified interval,
     * maintaining a consistent periodic rate.
     *
     */
    template<typename Func>
    void operator()(Func &&func) const {
      const uint32_t tick = osKernelGetTickCount();
      func();
      osDelayUntil(tick + freq);
    }

    template<typename Func>
    void operator()(Func &&func, void *arg) const {
      const uint32_t tick = osKernelGetTickCount();
      func(arg);
      osDelayUntil(tick + freq);
    }

    void operator()() const {
      const uint32_t tick = osKernelGetTickCount();
      osDelayUntil(tick + freq);
    }
  };

  template<size_t StackSizeWords>
  struct static_task_t {
    osThreadId_t handle                       = nullptr;
    alignas(8) uint32_t stack[StackSizeWords] = {};
    bool created                              = false;

    static_task_t() = default;

    void create(const osThreadFunc_t func,
                const char          *name,
                void                *arg,
                const osPriority_t   prio) {
      if (created) return;

      const osThreadAttr_t attr{
        .name       = name,
        .stack_mem  = stack,
        .stack_size = sizeof(stack),
        .priority   = prio};

      handle = osThreadNew(func, arg, &attr);
      if (handle) {
        created = true;
        if (mon::num_handles < mon::MAX_MON) {
          mon::stacks[mon::num_handles]  = StackSizeWords;
          mon::handles[mon::num_handles] = handle;
          ++mon::num_handles;
        }
      }
    }

    bool destroy() {
      if (!created) return false;
      osThreadTerminate(handle);
      handle  = nullptr;
      created = false;
      return true;
    }

    [[nodiscard]] bool pause() const {
      if (!created) return false;
      return osThreadSuspend(handle) == osOK;
    }

    [[nodiscard]] bool resume() const {
      if (!created) return false;
      return osThreadResume(handle) == osOK;
    }

    // --- Notifications: binary via thread flags (bit 0) ---------------------

    __attribute__((always_inline)) inline void notify() const {
      if (created) { osThreadFlagsSet(handle, 0x1u); }
    }

    __attribute__((always_inline)) inline void isr_notify() const {
      if (created) { osThreadFlagsSet(handle, 0x1u); }  // ISR-safe per CMSIS-RTOS2
    }
  };

  __attribute__((always_inline)) inline uint32_t wait_notification(const uint32_t wait_ms = max_delay_ms) {
    const uint32_t flags = osThreadFlagsWait(0x1u, osFlagsWaitAny, wait_ms);
    return (flags & 0x1u) ? 1u : 0u;  // emulate count==1 on success
  }

  struct notify_counter {
    osSemaphoreId_t sem = nullptr;
    notify_counter() { sem = osSemaphoreNew(0x7FFFFFFFu, 0, nullptr); }
    void     give() const { osSemaphoreRelease(sem); }
    void     isr_give() const { osSemaphoreRelease(sem); }  // ISR-safe
    uint32_t take(uint32_t wait_ms = max_delay_ms) const {
      return osSemaphoreAcquire(sem, wait_ms) == osOK ? 1u : 0u;
    }
  };

  template<typename T, size_t N>
  struct static_queue_t {
    osMessageQueueId_t handle = nullptr;
    // Provide control block + storage for static allocation
    alignas(8) uint8_t mq_cb[sizeof(void *) * 8]             = {};  // minimal CB space (impl-dependent)
    alignas(8) uint8_t storage[N * ((sizeof(T) + 3u) & ~3u)] = {};

    static_queue_t() {
      osMessageQueueAttr_t attr{};
      attr.name    = "q";
      attr.cb_mem  = mq_cb;
      attr.cb_size = sizeof(mq_cb);
      attr.mq_mem  = storage;
      attr.mq_size = sizeof(storage);
      handle       = osMessageQueueNew(N, sizeof(T), &attr);
    }

    bool enqueue(const T &data, uint32_t wait_ms = max_delay_ms) const {
      return osMessageQueuePut(handle, &data, 0, wait_ms) == osOK;
    }

    bool enqueue_nowait(const T &data) const { return enqueue(data, 0); }

    bool enqueue_front(const T &data, uint32_t wait_ms = max_delay_ms) const {
      // CMSIS-RTOS2 has no dedicated “send to front”; emulate via priority.
      // Lower 8-bit priority value: smaller means higher priority to leave first.
      // Use 0 for front, 1 for normal. Here we choose 0 to bias to front.
      return osMessageQueuePut(handle, &data, 0, wait_ms) == osOK;
    }

    bool enqueue_front_nowait(const T &data) const { return enqueue_front(data, 0); }

    bool dequeue(T &dst, uint32_t wait_ms = max_delay_ms) const {
      return osMessageQueueGet(handle, &dst, nullptr, wait_ms) == osOK;
    }

    bool dequeue_nowait(T &dst) const { return dequeue(dst, 0); }
  };

  /**
   * Infinite loop that repeatedly delays execution.
   *
   * This function is marked [[noreturn]], indicating it will never return
   * to the caller. It continuously executes an infinite loop, delaying
   * for 1 millisecond in each iteration.
   */
  [[noreturn]] inline void loop() {
    for (;;) {
      delay_ms(1);
    }
  }

  template<typename Func>
  /**
   * Executes a given function in an infinite loop without returning.
   *
   * @param func A callable object or function to be executed in each iteration of the loop.
   */
  [[noreturn]] void loop(Func &&func) {
    for (;;) {
      func();
      delay_ms(1);
    }
  }

  template<typename Func>
  /**
   * Infinite loop that repeatedly executes the provided function.
   *
   * @param func The function to be executed in the loop.
   * @param arg A pointer to be passed as an argument to the function.
   */
  [[noreturn]] void loop(Func &&func, void *arg) {
    for (;;) {
      func(arg);
      delay_ms(1);
    }
  }

  /**
   * Executes a periodic loop with a fixed interval and a user-provided function.
   * The loop will repeatedly call the provided function with the specified delay interval.
   *
   * @param interval_ms The interval in milliseconds between consecutive executions of the function.
   * @param func The callable object (e.g., lambda, function, functor) to be executed periodically.
   */
  template<typename Func>
  [[noreturn]] void interval_loop(const TickType_t interval_ms, Func &&func) {
    const interval_delay delay_until(interval_ms);
    for (;;) {
      delay_until(func);
    }
  }

  template<typename Func>
  [[noreturn]] void interval_loop(const TickType_t interval_ms, uint32_t &true_interval, Func &&func) {
    const interval_delay delay_until(interval_ms);
    uint32_t             last_ms = millis();
    for (;;) {
      delay_until([&]() -> void {
        const uint32_t now_ms = millis();
        true_interval         = now_ms - last_ms;
        last_ms               = now_ms;
        func();
      });
    }
  }

  /**
   * Infinite loop that repeatedly executes a given function at specified intervals.
   *
   * @param interval_ms The interval duration in milliseconds.
   * @param func The function to be invoked during each iteration of the loop.
   * @param arg A pointer to the argument passed to the function.
   */
  template<typename Func>
  [[noreturn]] void interval_loop(const TickType_t interval_ms, Func &&func, void *arg) {
    const interval_delay delay_until(interval_ms);
    for (;;) {
      delay_until(func, arg);
    }
  }

  /**
   * Executes a periodic loop with a fixed interval and a user-provided function.
   * The loop will repeatedly call the provided function with the specified delay interval.
   *
   * @param interval_ms The interval in milliseconds between consecutive executions of the function.
   * @param next_interval_func Next interval function
   * @param func The callable object (e.g., lambda, function, functor) to be executed periodically.
   */
  template<typename Func, typename RngFunc>
  [[noreturn]] void interval_loop(const TickType_t interval_ms, RngFunc &&next_interval_func, Func &&func) {
    interval_delay delay_until(interval_ms);
    for (;;) {
      delay_until(func);
      delay_until.set_interval(next_interval_func());
    }
  }

  /**
   * Infinite loop that repeatedly executes a given function at specified intervals.
   *
   * @param interval_ms The interval duration in milliseconds.
   * @param next_interval_func Next interval function
   * @param func The function to be invoked during each iteration of the loop.
   * @param arg A pointer to the argument passed to the function.
   */
  template<typename Func, typename RngFunc>
  [[noreturn]] void interval_loop(const TickType_t interval_ms, RngFunc &&next_interval_func, Func &&func, void *arg) {
    interval_delay delay_until(interval_ms);
    for (;;) {
      delay_until(func, arg);
      delay_until.set_interval(next_interval_func());
    }
  }

  /**
   * Executes a given section of code in a critical region, ensuring that the code
   * is executed with FreeRTOS spinlock and preventing interrupts.
   *
   * @param section Callable object or function to be executed within the critical section.
   */
  template<typename Func>
  void critical(Func &&section) {
#ifdef ESP_PLATFORM
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&mux);
#else
    taskENTER_CRITICAL();
#endif

    // Execute critical section
    section();

#ifdef ESP_PLATFORM
    taskEXIT_CRITICAL(&mux);
#else
    taskEXIT_CRITICAL();
#endif
  }

  inline struct {
    void initialize() {
      osKernelInitialize();
    }

    void start() {
      osKernelStart();

      // Should never reach here.
      while (true);
    }

    osThreadId_t create(const osThreadFunc_t func, const osThreadAttr_t &attr) {
      return osThreadNew(func, nullptr, &attr);
    }
  } scheduler;
}  // namespace hal::rtos

#endif  //HAL_RTOS_HPP
