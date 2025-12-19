/* ===== Includes ===== */
#include <Arduino.h>

#if __has_include("STM32FreeRTOS.h")
#  include "hal_rtos.h"
#endif

#include <SPI.h>
#include <ISM6HG256XSensor.h>
#include "UserPins.h"  // User's Pins Mapping

/* ===== Pins ===== */


/* ===== SPI + Sensor ===== */
SPIClass         spi1(USER_GPIO_SPI1_MOSI, USER_GPIO_SPI1_MISO, USER_GPIO_SPI1_SCK);
ISM6HG256XSensor imu(&spi1, USER_GPIO_ISM256_NSS);

/* ===== Data ===== */
ISM6HG256X_Axes_t imu_accel;
ISM6HG256X_Axes_t imu_gyro;

/* ===== RTOS Task to Read IMU ===== */
void CB_ReadIMU(void *) {
  hal::rtos::interval_loop(100ul, [&]() -> void {
    imu.Get_X_Axes(&imu_accel);
    imu.Get_G_Axes(&imu_gyro);
  });
}

/* ===== RTOS Task to Print IMU ===== */
void CB_PrintIMU(void *) {
  hal::rtos::interval_loop(200ul, [&]() -> void {
    Serial.print("ACC (mg): ");
    Serial.print(imu_accel.x);
    Serial.print(", ");
    Serial.print(imu_accel.y);
    Serial.print(", ");
    Serial.println(imu_accel.z);

    Serial.print("GYR (mdps): ");
    Serial.print(imu_gyro.x);
    Serial.print(", ");
    Serial.print(imu_gyro.y);
    Serial.print(", ");
    Serial.println(imu_gyro.z);
    Serial.println();
  });
}

/* ===== Register RTOS Threads ===== */
void UserThreads() {
  hal::rtos::scheduler.create(
    CB_ReadIMU,
    {.name = "ReadIMU", .stack_size = 4096, .priority = osPriorityHigh});

  hal::rtos::scheduler.create(
    CB_PrintIMU,
    {.name = "PrintIMU", .stack_size = 4096, .priority = osPriorityBelowNormal});
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  spi1.begin();

  if (!imu.begin()) {
    Serial.println("ISM6HG256X init FAILED");
    while (1);
  }
  Serial.println("ISM6HG256X init OK");

  imu.Enable_X();  // enable accelerometer
  imu.Enable_G();  // enable gyroscope

  hal::rtos::scheduler.initialize();
  UserThreads();
  hal::rtos::scheduler.start();
}

void loop() {
  // not used when RTOS is running
}
