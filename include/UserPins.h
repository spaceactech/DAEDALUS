#ifndef ROCKET_AVIONICS_TEMPLATE_USERPINS_H
#define ROCKET_AVIONICS_TEMPLATE_USERPINS_H

#include <Arduino.h>

constexpr uint32_t USER_GPIO_LED     = PC14;
constexpr uint32_t USER_GPIO_BUZZER  = PB0;

constexpr uint32_t USER_GPIO_SERVO_A = PA2;
constexpr uint32_t USER_GPIO_SERVO_B = PA8;

constexpr uint32_t USER_GPIO_SPI1_SCK  = PA5;
constexpr uint32_t USER_GPIO_SPI1_MISO = PA6;
constexpr uint32_t USER_GPIO_SPI1_MOSI = PA7;

constexpr uint32_t USER_GPIO_I2C1_SDA = PB7;
constexpr uint32_t USER_GPIO_I2C1_SCL = PB6;

constexpr uint32_t USER_GPIO_SDIO_CMD  = PD2;
constexpr uint32_t USER_GPIO_SDIO_CK   = PC12;
constexpr uint32_t USER_GPIO_SDIO_DAT0 = PB13;
constexpr uint32_t USER_GPIO_SDIO_DAT1 = PC9;
constexpr uint32_t USER_GPIO_SDIO_DAT2 = PC10;
constexpr uint32_t USER_GPIO_SDIO_DAT3 = PC11;

constexpr uint32_t USER_GPIO_BMP581_INT = PA3;
constexpr uint32_t USER_GPIO_BMP581_NSS = PA4;

constexpr uint32_t USER_GPIO_ISM256_INT = PH0;
constexpr uint32_t USER_GPIO_ISM256_NSS = PB12;

constexpr uint32_t USER_GPIO_XBEE_RX   = PA1;
constexpr uint32_t USER_GPIO_XBEE_TX   = PA0;
constexpr uint32_t USER_GPIO_XBEE_NRST = PB9;
#endif  //ROCKET_AVIONICS_TEMPLATE_USERPINS_H
