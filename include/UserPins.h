#ifndef ROCKET_AVIONICS_TEMPLATE_USERPINS_H
#define ROCKET_AVIONICS_TEMPLATE_USERPINS_H

#include <Arduino.h>

constexpr uint32_t USER_GPIO_LED    = PC14;
constexpr uint32_t USER_GPIO_BUZZER = PC15;

constexpr uint32_t USER_GPIO_SERVO_A = PA3;
constexpr uint32_t USER_GPIO_SERVO_B = PB9;

constexpr uint32_t USER_GPIO_SPI1_SCK  = PA5;
constexpr uint32_t USER_GPIO_SPI1_MISO = PA6;
constexpr uint32_t USER_GPIO_SPI1_MOSI = PA7;

constexpr uint32_t USER_GPIO_I2C4_SDA = PB7_ALT1;
constexpr uint32_t USER_GPIO_I2C4_SCL = PB8_ALT1;

constexpr uint32_t USER_GPIO_GPS_NRST = PH1;

constexpr uint32_t USER_GPIO_SDIO_CMD  = PD2;
constexpr uint32_t USER_GPIO_SDIO_CK   = PC12;
constexpr uint32_t USER_GPIO_SDIO_DAT0 = PB13;
constexpr uint32_t USER_GPIO_SDIO_DAT1 = PC9;
constexpr uint32_t USER_GPIO_SDIO_DAT2 = PC10;
constexpr uint32_t USER_GPIO_SDIO_DAT3 = PC11;

constexpr uint32_t USER_GPIO_BMP581_INT = PA9;
constexpr uint32_t USER_GPIO_BMP581_NSS = PA4;

constexpr uint32_t USER_GPIO_ISM256_INT1 = PH0;
constexpr uint32_t USER_GPIO_ISM256_INT2 = PA8;
constexpr uint32_t USER_GPIO_ISM256_NSS  = PB12;

constexpr uint32_t USER_GPIO_BNO_INT1 = PC4;
constexpr uint32_t BNO08X_RESET = PC0;
#define BNO08X_ADDR  0x4B

constexpr uint32_t M10S_RESET = PH1;

constexpr uint32_t USER_GPIO_XBEE_RX   = PB5;
constexpr uint32_t USER_GPIO_XBEE_TX   = PB_6_ALT1;
constexpr uint32_t USER_GPIO_XBEE_NRST = PB1;

constexpr uint32_t USER_GPIO_PWM2 = PB9;
constexpr uint32_t USER_GPIO_PWM3 = PA3;
constexpr uint32_t USER_GPIO_Half = PB4;

#endif  //ROCKET_AVIONICS_TEMPLATE_USERPINS_H
