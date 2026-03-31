#include <Arduino_Extended.h>
#include "UserPins.h"
#include "INA236.h"

TwoWire i2c4(USER_GPIO_I2C4_SDA, USER_GPIO_I2C4_SCL);
INA236  INA(0x40, &i2c4);

void setup() {
  Serial.begin();
  // pinMode(USER_GPIO_SPI1_SCK, OUTPUT);
  // pinMode(USER_GPIO_SPI1_MISO, OUTPUT);
  // pinMode(USER_GPIO_SPI1_MOSI, OUTPUT);
  // pinMode(USER_GPIO_BMP581_INT, OUTPUT);
  // pinMode(USER_GPIO_BMP581_NSS, OUTPUT);

  // pinMode(USER_GPIO_ISM256_INT1, OUTPUT);
  // pinMode(USER_GPIO_ISM256_INT2, OUTPUT);
  // pinMode(USER_GPIO_ISM256_NSS, OUTPUT);

  i2c4.setSDA(USER_GPIO_I2C4_SDA);
  i2c4.setSCL(USER_GPIO_I2C4_SCL);
  i2c4.setClock(400000);
  i2c4.begin();

  INA.setMaxCurrentShunt(1, 0.002);

  Serial.println("POWER2 = busVoltage x current\n");
  Serial.println("BUS\tCURRENT\tPOWER\tPOWER2\tDELTA");
}

void loop() {
  // i2c_detect(Serial, i2c4, 0x00, 127);
  // delay(1000);
  // digitalToggle(USER_GPIO_SPI1_SCK);
  // digitalToggle(USER_GPIO_SPI1_MISO);
  // digitalToggle(USER_GPIO_SPI1_MOSI);
  // digitalToggle(USER_GPIO_BMP581_INT);
  // digitalToggle(USER_GPIO_BMP581_NSS);

  // digitalToggle(USER_GPIO_ISM256_INT1);
  // digitalToggle(USER_GPIO_ISM256_INT2);
  // digitalToggle(USER_GPIO_ISM256_NSS);
  // delay(5000);

  float bv = INA.getBusVoltage();
  //  float sv = INA.getShuntVoltage_mV();
  float cu = INA.getCurrent_mA();
  float po = INA.getPower_mW();

  Serial.print(bv, 3);
  Serial.print("\t");
  Serial.print(cu, 3);
  Serial.print("\t");
  Serial.print(po, 3);
  Serial.print("\t");
  Serial.print(bv * cu, 3);
  Serial.print("\t");
  Serial.print((po - bv * cu), 3);
  Serial.println();
  delay(100);
}