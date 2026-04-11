#include <Arduino_Extended.h>
#include "UserPins.h"
#include "INA236.h"

TwoWire i2c3(PB7, PB8);
TwoWire i2c4(USER_GPIO_I2C4_SDA, USER_GPIO_I2C4_SCL);
INA236  INA(0x40, &i2c4);

void setup() {
  Serial.begin();
  i2c3.begin();
  Serial.begin();

  i2c4.setSDA(USER_GPIO_I2C4_SDA);
  i2c4.setSCL(USER_GPIO_I2C4_SCL);
  i2c4.setClock(400000);
  i2c4.begin();

}

void loop() {
  Serial.println("Hello");
  i2c_detect(Serial, i2c4, 0x00, 127);
  delay(1000);

}