/* BEGIN INCLUDE SYSTEM LIBRARIES */
#include <Arduino.h>  // Arduino Framework
#include "UserPins.h"     // User's Pins Mapping
#include "Arduino_Extended.h"

TwoWire  i2c1(USER_GPIO_I2C1_SDA, USER_GPIO_I2C1_SCL);

void setup() {
  Serial.begin(115200);
  i2c1.begin();
}

void loop() {
  i2c_detect(Serial, i2c1, 0x00, 127);
  delay(1000);
}