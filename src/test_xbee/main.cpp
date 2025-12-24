/* BEGIN INCLUDE SYSTEM LIBRARIES */
#include <Arduino.h>   // Arduino Framework
#include "UserPins.h"  // User's Pins Mapping
#include "Arduino_Extended.h"

TwoWire i2c1(USER_GPIO_I2C1_SDA, USER_GPIO_I2C1_SCL);

void setup() {
  Serial.begin(115200);
  i2c1.begin();
}

void loop() {
  i2c_detect(Serial, i2c1, 0x00, 127);
  delay(1000);
}

// #include <Arduino.h>
// #include "UserPins.h"

// HardwareSerial XBeeSerial(USER_GPIO_XBEE_RX, USER_GPIO_XBEE_TX);

// void setup() {
//   Serial.begin(115200);
//   XBeeSerial.begin(115200);   // or 115200
//   delay(2000);
//   Serial.println("XBee receiver ready");
// }

// void loop() {
//   while (XBeeSerial.available()) {
//     Serial.write(XBeeSerial.read());
//   }
// }
