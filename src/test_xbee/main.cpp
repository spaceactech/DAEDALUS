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

// #include <Arduino.h>
// #include "UserPins.h"  // User's Pins Mapping


// HardwareSerial XBeeSerial(USER_GPIO_XBEE_RX, USER_GPIO_XBEE_TX);

// void setup() {
//   Serial.begin(115200);
//   // Configure XBee RESET pin
//   pinMode(USER_GPIO_XBEE_NRST, OUTPUT);

//   // Hold XBee in reset (active LOW)
//   digitalWrite(USER_GPIO_XBEE_NRST, 0);
//   delay(50);  // >=10 ms required, 50 ms safe

//   digitalWrite(USER_GPIO_XBEE_NRST, 0.5);

//   XBeeSerial.begin(115200);
// }

// void loop() {
//   XBeeSerial.println("Hello");
//   Serial.println("TX");

//   delay(1000);
// }