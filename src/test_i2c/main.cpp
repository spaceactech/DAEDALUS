#include <Arduino_Extended.h>

TwoWire i2c3(PB7, PB8);

void setup() {
    Serial.begin();
    i2c3.begin();
}

void loop() {
    Serial.println("Hello");

    i2c_detect(Serial, i2c3, 0x00, 127);
    delay(1000);
}