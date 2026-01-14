#include <Arduino.h>
#include <Servo.h>
#include "UserPins.h"  // User's Pins Mapping

Servo myServo;
int   servoPin = USER_GPIO_SERVO_A;

int angle;

struct SERVO {
  uint32_t pin;
  SERVO(uint32_t pin) : pin(pin) {
    pinMode(pin, OUTPUT);
  }

  void write(int angle) {
    const int cpw = map(angle, 0, 180, 500, 2500);
    for (size_t i = 0; i < max(5, 20); ++i) {
      digitalWrite(pin, HIGH);
      delayMicroseconds(cpw);
      digitalWrite(pin, LOW);
      delayMicroseconds(20000);
    }
  }
};

SERVO servo_a(USER_GPIO_SERVO_A);

void setup() {
  Serial.begin();
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // read line until Enter key
    input.trim();                                 // remove spaces/newline

    if (input.length() > 0) {
      int newAngle = input.toInt();  // convert to integer

      if (newAngle >= 0 && newAngle <= 180) {
        servo_a.write(newAngle);
        Serial.print("Servo moved to: ");
        Serial.println(newAngle);
      } else if (newAngle == 200) {
        servo_a.write(60);
        for (;;) {
          for (angle = 0; angle <= 180; angle += 10) {
            servo_a.write(angle);
            delay(20);  // หน่วงเวลาเพื่อให้เซอร์โวหมุนตามทัน
          }
          for (angle > 180; angle -= 10;) {
            servo_a.write(angle);
            delay(20);
          }
        }
      } else {
        Serial.println("Invalid! Enter 0 - 180.");
      }
    }
  }
}
