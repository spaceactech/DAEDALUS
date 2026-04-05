#include <Arduino.h>
#include <Servo.h>
#include "UserPins.h"
constexpr int SERVO_MIN = 500;
constexpr int SERVO_MAX = 2950;
constexpr int SERVO_CEN = (SERVO_MIN + SERVO_MAX) / 2;

Servo servo_a;

String inputString    = "";
bool   stringComplete = false;

void setup() {
  // while (1) {
  //   delay(1);
  // }
  delay(10000);
  pinMode(USER_GPIO_BUZZER, OUTPUT);
  digitalWrite(USER_GPIO_BUZZER, 0);

  // servo_a.attach(USER_GPIO_SERVO_B, SERVO_MIN, SERVO_MAX, SERVO_MAX);
  // Serial.begin();
  // inputString.reserve(10);
  digitalWrite(USER_GPIO_BUZZER, 1);
  delay(2000);
  Serial.println("Hello!");
  digitalWrite(USER_GPIO_BUZZER, 0);

  servo_a.write(0);
}

void loop() {
  // if (stringComplete) {
  //   int angle = inputString.toInt();

  //   // constrain angle to valid range
  //   angle = constrain(angle, 0, 180);

  //   servo_a.write(angle);

  //   Serial.print("Servo moved to: ");
  //   Serial.println(angle);

  //   // clear input
  //   inputString    = "";
  //   stringComplete = false;
  // }

  servo_a.write(25);
  delay(1000);
  // servo_a.write(0);
  // delay(1000);
}

// this runs when serial data arrives
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char) Serial.read();

    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
