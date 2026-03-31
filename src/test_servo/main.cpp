#include <Arduino.h>
#include <Servo.h>

constexpr int SERVO_MIN = 500;
constexpr int SERVO_MAX = 2450;
// constexpr int SERVO_CEN = (SERVO_MIN + SERVO_MAX) / 2;

Servo servo_a;

void setup() {
  pinMode(PC6, OUTPUT);
  digitalWrite(PC6, 0);

  servo_a.attach(PB6, SERVO_MIN, SERVO_MAX, SERVO_MAX);
  Serial.begin();

  delay(3000);
  digitalWrite(PC6, 1);
  Serial.println("Hello!");
}

void loop() {
  servo_a.write(0);
  delay(100);
  servo_a.write(180);
}

