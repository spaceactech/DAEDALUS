#include <Arduino.h>
#include <Servo.h>
#include "UserPins.h"
#include "config/Main/UserConfig.h"

Servo servo_b;

void setup() {
  Serial.begin(115200);
  delay(2000);

  servo_b.attach(USER_GPIO_SERVO_B, RA_SERVO_MIN, RA_SERVO_MAX, RA_SERVO_MAX);
  Serial.println("Servo B test ready. Send angle (0-180):");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    const float angle = input.toFloat();

    if (angle >= 0.0f && angle <= 180.0f) {
      servo_b.write(angle);
      Serial.print("Servo B -> ");
      Serial.println(angle);
    } else {
      Serial.println("Out of range (0-180)");
    }
  }
}
