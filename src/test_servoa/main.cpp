#include <Arduino.h>
#include <Servo.h>
#include "UserPins.h"
#include "config/Main/UserConfig.h"

Servo servo_a;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("[TEST_SERVOA] boot");

  servo_a.attach(USER_GPIO_SERVO_A, RA_SERVO_MIN, RA_SERVO_MAX);
  servo_a.write(90);
  Serial.println("Servo A ready at 90 deg. Send angle (0-180):");
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();

    float angle = s.toFloat();

    if (angle >= 0.0f && angle <= 180.0f) {
      servo_a.write(angle);
      Serial.print("[CMD] angle=");
      Serial.println(angle, 1);
    } else {
      Serial.println("[ERR] out of range (0-180)");
    }
  }
}
