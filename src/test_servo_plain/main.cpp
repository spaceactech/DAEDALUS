#include <Arduino.h>
#include <SCServo.h>
#include "UserPins.h"

SMS_STS sms_sts;

static constexpr uint8_t  IDS[]    = {1, 2, 3};
static constexpr uint8_t  N_SERVO  = sizeof(IDS);
static constexpr int16_t  TEST_SPD = 2500;
static constexpr uint8_t  ACC      = 50;

HardwareSerial ServoSerial(USER_GPIO_Half);

void setup() {
  Serial.begin(460800);
  delay(2000);
  Serial.println("[TEST_SERVO_PLAIN] boot");

  ServoSerial.begin(1'000'000);
  ServoSerial.setTimeout(5);
  sms_sts.pSerial = &ServoSerial;

  Serial.println("Commands: r=read all  f=forward  b=backward  s=stop");
}

void loop() {
  // -- Read and print position of all servos every 200 ms --
  static uint32_t last_print = 0;
  if (millis() - last_print >= 200) {
    last_print = millis();
    for (uint8_t i = 0; i < N_SERVO; i++) {
      int16_t pos = sms_sts.ReadPos(IDS[i]);
      Serial.print("ID"); Serial.print(IDS[i]);
      Serial.print(" pos=");
      if (pos < 0)
        Serial.print("ERR");
      else
        Serial.print(pos);
      Serial.print("  ");
    }
    Serial.println();
  }

  // -- Serial command interface --
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'f') {
      Serial.println("[CMD] forward");
      for (uint8_t i = 0; i < N_SERVO; i++)
        sms_sts.WriteSpe(IDS[i], TEST_SPD, ACC);
    } else if (cmd == 'b') {
      Serial.println("[CMD] backward");
      for (uint8_t i = 0; i < N_SERVO; i++)
        sms_sts.WriteSpe(IDS[i], -TEST_SPD, ACC);
    } else if (cmd == 's') {
      Serial.println("[CMD] stop");
      for (uint8_t i = 0; i < N_SERVO; i++)
        sms_sts.WriteSpe(IDS[i], 0, ACC);
    }
  }
}
