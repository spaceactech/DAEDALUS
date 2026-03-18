#include <SCServo.h>
#include "UserPins.h"
#include <Arduino_Extended.h>

SMS_STS        sms_sts;
HardwareSerial HalfSerial(USER_GPIO_Half);
uint8_t        servo_ids[]     = {1, 2};
byte           servo_accels[2] = {50, 50};
uint8_t        rxPacket[4];

int Position;
int Speed;

void setup() {
  Serial.begin(115200);
  HalfSerial.begin(115200);
  sms_sts.pSerial = &HalfSerial;
  delay(1000);

  sms_sts.syncReadBegin(sizeof(servo_ids), sizeof(rxPacket), 5);
  for (size_t i = 0; i < sizeof(servo_ids); ++i) {
    sms_sts.WheelMode(i);
    sms_sts.WriteSpe(i, 1000, servo_accels[i]);
  }
}

void loop() {
  sms_sts.syncReadPacketTx(servo_ids, sizeof(servo_ids), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket));

  for (size_t i = 0; i < sizeof(servo_ids); ++i) {
    if (!sms_sts.syncReadPacketRx(servo_ids[i], rxPacket)) {
      Serial.print("ID:");
      Serial.println(servo_ids[i]);
      Serial.println("sync read error!");
      continue;
    }

    Position = sms_sts.syncReadRxPacketToWrod(15);
    Speed    = sms_sts.syncReadRxPacketToWrod(15);

    Serial.print("ID:");
    Serial.println(servo_ids[i]);
    Serial.print("Position:");
    Serial.println(Position);
    Serial.print("Speed:");
    Serial.println(Speed);
  }
  delay(10);
}