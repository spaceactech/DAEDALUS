#include <SCServo.h>
#include "UserPins.h"
#include <Arduino_Extended.h>

SMS_STS        sms_sts;
HardwareSerial HalfSerial(USER_GPIO_Half);
uint8_t        SERVO_IDS[]     = {1, 2, 3};
byte           servo_accels[2] = {50, 50};
uint8_t        rxPacket[4];

int Position;
int Speed;

void setup() {
  Serial.begin(115200);
  HalfSerial.begin(1'000'000);
  sms_sts.pSerial = &HalfSerial;
  delay(1000);

  // sms_sts.unLockEprom(1);               //打开EPROM保存功能
  // sms_sts.writeByte(1, SMS_STS_ID, 3);  //ID
  // sms_sts.LockEprom(3);                 //关闭EPROM保存功能

  sms_sts.syncReadBegin(sizeof(SERVO_IDS), sizeof(rxPacket), 5);
  for (size_t i = 0; i < sizeof(SERVO_IDS); ++i) {
    sms_sts.WheelMode(SERVO_IDS[i]);
    sms_sts.WriteSpe(SERVO_IDS[i], 10000, 255);
  }
}

void loop() {
  sms_sts.syncReadPacketTx  (SERVO_IDS, sizeof(SERVO_IDS), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket));

  for (size_t i = 0; i < sizeof(SERVO_IDS); ++i) {
    sms_sts.syncReadPacketRx(SERVO_IDS[i], rxPacket);

    Position = sms_sts.syncReadRxPacketToWrod(15);
    Speed    = sms_sts.syncReadRxPacketToWrod(15);

    Serial.print("ID:");
    Serial.println(SERVO_IDS[i]);
    Serial.print("Position:");
    Serial.println(Position);
    Serial.print("Speed:");
    Serial.println(Speed);


    // int ID = sms_sts.Ping(servo_ids[i]);
    // if (!sms_sts.getLastError()) {
    //   Serial.print("Servo ID:");
    //   Serial.println(servo_ids[i], DEC);
    //   delay(100);
    // } else {
    //   Serial.println("Ping servo ID error!");
    //   delay(2000);
    // }
  }

  delay(10);
}