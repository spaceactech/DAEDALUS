#include <SCServo.h>
#include "UserPins.h"
#include <Arduino.h>

SMS_STS        sms_sts;
HardwareSerial HalfSerial(USER_GPIO_Half, USER_GPIO_Half);
uint8_t ID[] = {1, 2};
byte    ACC[2];
uint8_t rxPacket[4];

int16_t Position;
int16_t Speed;

void setup() {
  Serial.begin(115200);       //sms舵机波特率115200
  HalfSerial.begin(1000000);  //sts舵机波特率1000000
  HalfSerial.setHalfDuplex();
  sms_sts.pSerial = &HalfSerial;
  delay(1000);
  sms_sts.WheelMode(1);
  sms_sts.WheelMode(2);                                    //舵机ID1切换至电机恒速模式
  sms_sts.syncReadBegin(sizeof(ID), sizeof(rxPacket), 5);  //10*10*2=200us<5ms
  HalfSerial.enableHalfDuplexRx();
  delay(150);
}

void loop() {
  sms_sts.WriteSpe(1, 1000, 50);
  sms_sts.WriteSpe(2, 1000, 50);

  HalfSerial.flush();
  delay(150);
  HalfSerial.enableHalfDuplexRx();
  sms_sts.syncReadPacketTx(ID, sizeof(ID), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket));  //同步读指令包发送
  for (uint8_t i = 0; i < sizeof(ID); i++) {
    //接收ID[i]同步读返回包
    if (!sms_sts.syncReadPacketRx(ID[i], rxPacket)) {
      Serial.print("ID:");
      Serial.println(ID[i]);
      Serial.println("sync read error!");
      continue;  //接收解码失败
    }
    Position = sms_sts.syncReadRxPacketToWrod(15);  //解码两个字节 bit15为方向位,参数=0表示无方向位
    Speed    = sms_sts.syncReadRxPacketToWrod(15);  //解码两个字节 bit15为方向位,参数=0表示无方向位
    Serial.print("ID:");
    Serial.println(ID[i]);
    Serial.print("Position:");
    Serial.println(Position);
    Serial.print("Speed:");
    Serial.println(Speed);
  }
  delay(10);
}