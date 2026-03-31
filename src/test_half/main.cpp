#include <SCServo.h>
#include "UserPins.h"
#include <Arduino_Extended.h>

HLSCL        hlscl;
HardwareSerial ServoSerial(USER_GPIO_Half);
uint8_t        SERVO_IDS[]     = {3};
byte           servo_accels[2] = {50, 50};
uint8_t        rxPacket[4];

int Position;
int Speed;

void setup() {
  Serial.begin(115200);
  ServoSerial.begin(1'000'000);
  hlscl.pSerial = &ServoSerial;
  delay(1000);

  delay(2000);
  for (size_t i = 0; i < sizeof(SERVO_IDS); ++i) {
    hlscl.WheelMode(SERVO_IDS[i]);
    hlscl.syncReadBegin(sizeof(SERVO_IDS), sizeof(rxPacket), 5);  //10*10*2=200us<5ms
  }
}

void loop() {
  for (uint8_t i = 0; i < sizeof(SERVO_IDS); i++) {
    hlscl.WriteSpe(SERVO_IDS[i], 3500, 50, 500);
    // Serial.println(SERVO_IDS[i]);
  }


  hlscl.syncReadPacketTx(SERVO_IDS, sizeof(SERVO_IDS), HLSCL_PRESENT_POSITION_L, sizeof(rxPacket));  //同步读指令包发送
  for (uint8_t i = 0; i < sizeof(SERVO_IDS); i++) {
    //接收ID[i]同步读返回包
    if (!hlscl.syncReadPacketRx(SERVO_IDS[i], rxPacket)) {
      Serial.print("ID:");
      Serial.println(SERVO_IDS[i]);
      Serial.println("sync read error!");
      continue;  //接收解码失败
    }

    Position = hlscl.ReadPos(SERVO_IDS[i]);  //解码两个字节 bit15为方向位,参数=0表示无方向位
    Speed    = hlscl.ReadSpeed(SERVO_IDS[i]);  //解码两个字节 bit15为方向位,参数=0表示无方向位
    Serial.print("ID:");
    Serial.println(SERVO_IDS[i]);
    Serial.print("Position:");
    Serial.println(Position);
    Serial.print("Speed:");
    Serial.println(Speed);
  }
}