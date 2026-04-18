#include <Arduino.h>
#include "UserPins.h"
#include <SCServo.h>

static constexpr uint8_t  IDS[3]  = {1, 2, 3};
static constexpr uint8_t  ACC     = 50;
static constexpr uint16_t TORQUE  = 1000;

HardwareSerial ServoSerial(USER_GPIO_Half);
HLSCL hlscl;

int16_t cmd_speed[3] = {0, 0, 0};

// Commands:
//   s <spd>    — set all servos speed (-3500 to 3500)
//   s0 <spd>   — servo 1 only
//   s1 <spd>   — servo 2 only
//   s2 <spd>   — servo 3 only
//   stop       — stop all
static void parse_serial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();

  if (s == "stop") {
    for (int i = 0; i < 3; i++) {
      cmd_speed[i] = 0;
      hlscl.WriteSpe(IDS[i], 0, ACC, TORQUE);
    }
    Serial.println("[STOP] all");
    return;
  }

  int sep = s.indexOf(' ');
  if (sep < 0) return;

  String  key = s.substring(0, sep);
  int16_t val = static_cast<int16_t>(s.substring(sep + 1).toInt());

  auto go = [&](int idx, int16_t spd) {
    cmd_speed[idx] = spd;
    int ack = hlscl.WriteSpe(IDS[idx], spd, ACC, TORQUE);
    Serial.printf("[SPD] servo %d -> %d  ACK=%d\n", IDS[idx], spd, ack);
  };

  if      (key == "s")  { for (int i = 0; i < 3; i++) go(i, val); }
  else if (key == "s0") go(0, val);
  else if (key == "s1") go(1, val);
  else if (key == "s2") go(2, val);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("[TEST_SERVO_RAW] boot");

  ServoSerial.begin(1'000'000);
  ServoSerial.setTimeout(20);
  hlscl.pSerial = &ServoSerial;

  for (int i = 0; i < 3; i++) {
    if (hlscl.Ping(IDS[i]) == IDS[i])
      Serial.printf("[PING] servo %d OK\n", IDS[i]);
    else
      Serial.printf("[PING] servo %d FAIL\n", IDS[i]);

    hlscl.WheelMode(IDS[i]);
    hlscl.EnableTorque(IDS[i], 1);
  }

  Serial.println("Cmds: s <spd>  s0/s1/s2 <spd>  stop");
  Serial.println("ID,POS_DEG,SPEED,LOAD,VOLTAGE,TEMP,CURRENT");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 100) {
    parse_serial();
    return;
  }
  last = millis();

  parse_serial();

  for (int i = 0; i < 3; i++) {
    if (hlscl.FeedBack(IDS[i]) < 0) {
      Serial.printf("%d,ERR\n", IDS[i]);
      continue;
    }

    int     pos     = hlscl.ReadPos(IDS[i]);
    int     speed   = hlscl.ReadSpeed(IDS[i]);
    int     load    = hlscl.ReadLoad(IDS[i]);
    int     voltage = hlscl.ReadVoltage(IDS[i]);
    int     temp    = hlscl.ReadTemper(IDS[i]);
    int     current = hlscl.ReadCurrent(IDS[i]);
    double  pos_deg = (pos >= 0) ? (pos / 4096.0) * 360.0 : -1.0;

    Serial.printf("%d,%.2f,%d,%d,%d,%d,%d\n",
                  IDS[i], pos_deg, speed, load, voltage, temp, current);
  }
}
