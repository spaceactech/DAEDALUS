#include <Arduino.h>
#include "UserPins.h"
#include "Controlling.h"

HardwareSerial ServoSerial(USER_GPIO_Half);

ServoDriver driver;

double target_deg[3] = {0.0, 0.0, 0.0};

static constexpr uint16_t SPD    = 500;
static constexpr uint8_t  ACC    = 50;
static constexpr uint16_t TORQUE = 1000;

static inline s16 deg_to_pos(double deg) {
  deg = constrain(deg, 0.0, 360.0);
  return static_cast<s16>((deg / 360.0) * 4096.0);
}

static inline double pos_to_deg(int raw) {
  return (raw / 4096.0) * 360.0;
}

// Commands: t <deg>  t0 <deg>  t1 <deg>  t2 <deg>
static void parse_serial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();

  int sep = s.indexOf(' ');
  if (sep < 0) return;

  String key = s.substring(0, sep);
  double val = s.substring(sep + 1).toDouble();

  auto go = [&](int idx, double d) {
    target_deg[idx] = d;
    int ack         = driver.hlscl.WritePosEx(ServoDriver::IDS[idx], deg_to_pos(d), SPD, ACC, TORQUE);
    Serial.printf("[GO] servo %d -> %.2f deg  ACK=%d\n", ServoDriver::IDS[idx], d, ack);
  };

  if (key == "t") {
    for (int i = 0; i < 3; i++) go(i, val);
  } else if (key == "t0")
    go(0, val);
  else if (key == "t1")
    go(1, val);
  else if (key == "t2")
    go(2, val);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("[TEST_SERVO] boot");

  ServoSerial.begin(1'000'000);
  ServoSerial.setTimeout(20);
  driver.hlscl.pSerial = &ServoSerial;
  driver.hlscl.syncReadBegin(sizeof(ServoDriver::IDS), sizeof(driver.rxPacket), 10);

  for (int i = 0; i < 3; i++) {
    if (driver.hlscl.Ping(ServoDriver::IDS[i]) == ServoDriver::IDS[i])
      Serial.printf("[PING] servo %d OK\n", ServoDriver::IDS[i]);
    else
      Serial.printf("[PING] servo %d FAIL\n", ServoDriver::IDS[i]);

    driver.hlscl.ServoMode(ServoDriver::IDS[i]);
    driver.hlscl.EnableTorque(ServoDriver::IDS[i], 1);
  }

  Serial.println("Cmds: t <deg>  t0 <deg>  t1 <deg>  t2 <deg>  (0-360)");
  Serial.println("T(s),TGT0,CUR0,TGT1,CUR1,TGT2,CUR2");
}

void loop() {

  parse_serial();

  double t = millis() * 0.001;
  Serial.print(t, 2);
  for (int i = 0; i < 3; i++) {
    int    raw     = driver.hlscl.ReadPos(ServoDriver::IDS[i]);
    double cur_deg = (raw >= 0) ? pos_to_deg(raw) : -1.0;
    Serial.print(',');
    Serial.print(target_deg[i], 2);
    Serial.print(',');
    Serial.print(cur_deg, 2);
  }
  Serial.println();
}
