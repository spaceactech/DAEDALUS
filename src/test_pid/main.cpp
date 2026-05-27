#include <Arduino.h>
#include "UserPins.h"
#include "Controlling.h"

HardwareSerial ServoSerial(USER_GPIO_Half);

Controller controller;

static numeric_vector<3> target_angles{};

// Commands: t <deg>  t0 <deg>  t1 <deg>  t2 <deg>  stop
static void parse_Serial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();

  if (s == "stop") {
    for (int i = 0; i < 3; i++) {
      target_angles[i] = controller.last_angles[i];
      controller.driver.hlscl.WriteSpe(ServoDriver::IDS[i], 0,
        controller.driver.servo_accels, controller.driver.torque);
      controller.last_speeds[i] = 0;
    }
    Serial.println("[STOP]");
    return;
  }

  int sep = s.indexOf(' ');
  if (sep < 0) return;

  String key = s.substring(0, sep);
  double val = s.substring(sep + 1).toDouble();

  auto go = [&](int idx, double d) {
    target_angles[idx] = d;
    controller.reset_dir_accum(idx);
    Serial.printf("[TGT] servo %d -> %.2f deg\n", ServoDriver::IDS[idx], d);
  };

  if      (key == "t")  { for (int i = 0; i < 3; i++) go(i, val); }
  else if (key == "t0") go(0, val);
  else if (key == "t1") go(1, val);
  else if (key == "t2") go(2, val);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("[TEST_SERVO] boot - PID wheel mode");

  ServoSerial.begin(1'000'000);
  ServoSerial.setTimeout(20);
  controller.driver.hlscl.pSerial = &ServoSerial;
  controller.driver.hlscl.syncReadBegin(
    sizeof(ServoDriver::IDS), sizeof(controller.driver.rxPacket), 10);

  for (int i = 0; i < 3; i++) {
    if (controller.driver.hlscl.Ping(ServoDriver::IDS[i]) == ServoDriver::IDS[i])
      Serial.printf("[PING] servo %d OK\n", ServoDriver::IDS[i]);
    else
      Serial.printf("[PING] servo %d FAIL\n", ServoDriver::IDS[i]);

    controller.driver.hlscl.WheelMode(ServoDriver::IDS[i]);
    controller.driver.hlscl.EnableTorque(ServoDriver::IDS[i], 1);
  }

  controller.init_pid();

  for (int i = 0; i < 3; i++) {
    target_angles[i]               = controller.driver.read_angle(i);
    controller.prev_angles[i]      = target_angles[i];
    controller.last_angles[i]      = target_angles[i];
  }

  Serial.println("Cmds: t <deg>  t0/t1/t2 <deg>  stop");
  Serial.println("T(s),TGT0,CUR0,SPD0,TGT1,CUR1,SPD1,TGT2,CUR2,SPD2");
}

void loop() {
  static xcore::NbDelay log_delay(100, millis);

  parse_Serial();

  controller.servo_pid_update(target_angles);

  log_delay([&]() {
    double t = millis() * 0.001;
    Serial.print(t, 2);
    for (int i = 0; i < 3; i++) {
      Serial.print(',');
      Serial.print(target_angles[i], 2);
      Serial.print(',');
      Serial.print(controller.last_angles[i], 2);
      Serial.print(',');
      Serial.print(controller.last_speeds[i]);
    }
    Serial.println();
  });
}