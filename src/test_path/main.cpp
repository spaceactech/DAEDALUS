// --- Example ---
#include <Arduino.h>
#include "Controlling.h"
#include <lib_xcore>
#include <xcore/dispatcher>
#include <xcore/math_module>
#include "UserPins.h"

HardwareSerial ServoSerial(USER_GPIO_Half);

// Timer start
unsigned long startTime;

double       heading_deg = 0.0;
static float dt          = 0.02;

GPSCoordinate current = {38.3756417, -79.6073944};
GPSCoordinate target  = {38.3760167, -79.6078722};

Controller controller;

void setup() {
  Serial.begin(115200);
  delay(4000);

  startTime = millis();  // start timer

  ServoSerial.begin(1'000'000);
  controller.driver.sms_sts.pSerial = &ServoSerial;
  delay(1000);

  // Initialize servo driver
  controller.driver.sms_sts.syncReadBegin(sizeof(ServoDriver::IDS), sizeof(controller.driver.rxPacket), 5);
  for (size_t i = 0; i < sizeof(ServoDriver::IDS); ++i) {
    controller.driver.sms_sts.WheelMode(ServoDriver::IDS[i]);
  }

  Serial.println("Servo initialized");
}

void loop() {
  // -------------------------------
  // Example sensor inputs
  // -------------------------------
  static double altitude = 5000.0;
  altitude               = altitude - 5;

  double vN = 7.5;
  double vE = 10.0;

  static double prev_dv     = 0;
  static double prev_dtheta = 0;

  static double at = 0.2;
  static double av = 0.2;

  // ---------------------------------------------------
  // PID SERVO CONTROL
  // ---------------------------------------------------
  static uint16_t       interval = 50;
  static xcore::NbDelay delay(interval, millis);
  static xcore::NbDelay delay1(100, millis);

  double            angle1;
  numeric_vector<3> servo_target_angles;

  delay([&]() {
    servo_target_angles =
      controller.guidance.update(
        current,
        target,
        altitude,
        vN,
        vE,
        0);

    // Read servo angle
    angle1 = controller.driver.read_angle(0);

    // PID speed control
    const int16_t speed1 = Controller::compute_speed(controller.pid_controllers[0], servo_target_angles[0], angle1);

    Serial.print("Control: ");
    Serial.println(speed1);
    controller.driver.write_speed(0, speed1);
  });

  // ---------------------------------------------------
  // Serial Debug Output (Time Target Current)
  // ---------------------------------------------------
  delay1([&]() {
    double time_sec = (millis() - startTime) / 1000.0;

    Serial.print(time_sec);
    Serial.print(" ");

    Serial.print(servo_target_angles[0]);
    Serial.print(" ");

    Serial.println(angle1);
  });
}
