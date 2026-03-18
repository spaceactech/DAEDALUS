// --- Example ---
#include <Arduino.h>
#include "Controlling.h"
#include <lib_xcore>
#include <xcore/dispatcher>
#include <xcore/math_module>
#include "UserPins.h"

HardwareSerial ServoSerial(USER_GPIO_XBEE_RX, USER_GPIO_XBEE_TX);

const int SERVO_DIR_PIN = 0;

// Timer start
unsigned long startTime;

double       heading_deg = 0.0;
static float dt          = 0.02;

GPSCoordinate current = {38.3756417, -79.6073944};
GPSCoordinate target  = {38.3760167, -79.6078722};

void setup() {

  Serial.begin(115200);
  delay(4000);

  startTime = millis();  // start timer

  ServoSerial.begin(1000000);

  // Initialize servo driver
  if (!servo.init(SERVO_DIR_PIN, &ServoSerial)) {
    Serial.println("Servo init failed!");
    while (1);
  }

  Serial.println("Servo initialized");

  servo.writeRegister(SERVO1_ID, STSRegisters::OPERATION_MODE, 1);
  delay(1000);
}

void loop() {
  return;
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
      guidance_update(
        current,
        target,
        altitude,
        vN,
        vE,
        0);

    // Read servo angle
    angle1 = read_angle(SERVO1_ID, enc1);

    // PID speed control
    int speed1 = compute_speed(pid1, servo_target_angles[0], angle1);

    write_speed(SERVO1_ID, speed1);
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