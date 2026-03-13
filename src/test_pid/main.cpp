#include <Arduino.h>
#include <STSServoDriver.h>
#include "UserPins.h"

HardwareSerial ServoSerial(USER_GPIO_XBEE_RX, USER_GPIO_XBEE_TX);

STSServoDriver servo;

const byte SERVO_ID      = 1;
const int  SERVO_DIR_PIN = 0;

void setup() {
  Serial.begin(115200);
  delay(4000);
  // Start servo serial
  ServoSerial.begin(1000000);

  // Initialize servo driver
  if (!servo.init(SERVO_DIR_PIN, &ServoSerial)) {
    Serial.println("Servo init failed!");
    while (1);
  }

  Serial.println("Servo initialized");

  // Set Servo Mode (Position mode)
  servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 1);
  delay(1000);
}

void loop() {

  // -------- WRITE POSITION --------
  Serial.println("Move to 0 deg");
  servo.setTargetVelocity(SERVO_ID, -3500);
  delay(2000);

  Serial.println("Move to 180 deg");
  servo.setTargetVelocity(SERVO_ID, 3500);
  delay(2000);

  // -------- READ DATA --------
  int pos        = servo.getCurrentPosition(SERVO_ID);
  int speed      = servo.getCurrentSpeed(SERVO_ID);
  int temp       = servo.getCurrentTemperature(SERVO_ID);
  int current    = servo.getCurrentCurrent(SERVO_ID);
  int voltageRaw = servo.readRegister(SERVO_ID, STSRegisters::CURRENT_VOLTAGE);

  float voltage = voltageRaw / 10.0;

  Serial.println("----- Servo Status -----");
  Serial.print("Position: ");
  Serial.println(pos);

  Serial.print("Speed: ");
  Serial.println(speed);

  Serial.print("Temperature: ");
  Serial.println(temp);

  Serial.print("Current: ");
  Serial.println(current);

  Serial.print("Voltage: ");
  Serial.println(voltage);

  Serial.println("------------------------");

  delay(1000);
}