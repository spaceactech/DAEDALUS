#ifndef ROCKET_AVIONICS_TEMPLATE_MAIN_H
#define ROCKET_AVIONICS_TEMPLATE_MAIN_H

#include <Arduino.h>  // Arduino Framework
#include <math.h>
#include "Controlling.h"

GPSCoordinate target_location  = {13.723186, 100.515473};

/* Telemetry control */
bool telemetry_enabled = true; // need to false

/* Simulation mode */
bool     simEnabled   = false;
bool     simActivated = false;
uint32_t simPressure;
String   rx_message = "";

/* ACK / NACK counters */
uint32_t last_ack;
uint32_t last_nack;

struct DataMemory {
  SensorIMU::Data       imu[RA_NUM_IMU];
  SensorAltimeter::Data altimeter[RA_NUM_ALTIMETER];

  String mode = "F";

  uint32_t timestamp_epoch;
  uint32_t timestamp_us{};

  char    utc[9] = "00:00:00";
  uint8_t siv;
  double  latitude;
  double  longitude;
  double  altitude_msl;
  double  velocity_n;
  double  velocity_e;
  uint8_t hh, mm, ss;

  float batt_volt;
  float batt_curr;

  float tof;

  float yaw;
  float pitch;
  float roll;
  float heading;

  bool   deploy;
  String cmd_echo = "ECHO";

};

struct valid {
  bool ina;
  bool bno;
  bool m10s;
  bool tof;
} pvalid;

extern void ReadINA();

extern void ReadMAG();

extern void ReadTOF();

extern void AutoZeroAlt();

extern void HandleCommand(const String &rx);

extern void ConstructString();

#endif  //ROCKET_AVIONICS_TEMPLATE_MAIN_H
