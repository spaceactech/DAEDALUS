#ifndef ROCKET_AVIONICS_TEMPLATE_MAIN_H
#define ROCKET_AVIONICS_TEMPLATE_MAIN_H

#include <Arduino.h>  // Arduino Framework
#include <math.h>
#include "Controlling.h"

GPSCoordinate target_location = {13.723222811234036, 100.51557449640619};

/* Telemetry control */
bool telemetry_enabled = true;  // need to false

/* Simulation mode */
bool     simEnabled   = false;
bool     simActivated = false;
uint32_t simPressure = 101325u;
String   rx_message = "";

/* ACK / NACK counters */
uint32_t last_ack;
uint32_t last_nack;

char c;

bool gps_fixed = false;

struct DataMemory {
  SensorIMU::Data       imu[RA_NUM_IMU];
  SensorAltimeter::Data altimeter[RA_NUM_ALTIMETER];

  char       mode[4] = "F";
  
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
  uint8_t yaw_accuracy = 0;  // 0=unreliable … 3=high (BNO08x mag calibration)

  uint8_t deploy;
  char cmd_echo[16] = "CXON";

  // Freshness flags — set true by each Read*() when new data arrives,
  // cleared by ConstructString() after consuming, mirroring test_i2c pattern.
  int32_t cpu_temp = 0;  // snapshot taken by CB_ConstructData before ConstructString

  bool imu_fresh = false;
  bool alt_fresh = false;
  bool bno_fresh = false;
  bool gps_fresh = false;
  bool ina_fresh = false;
  bool tof_fresh = false;
};

struct valid {
  bool ina;
  bool bno;
  bool m10s;
  bool tof;
  bool sd = false;
} pvalid;

extern void ReadINA();

extern void ReadMAG();

extern void ReadTOF();

extern void AutoZeroAlt();

extern void HandleCommand(const String &rx);

extern void ConstructString();

#endif  //ROCKET_AVIONICS_TEMPLATE_MAIN_H
