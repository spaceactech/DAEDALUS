#ifndef ROCKET_AVIONICS_TEMPLATE_MAIN_H
#define ROCKET_AVIONICS_TEMPLATE_MAIN_H

#include <Arduino.h>  // Arduino Framework
#include <math.h>
#include "Controlling.h"

GPSCoordinate target  = {13.723186, 100.515473};

/* Telemetry control */
bool telemetry_enabled = false; // need to false

/* Simulation mode */
bool     simEnabled   = false;
bool     simActivated = false;
uint32_t simPressure;
String   rx_message = "";

/* ACK / NACK counters */
uint32_t last_ack;
uint32_t last_nack;

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
