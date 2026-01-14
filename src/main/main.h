#ifndef ROCKET_AVIONICS_TEMPLATE_MAIN_H
#define ROCKET_AVIONICS_TEMPLATE_MAIN_H

#include <Arduino.h>  // Arduino Framework

/* Telemetry control */
bool telemetry_enabled;

/* Simulation mode */
bool     simEnabled;
bool     simActivated;
uint32_t simPressure;
String   rx_message;

/* ACK / NACK counters */
uint32_t last_ack;
uint32_t last_nack;

extern void ReadINA();

extern void ReadMAG();

extern void ReadTOF();

extern void AutoZeroAlt();

extern void HandleCommand(const String &rx);

#endif  //ROCKET_AVIONICS_TEMPLATE_MAIN_H
