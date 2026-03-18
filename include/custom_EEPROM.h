#ifndef CUSTOM_EEPROM_H
#define CUSTOM_EEPROM_H

#include <Arduino.h>
#include <EEPROM.h>
#include "UserFSM.h"  // User's FSM States

#define EEPROM_SIZE 64

/* ------------ EEPROM ADDRESSES ------------ */

#define ADDR_PACKET_COUNT 0
#define ADDR_ALT_REF      4
#define ADDR_ACC          12
#define ADDR_UTC          20
#define ADDR_FSM_STATE    29
#define ADDR_SERVO1       30
#define ADDR_SERVO2       38
#define ADDR_SERVO3       46

/* ------------ VARIABLES ------------ */

uint32_t packet_count = 0;
double   alt_ref      = 0;
double   acc          = 0;
char     utc[9]       = "00:00:00";

double angle1 = 0.0;
double angle2 = 0.0;
double angle3 = 0.0;

UserFSM fsm;

uint8_t fsm_int = static_cast<uint8_t>(fsm.state());

/* ------------ SAVE ALL ------------ */

void savePersistent() {
  EEPROM.get(ADDR_PACKET_COUNT, packet_count);
  EEPROM.get(ADDR_ALT_REF, alt_ref);
  EEPROM.get(ADDR_ACC, acc);
  EEPROM.get(ADDR_UTC, utc);

  EEPROM.get(ADDR_FSM_STATE, fsm_int);

  EEPROM.get(ADDR_SERVO1, angle1);
  EEPROM.get(ADDR_SERVO2, angle2);
  EEPROM.get(ADDR_SERVO3, angle3);
}
/* ------------ LOAD ALL ------------ */

void loadPersistent() {
  EEPROM.get(ADDR_PACKET_COUNT, packet_count);
  EEPROM.get(ADDR_ALT_REF, alt_ref);
  EEPROM.get(ADDR_ACC, acc);
  EEPROM.get(ADDR_UTC, utc);

  EEPROM.get(ADDR_FSM_STATE, fsm_int);

  EEPROM.get(ADDR_SERVO1, angle1);
  EEPROM.get(ADDR_SERVO2, angle2);
  EEPROM.get(ADDR_SERVO3, angle3);
}

#endif
