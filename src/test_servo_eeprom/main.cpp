/*
 * HLS Servo EEPROM Programmer
 *
 * Unlocks EEPROM, writes parameters, locks EEPROM, then verifies.
 * Commands (via USB Serial at 460800):
 *   dump <id>              — print all EEPROM registers for servo <id>
 *   set_id <old> <new>     — change servo ID
 *   set_baud <id> <code>   — set baud rate code (0=1M, 1=500k, 2=250k, 3=128k, 4=115.2k, 7=38.4k)
 *   set_angle <id> <min> <max>  — angle limits in raw ticks (0–4095)
 *   set_ofs <id> <ofs>     — position offset in raw ticks (signed)
 *   set_mode <id> <mode>   — 0=servo, 1=wheel, 2=constant-torque
 *   set_cw_dead <id> <val> — CW dead band
 *   set_ccw_dead <id> <val>— CCW dead band
 *   write <id> <addr> <val>— raw byte write (auto unlock/lock)
 *   ping <id>              — ping servo
 */

#include <Arduino.h>
#include "UserPins.h"
#include <SCServo.h>
#include "Controlling.h"

// ─── Config ──────────────────────────────────────────────────────────────────
static constexpr uint32_t SERVO_BAUD = 1'000'000;
static constexpr uint32_t USB_BAUD   = 460800;

// ─── Globals ─────────────────────────────────────────────────────────────────
HardwareSerial ServoSerial(USER_GPIO_Half);
HLSCL          hlscl;
Controller     controller;
// ─── Helpers ─────────────────────────────────────────────────────────────────

static bool eeprom_write_byte(uint8_t id, uint8_t addr, uint8_t val) {
  hlscl.unLockEprom(id);
  delay(10);
  int ack = hlscl.writeByte(id, addr, val);
  delay(10);
  hlscl.LockEprom(id);
  delay(10);
  return ack >= 0;
}

static bool eeprom_write_word(uint8_t id, uint8_t addr, uint16_t val) {
  hlscl.unLockEprom(id);
  delay(10);
  int ack = hlscl.writeWord(id, addr, val);
  delay(10);
  hlscl.LockEprom(id);
  delay(10);
  return ack >= 0;
}

static void dump_eeprom(uint8_t id) {
  Serial.printf("\n── EEPROM dump: servo %d ──\n", id);

  int model_l  = hlscl.readByte(id, HLSCL_MODEL_L);
  int model_h  = hlscl.readByte(id, HLSCL_MODEL_H);
  int s_id     = hlscl.readByte(id, HLSCL_ID);
  int baud     = hlscl.readByte(id, HLSCL_BAUD_RATE);
  int sec_id   = hlscl.readByte(id, HLSCL_SECOND_ID);
  int min_ang  = hlscl.readWord(id, HLSCL_MIN_ANGLE_LIMIT_L);
  int max_ang  = hlscl.readWord(id, HLSCL_MAX_ANGLE_LIMIT_L);
  int cw_dead  = hlscl.readByte(id, HLSCL_CW_DEAD);
  int ccw_dead = hlscl.readByte(id, HLSCL_CCW_DEAD);
  int ofs      = hlscl.readWord(id, HLSCL_OFS_L);
  int mode     = hlscl.readByte(id, HLSCL_MODE);

  Serial.printf("  MODEL        : 0x%02X%02X\n", model_h, model_l);
  Serial.printf("  ID           : %d\n", s_id);
  Serial.printf("  BAUD_RATE    : %d  (0=1M 1=500k 2=250k 3=128k 4=115.2k 7=38.4k)\n", baud);
  Serial.printf("  SECOND_ID    : %d\n", sec_id);
  Serial.printf("  MIN_ANGLE    : %d  (%.2f deg)\n", min_ang, min_ang / 4096.0 * 360.0);
  Serial.printf("  MAX_ANGLE    : %d  (%.2f deg)\n", max_ang, max_ang / 4096.0 * 360.0);
  Serial.printf("  CW_DEAD      : %d\n", cw_dead);
  Serial.printf("  CCW_DEAD     : %d\n", ccw_dead);
  Serial.printf("  OFS          : %d\n", (int16_t) ofs);
  Serial.printf("  MODE         : %d  (0=servo 1=wheel 2=torque)\n", mode);
  Serial.println("────────────────────────────────");
}

// ─── Command parser ───────────────────────────────────────────────────────────

static void handle_command(String &line) {
  line.trim();
  if (line.length() == 0) return;

  // Tokenise
  String tok[5];
  int    n   = 0;
  int    pos = 0;
  while (n < 5 && pos <= (int) line.length()) {
    int sp = line.indexOf(' ', pos);
    if (sp < 0) sp = line.length();
    tok[n++] = line.substring(pos, sp);
    pos      = sp + 1;
  }

  String &cmd = tok[0];

  if (cmd == "ping") {
    uint8_t id = (uint8_t) tok[1].toInt();
    int     r  = hlscl.Ping(id);
    Serial.printf("[PING] servo %d -> %s\n", id, r == id ? "OK" : "FAIL");

  } else if (cmd == "dump") {
    uint8_t id = (uint8_t) tok[1].toInt();
    dump_eeprom(id);

  } else if (cmd == "set_id") {
    uint8_t old_id = (uint8_t) tok[1].toInt();
    uint8_t new_id = (uint8_t) tok[2].toInt();
    bool    ok     = eeprom_write_byte(old_id, HLSCL_ID, new_id);
    Serial.printf("[SET_ID] %d -> %d  %s\n", old_id, new_id, ok ? "OK" : "FAIL");
    if (ok) Serial.println("  ** Re-power the servo to activate the new ID **");

  } else if (cmd == "set_baud") {
    uint8_t id   = (uint8_t) tok[1].toInt();
    uint8_t code = (uint8_t) tok[2].toInt();
    bool    ok   = eeprom_write_byte(id, HLSCL_BAUD_RATE, code);
    Serial.printf("[SET_BAUD] servo %d code=%d  %s\n", id, code, ok ? "OK" : "FAIL");

  } else if (cmd == "set_angle") {
    uint8_t  id = (uint8_t) tok[1].toInt();
    uint16_t mn = (uint16_t) tok[2].toInt();
    uint16_t mx = (uint16_t) tok[3].toInt();
    hlscl.unLockEprom(id);
    delay(10);
    int a1 = hlscl.writeWord(id, HLSCL_MIN_ANGLE_LIMIT_L, mn);
    delay(5);
    int a2 = hlscl.writeWord(id, HLSCL_MAX_ANGLE_LIMIT_L, mx);
    delay(10);
    hlscl.LockEprom(id);
    delay(10);
    Serial.printf("[SET_ANGLE] servo %d min=%d max=%d  %s\n",
                  id, mn, mx, (a1 >= 0 && a2 >= 0) ? "OK" : "FAIL");

  } else if (cmd == "set_ofs") {
    uint8_t id  = (uint8_t) tok[1].toInt();
    int16_t ofs = (int16_t) tok[2].toInt();
    bool    ok  = eeprom_write_word(id, HLSCL_OFS_L, (uint16_t) ofs);
    Serial.printf("[SET_OFS] servo %d ofs=%d  %s\n", id, ofs, ok ? "OK" : "FAIL");

  } else if (cmd == "set_mode") {
    uint8_t id   = (uint8_t) tok[1].toInt();
    uint8_t mode = (uint8_t) tok[2].toInt();
    bool    ok   = eeprom_write_byte(id, HLSCL_MODE, mode);
    Serial.printf("[SET_MODE] servo %d mode=%d  %s\n", id, mode, ok ? "OK" : "FAIL");

  } else if (cmd == "set_cw_dead") {
    uint8_t id  = (uint8_t) tok[1].toInt();
    uint8_t val = (uint8_t) tok[2].toInt();
    bool    ok  = eeprom_write_byte(id, HLSCL_CW_DEAD, val);
    Serial.printf("[SET_CW_DEAD] servo %d val=%d  %s\n", id, val, ok ? "OK" : "FAIL");

  } else if (cmd == "set_ccw_dead") {
    uint8_t id  = (uint8_t) tok[1].toInt();
    uint8_t val = (uint8_t) tok[2].toInt();
    bool    ok  = eeprom_write_byte(id, HLSCL_CCW_DEAD, val);
    Serial.printf("[SET_CCW_DEAD] servo %d val=%d  %s\n", id, val, ok ? "OK" : "FAIL");

  } else if (cmd == "write") {
    uint8_t id   = (uint8_t) tok[1].toInt();
    uint8_t addr = (uint8_t) tok[2].toInt();
    uint8_t val  = (uint8_t) tok[3].toInt();
    bool    ok   = eeprom_write_byte(id, addr, val);
    Serial.printf("[WRITE] servo %d addr=0x%02X val=%d  %s\n", id, addr, val, ok ? "OK" : "FAIL");

  } else {
    Serial.println("[ERR] unknown command");
  }
}

// ─── Entry points ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(USB_BAUD);
  delay(2000);
  Serial.println("\n[TEST_SERVO_EEPROM] boot");

  ServoSerial.begin(SERVO_BAUD);
  ServoSerial.setTimeout(20);
  hlscl.pSerial = &ServoSerial;

  // Quick scan: ping IDs 1–5
  Serial.println("Scanning IDs 1-5...");
//   for (uint8_t id = 1; id <= 5; id++) {
//     if (hlscl.Ping(id) == id)
//       Serial.printf("  [FOUND] servo ID %d\n", id);
//   }

  Serial.println("\nCommands:");
  Serial.println("  ping <id>");
  Serial.println("  dump <id>");
  Serial.println("  set_id <old> <new>");
  Serial.println("  set_baud <id> <code>     0=1M 1=500k 2=250k 3=128k 4=115.2k 7=38.4k");
  Serial.println("  set_angle <id> <min> <max>   raw ticks 0-4095");
  Serial.println("  set_ofs <id> <ofs>           signed ticks");
  Serial.println("  set_mode <id> <mode>         0=servo 1=wheel 2=torque");
  Serial.println("  set_cw_dead / set_ccw_dead <id> <val>");
  Serial.println("  write <id> <addr> <val>      raw byte write");
  Serial.println();
}

void loop() {
  //   if (Serial.available()) {
  //     String line = Serial.readStringUntil('\n');
  //     handle_command(line);
  //   }
  //   hlscl.WriteSpe(3, 1000, 255, 500);
  //   delay(1000);
  //   hlscl.WriteSpe(3, 0, 255, 500);
  //   delay(1000);
  //   controller.driver.write_speed(3, 0);
  //   delay(1900);
  //   controller.driver.write_speed(3, 1000);
  //   delay(1900);
  for (size_t i = 0; i < 3; ++i)
    controller.driver.write_speed(ServoDriver::IDS[i], 0);
  delay(3000);
  for (size_t i = 0; i < 3; ++i)
    controller.driver.write_speed(ServoDriver::IDS[i], 1000);
  delay(3000);
}
