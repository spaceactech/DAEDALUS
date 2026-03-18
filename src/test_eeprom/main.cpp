#include "custom_EEPROM.h"
#include <Arduino.h>

void setup() {
  Serial.begin(115200);

  // EEPROM.begin(EEPROM_SIZE);

  loadPersistent();

  Serial.println("Recovered data");

  Serial.print("Packet count: ");
  Serial.println(packet_count);

  Serial.print("UTC: ");
  Serial.println(utc);

  Serial.print("FSM state: ");
  Serial.println(fsm_int);

  Serial.print("Servo positions: ");
  Serial.print(angle1);
  Serial.print(" ");
  Serial.print(angle2);
  Serial.print(" ");
  Serial.println(angle3);
}

/* ------------ LOOP ------------ */

void loop() {
  packet_count++;

  angle1 += 0.1;
  angle2 += 0.1;
  angle3 += 0.1;

  static uint32_t last_save = 0;

  if (millis() - last_save > 5000) {
    savePersistent();
    Serial.println("Saved persistent data");
    last_save = millis();
    delay(100);
    loadPersistent();
    Serial.println("Recovered data");

    Serial.print("Packet count: ");
    Serial.println(packet_count);

    Serial.print("UTC: ");
    Serial.println(utc);

    Serial.print("FSM state: ");
    Serial.println(fsm_int);

    Serial.print("Servo positions: ");
    Serial.print(angle1);
    Serial.print(" ");
    Serial.print(angle2);
    Serial.print(" ");
    Serial.println(angle3);
  }

  delay(1000);
}