// --- Example ---
#include <Arduino.h>
#include "Controlling.h"
#include <lib_xcore>
#include <xcore/math_module>

// GPSCoordinate target_waypoint = {13.736746, 100.736746};
// GPSCoordinate current         = {13.736780, 100.736780};

double       heading_deg = 0.0;  // input heading (deg)
static float dt          = 0.02;
float        u1, u2, u3;

GPSCoordinate current = {13.736717, 100.523186};
GPSCoordinate target  = {13.7360108, 100.5218896};

void setup() {
  Serial.begin(115200);
  delay(2000);
}


void loop() {
  auto control = path_calculation(current, target);

  Serial.print("U1_ref: ");
  Serial.println(control[0]);

  Serial.print("U2_ref: ");
  Serial.println(control[1]);

  Serial.print("U3_ref: ");
  Serial.println(control[2]);

  PID_Control(control[0], 1, u1);
  PID_Control(control[1], 1, u2);
  PID_Control(control[2], 1, u3);

  Serial.print("U1: ");
  Serial.println(u1);

  Serial.print("U2: ");
  Serial.println(u2);

  Serial.print("U3: ");
  Serial.println(u3);

  delay(1000);
}
