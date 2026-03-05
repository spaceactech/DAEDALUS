// --- Example ---
#include <Arduino.h>
#include "Controlling.h"
#include <lib_xcore>
#include <xcore/math_module>

GPSCoordinate target_waypoint = {13.736746, 100.736746};
GPSCoordinate current         = {13.736780, 100.736780};

double       heading_deg = 0.0;  // input heading (deg)
static float dt          = 0.02;
float        u1, u2, u3;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
}

void loop() {
  double dist    = calculate_distance(current, target_waypoint);
  double bearing = calculate_bearing(current, target_waypoint);

  // Controlling
  // Body-frame transform
  double theta = (heading_deg - bearing) * DEG_TO_RAD_VAL;

  numeric_vector<2> body_vec{};
  body_vec[0] = dist * cos(theta);  // X (forward)
  body_vec[1] = dist * sin(theta);  // Y (left)

  numeric_vector<3> u_ref = compute_control_vector(body_vec, theta);

  PID_Control(u_ref[0], dt, u1);
  PID_Control(u_ref[1], dt, u2);
  PID_Control(u_ref[2], dt, u3);

  delay(1000);
}
