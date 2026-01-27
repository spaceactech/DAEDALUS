// --- Example ---
#include <Arduino.h>
#include "PathCal.h"

GPSCoordinate target_waypoint = {13.736717, 100.523186};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  GPSCoordinate current     = {13.736000, 100.523000};
  double        heading_deg = 0.0;  // input heading (deg)

  double dist    = calculate_distance(current, target_waypoint);
  double bearing = calculate_bearing(current, target_waypoint);

  // Body-frame transform
  double theta = (heading_deg - bearing) * DEG_TO_RAD_VAL;

  numeric_vector<2> body_vec{};
  body_vec[0] = dist * cos(theta);  // X (forward)
  body_vec[1] = dist * sin(theta);  // Y (left)

  numeric_vector<3> u = compute_control_vector(body_vec, theta);

  Serial.println("--- Control Output ---");
  Serial.print("U1: ");
  Serial.println(u[0]);
  Serial.print("U2: ");
  Serial.println(u[1]);
  Serial.print("U3: ");
  Serial.println(u[2]);
}

void loop() {
  delay(1000);
}
