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

GPSCoordinate current = {38.3756417, -79.6073944};
GPSCoordinate target  = {38.3760167, -79.6078722};

void setup() {
  Serial.begin(115200);
  delay(2000);
}


void loop() {
  // 5 variable needed
  // double altitude = getAltitude();

  // double vN = getVelNorth();
  // double vE = getVelEast();

  // current.lat     = getLatitude();
  // current.lon     = getLongitude();

  static double altitude = 5000.0;
  altitude               = altitude - 5;

  double vN = 7.5;
  double vE = 10.0;

  static double prev_dv     = 0;
  static double prev_dtheta = 0;
  static double at          = 0.2;  //Change
  static double av          = 0.2;  //Change

  double distance =
    calculate_distance(current, target);

  // Serial.println(distance);

  double bearing =
    calculate_bearing(current, target);

  auto nav =
    compute_velocity_navigation(vN, vE);

  double velocity = nav[0];
  double heading  = nav[1];

  double time_air =
    compute_time_in_air(altitude);

  double target_velocity =
    compute_target_velocity(
      distance,
      time_air);
  // Serial.println(target_velocity);

  auto delta =
    compute_delta_from_target(
      velocity,
      heading,
      target_velocity,
      bearing);

  double dv     = delta[0];
  double dtheta = delta[1];

  // Serial.println(dv);
  // Serial.println(dtheta);

  double dv_filtered =
    ema_filter(dv, prev_dv, av);

  double dtheta_filtered =
    ema_filter(dtheta, prev_dtheta, at);

  prev_dv     = dv_filtered;
  prev_dtheta = dtheta_filtered;

  // Serial.println(dv_filtered);
  // Serial.println(dtheta_filtered);

  auto target_vec =
    compute_target_vector(distance, bearing);
  // Serial.println(target_vec[0]);
  // Serial.println(target_vec[1]);

  auto control =
    compute_control_vector(target_vec, bearing);
  // Serial.println(bearing);
  // Serial.println(control[0]);
  // Serial.println(control[1]);
  // Serial.println(control[2]);

  auto servo = control_to_servo_angle(control);
  Serial.println(servo[0]); //servo 1
  Serial.println(servo[1]);
  Serial.println(servo[2]);
  delay(1000);
}
