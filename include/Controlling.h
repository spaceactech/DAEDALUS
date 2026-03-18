#ifndef CONTROLLING_H
#define CONTROLLING_H

#include <Arduino.h>
#include <cmath>
#include <algorithm>
#include <lib_xcore>
#include <xcore/math_module>
#include <xcore/dispatcher>
#include <SCServo.h>

// ---------------- VECTOR ALIAS ----------------

template<size_t Size>
using numeric_vector = xcore::numeric_vector<Size>;


// ---------------- CONSTANTS ----------------

constexpr double EARTH_RADIUS_M     = 6371000.0;
constexpr double DISTANCE_TO_TARGET = 59.16;

constexpr double SPOOL_RADIUS = 0.0109;
constexpr double dL_max       = 0.2;  // rope pull limit **Change

constexpr double ENC_TO_DEG = 360.0 / 4096.0;  // **Change


// ---------------- STRUCT ----------------

struct GPSCoordinate {
  double lat;
  double lon;
};

struct EncoderTracker {
  int  prev_pos       = 0;
  long rotation_count = 0;

  double update(int pos) {
    int diff = pos - prev_pos;

    if (diff > 2048)
      rotation_count--;

    if (diff < -2048)
      rotation_count++;

    prev_pos = pos;

    long total_counts = rotation_count * 4096L + pos;

    return total_counts * ENC_TO_DEG;
  }
};

// ---------------- SERVO IDS ----------------

constexpr uint8_t servo_ids[3] = {1, 2, 3};

// ---------------- SERVO DRIVER ----------------
// PID gains (tune later)
constexpr double KP = 24.0;  //12
constexpr double KI = 0.0;
constexpr double KD = 0.15;

SMS_STS sms_sts;

uint8_t rxPacket[4];
byte    servo_accels = 255;

// ---------------- PID CONTROLLERS ----------------

struct Controller {
  xcore::pid_controller_t<decltype(millis())> pid_controllers[3] = {
    xcore::pid_controller_t<decltype(millis())>(KP, KI, KD),  //
    xcore::pid_controller_t<decltype(millis())>(KP, KI, KD),  //
    xcore::pid_controller_t<decltype(millis())>(KP, KI, KD),  //
  };

  EncoderTracker encoder_trackers[3]{};

  void init_pid() {
    for (auto &pid: pid_controllers) {
      pid.update_limits(-32767, 32767);
      pid.update_dt(0.05);
    }
  }


};


// ======================================================
// COMPUTE TIME IN AIR
// altitude → vz (dh/dt) → time_air
// ======================================================

double compute_time_in_air(double altitude) {
  static double         prev_altitude = 0;
  static double         vz            = 0;
  static double         time_air      = 0;
  static uint16_t       interval      = 50;
  static xcore::NbDelay delay(interval, millis);  // 50 ms sampling

  delay([&]() {
    double dh = altitude - prev_altitude;

    double dt = interval / 1000.0;

    if (dt > 0)
      vz = dh / dt;

    prev_altitude = altitude;

    if (std::abs(vz) > 0.01)
      time_air = altitude / std::abs(vz);
  });

  return time_air;
}


// ======================================================
// RESULTANT VELOCITY + HEADING FROM GPS
// ======================================================

numeric_vector<2> compute_velocity_navigation(double vN, double vE) {
  numeric_vector<2> result{};

  result[0] = std::sqrt(vN * vN + vE * vE);

  double heading =
    std::atan2(vE, vN) * RAD_TO_DEG;

  if (heading < 0)
    heading += 360;

  result[1] = heading;

  return result;
}


// ======================================================
// HAVERSINE DISTANCE
// ======================================================

double calculate_distance(GPSCoordinate p1, GPSCoordinate p2) {
  double dLat = (p2.lat - p1.lat) * DEG_TO_RAD;
  double dLon = (p2.lon - p1.lon) * DEG_TO_RAD;

  double a =
    std::sin(dLat / 2) * std::sin(dLat / 2) +
    std::cos(p1.lat * DEG_TO_RAD) *
      std::cos(p2.lat * DEG_TO_RAD) *
      std::sin(dLon / 2) * std::sin(dLon / 2);

  double c =
    2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

  double distance =
    EARTH_RADIUS_M * c;

  double ratio_distance = distance / DISTANCE_TO_TARGET;  // (0, 1)
  if (ratio_distance >= 1)
    ratio_distance = 1;

  return ratio_distance;
}

// ======================================================
// BEARING
// ======================================================

double calculate_bearing(GPSCoordinate p1, GPSCoordinate p2, double theta) {
  double lat1 = p1.lat * DEG_TO_RAD;
  double lat2 = p2.lat * DEG_TO_RAD;

  double dLon = (p2.lon - p1.lon) * DEG_TO_RAD;

  double y = std::sin(dLon) * std::cos(lat2);

  double x =
    std::cos(lat1) * std::sin(lat2) -
    std::sin(lat1) * std::cos(lat2) * std::cos(dLon);

  double bearing = (std::atan2(y, x) * RAD_TO_DEG) - theta;

  return std::fmod(bearing + 360.0, 360.0);
}


// ======================================================
// TARGET VELOCITY
// distance / time_air
// ======================================================

double compute_target_velocity(
  double distance,
  double time_air) {

  if (time_air <= 0.001)
    return 0;

  return distance / time_air;
}


// ======================================================
// DELTA FROM TARGET
// ======================================================

numeric_vector<2> compute_delta_from_target(
  double velocity,
  double heading,
  double target_velocity,
  double target_heading) {
  numeric_vector<2> delta{};

  delta[0] = target_velocity - velocity;
  delta[1] = target_heading - heading;

  // if (delta[1] > 180)
  //   delta[1] -= 360;

  // if (delta[1] < -180)
  //   delta[1] += 360;

  return delta;
}


// ======================================================
// EMA FILTER
// ======================================================

double ema_filter(double input, double prev, double alpha) {
  return alpha * input + (1 - alpha) * prev;
}


// ======================================================
// TARGET VECTOR
// ======================================================

numeric_vector<2> compute_target_vector(
  double distance,
  double bearing) {
  numeric_vector<2> v{};

  double rad = bearing * DEG_TO_RAD;

  v[0] = distance * std::cos(rad);
  v[1] = distance * std::sin(rad);

  return v;
}


// ======================================================
// CONTROL ALLOCATION
// ======================================================

numeric_vector<3> compute_control_vector(
  const numeric_vector<2> &target,
  double                   theta) {
  constexpr double INV_SQRT3     = 0.57735026919;
  constexpr double TWO_INV_SQRT3 = 1.15470053838;

  double X = target[0];
  double Y = target[1];

  double u1 = 0, u2 = 0, u3 = 0;

  if (theta > 180.0)
    theta = theta - 360.0;

  if (theta >= 0 && theta < 120)
    u1 = X + INV_SQRT3 * Y;
  else if (theta >= -120 && theta < 0)
    u1 = X - INV_SQRT3 * Y;

  if (theta >= 0 && theta < 120)
    u2 = TWO_INV_SQRT3 * Y;
  else if ((theta >= 120 && theta <= 180) ||
           (theta > -180 && theta < -120))
    u2 = -X + INV_SQRT3 * Y;

  if (theta >= -120 && theta < 0)
    u3 = -TWO_INV_SQRT3 * Y;
  else if ((theta >= 120 && theta <= 180) ||
           (theta > -180 && theta < -120))
    u3 = -X - INV_SQRT3 * Y;

  numeric_vector<3> out{};

  out[0] = u1;
  out[1] = u2;
  out[2] = u3;

  return out;
}


// ======================================================
// CONTROL → SERVO ANGLE
// ======================================================

numeric_vector<3> control_to_servo_angle(
  numeric_vector<3> u) {
  numeric_vector<3> angle{};

  for (int i = 0; i < 3; i++) {
    double scaled = u[i] * 0.75;

    double dL = scaled * dL_max;

    double rotation =
      dL / (2.0 * M_PI * SPOOL_RADIUS);

    angle[i] = rotation * 360.0;
  }

  return angle;
}

// =====================================================
// READ MULTI TURN ANGLE
// =====================================================

double read_angle(uint8_t id, EncoderTracker &tracker) {
  sms_sts.syncReadPacketTx(servo_ids, sizeof(servo_ids), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket));
  sms_sts.syncReadPacketRx(servo_ids[id], rxPacket);
  int pos = sms_sts.syncReadRxPacketToWrod(15);

  // Serial.println(pos);

  if (pos < 0)
    return 0;


  return tracker.update(pos);
}

// =====================================================
// PID speed computation
// =====================================================

inline int16_t compute_speed(xcore::pid_controller_t<uint32_t> &pid,
                             const double                      &target_angle,
                             const double                      &current_angle) {
  double speed = pid.update(target_angle, current_angle, 0.02);
  speed        = constrain(speed, -32767.0, 32767.0);

  return static_cast<int16_t>(speed);
}

// =====================================================
// Apply speed to servo (wheel mode)
// =====================================================

inline void write_speed(const uint8_t id, const int16_t speed) {
  sms_sts.WriteSpe(id, speed, servo_accels);
}

// ======================================================
// MAIN GUIDANCE FUNCTION
// ======================================================

numeric_vector<3> guidance_update(
  GPSCoordinate current,
  GPSCoordinate target,
  double        altitude,
  double        vN,
  double        vE,
  double        theta_offset) {
  static double prev_dv     = 0;
  static double prev_dtheta = 0;
  static double at          = 0.2;  //Change
  static double av          = 0.2;  //Change

  double distance =
    calculate_distance(current, target);

  double bearing =
    calculate_bearing(current, target, theta_offset);

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

  auto delta =
    compute_delta_from_target(
      velocity,
      heading,
      target_velocity,
      bearing);

  double dv     = delta[0];
  double dtheta = delta[1];

  double dv_filtered =
    ema_filter(dv, prev_dv, av);

  double dtheta_filtered =
    ema_filter(dtheta, prev_dtheta, at);

  prev_dv     = dv_filtered;
  prev_dtheta = dtheta_filtered;

  auto target_vec =
    compute_target_vector(distance, bearing);

  auto control =
    compute_control_vector(target_vec, bearing);

  return control_to_servo_angle(control);
}

// =====================================================
// MAIN SERVO PID UPDATE
// input = numeric_vector<3> target angles
// =====================================================

void servo_pid_update(const numeric_vector<3> &target_angles) {
  // ---- Read current angles ----

  double angle1 = read_angle(SERVO1_ID, enc1);
  double angle2 = read_angle(SERVO2_ID, enc2);
  double angle3 = read_angle(SERVO3_ID, enc3);

  // ---- Compute PID speeds ----
  int speed1 = compute_speed(pid1, target_angles[0], angle1);
  int speed2 = compute_speed(pid2, target_angles[1], angle2);
  int speed3 = compute_speed(pid3, target_angles[2], angle3);

  // ---- Small deadband ----

  if (abs(target_angles[0] - angle1) < 0.5)
    speed1 = 0;

  if (abs(target_angles[1] - angle2) < 0.5)
    speed2 = 0;

  if (abs(target_angles[2] - angle3) < 0.5)
    speed3 = 0;

  // ---- Send to servo ----

  write_speed(SERVO1_ID, speed1);
  write_speed(SERVO2_ID, speed2);
  write_speed(SERVO3_ID, speed3);
}

#endif