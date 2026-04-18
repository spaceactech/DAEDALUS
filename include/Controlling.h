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
constexpr double dL_max       = 0.298758;
;  // rope pull limit **Change

constexpr double ENC_TO_DEG = 360.0 / 4096.0;  // **Change

// PID gains (tune later)
constexpr double KP = 2.5;
constexpr double KI = 0.0;
constexpr double KD = 0.15;

constexpr double at = 0.2;
constexpr double av = 0.2;

// ---------------- STRUCTS ----------------

struct GPSCoordinate {
  double lat;
  double lon;
};

struct EncoderTracker {
  int    prev_pos       = 0;
  long   rotation_count = 0;
  double last_angle     = 0;

  double update(int16_t raw_pos, double max_deg_per_step = 200.0) {
    int pos = raw_pos % 4096;
    if (pos < 0) pos += 4096;

    int  diff      = pos - prev_pos;
    bool crossover = false;

    // Rollover detection: only valid if servo moves <180 deg per read interval
    if (diff > 2048)  { rotation_count--; crossover = true; }
    if (diff < -2048) { rotation_count++; crossover = true; }

    double new_angle = (rotation_count * 4096L + pos) * ENC_TO_DEG;

    // Reject corrupt (non-crossover) reads that imply an impossible jump.
    // Always advance prev_pos so a failed read cannot re-trigger the same
    // crossover on the next call and lock the tracker in place.
    if (!crossover && std::abs(new_angle - last_angle) > max_deg_per_step) {
      prev_pos = pos;
      return last_angle;
    }

    prev_pos   = pos;
    last_angle = new_angle;
    return last_angle;
  }
};


// ======================================================
// SERVO DRIVER
// Low-level servo hardware I/O and encoder tracking
// ======================================================

struct ServoDriver {

  static constexpr uint8_t IDS[3] = {1, 2, 3};

  HLSCL          hlscl;
  uint8_t        rxPacket[4]  = {};
  byte           servo_accels = 255;
  uint16_t       torque       = 500;
  EncoderTracker encoder_trackers[3]{};

  // ---- Read multi-turn angle for servo at index ----

  double read_angle(size_t index) {
    hlscl.syncReadPacketTx(const_cast<uint8_t *>(&IDS[index]), 1, SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket));

    if (hlscl.syncReadPacketRx(IDS[index], rxPacket) <= 0)
      return encoder_trackers[index].last_angle;

    int pos = ((rxPacket[1] & 0x0F) << 8) | rxPacket[0];
    return encoder_trackers[index].update(static_cast<int16_t>(pos));
  }

  // ---- Apply speed to servo (wheel mode) ----

  void write_speed(uint8_t id, int16_t speed) {
    int ack = hlscl.WriteSpe(id, speed, servo_accels, torque);
    Serial.print("ACK(");
    Serial.print(id);
    Serial.print("): ");
    Serial.println(ack);
  }
};


// ======================================================
// GUIDANCE
// Navigation math and guidance computation
// ======================================================

struct Guidance {

  // ---- Compute time in air: altitude → vz → time_air ----

  static double compute_time_in_air(double altitude) {
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

  // ---- Resultant velocity + heading from GPS ----

  static numeric_vector<2> compute_velocity_navigation(double vN, double vE) {
    numeric_vector<2> result{};

    result[0] = std::sqrt(vN * vN + vE * vE);

    double heading = std::atan2(vE, vN) * RAD_TO_DEG;

    if (heading < 0)
      heading += 360;

    result[1] = heading;

    return result;
  }

  // ---- Haversine distance ----

  static double calculate_distance(const GPSCoordinate &p1, const GPSCoordinate &p2) {
    double dLat = (p2.lat - p1.lat) * DEG_TO_RAD;
    double dLon = (p2.lon - p1.lon) * DEG_TO_RAD;

    double a =
      std::sin(dLat / 2) * std::sin(dLat / 2) +
      std::cos(p1.lat * DEG_TO_RAD) *
        std::cos(p2.lat * DEG_TO_RAD) *
        std::sin(dLon / 2) * std::sin(dLon / 2);

    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    double ratio_distance = (EARTH_RADIUS_M * c) / DISTANCE_TO_TARGET;  // (0, 1)

    if (ratio_distance >= 1)
      ratio_distance = 1;

    return ratio_distance;
  }

  // ---- Bearing ----

  static double calculate_bearing(const GPSCoordinate &p1, const GPSCoordinate &p2, double theta) {
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

  // ---- Target velocity: distance / time_air ----

  static double compute_target_velocity(double distance, double time_air) {
    if (time_air <= 0.001)
      return 0;

    return distance / time_air;
  }

  // ---- Delta from target ----

  static numeric_vector<2> compute_delta_from_target(
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

  // ---- EMA filter ----

  static double ema_filter(double input, double prev, double alpha) {
    return alpha * input + (1 - alpha) * prev;
  }

  // ---- Target vector ----

  static numeric_vector<2> compute_target_vector(double distance, double bearing) {
    numeric_vector<2> v{};

    double rad = bearing * DEG_TO_RAD;

    v[0] = distance * std::cos(rad);
    v[1] = distance * std::sin(rad);

    return v;
  }

  // ---- Control allocation ----

  static numeric_vector<3> compute_control_vector(
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

  // ---- Control → servo angle ----

  static numeric_vector<3> control_to_servo_angle(const numeric_vector<3> &u) {
    numeric_vector<3> angle{};

    for (int i = 0; i < 3; i++) {
      double scaled   = std::max(0.0, u[i]) * 0.75;
      double dL       = scaled * dL_max;
      double rotation = dL / (2.0 * M_PI * SPOOL_RADIUS);

      angle[i] = rotation * 360.0;
    }

    return angle;
  }

  // ---- Main guidance update ----

  numeric_vector<3> update(
    const GPSCoordinate &current,
    const GPSCoordinate &target,
    double               altitude,
    double               vN,
    double               vE,
    double               theta_offset) {
    static double prev_dv     = 0;
    static double prev_dtheta = 0;
    static double at          = 0.2;  //Change
    static double av          = 0.2;  //Change

    double distance = calculate_distance(current, target);
    double bearing  = calculate_bearing(current, target, theta_offset);

    auto   nav      = compute_velocity_navigation(vN, vE);
    double velocity = nav[0];
    double heading  = nav[1];

    double time_air        = compute_time_in_air(altitude);
    double target_velocity = compute_target_velocity(distance, time_air);

    auto   delta  = compute_delta_from_target(velocity, heading, target_velocity, bearing);
    double dv     = delta[0];
    double dtheta = delta[1];

    prev_dv     = ema_filter(dv, prev_dv, av);
    prev_dtheta = ema_filter(dtheta, prev_dtheta, at);

    auto target_vec = compute_target_vector(distance, bearing);
    auto control    = compute_control_vector(target_vec, bearing);

    return control_to_servo_angle(control);
  }
};


// ======================================================
// CONTROLLER
// PID servo control loop
// ======================================================

struct Controller {

  xcore::pid_controller_t<decltype(millis())> pid_controllers[3] = {
    xcore::pid_controller_t<decltype(millis())>(KP, KI, KD),
    xcore::pid_controller_t<decltype(millis())>(KP, KI, KD),
    xcore::pid_controller_t<decltype(millis())>(KP, KI, KD),
  };

  ServoDriver driver;
  Guidance    guidance;
  double      last_angles[3] = {};
  int16_t     last_speeds[3] = {};

  // Per-servo direction sign; flips automatically when wrong-way travel is detected
  int8_t  dirs[3]        = {1, 1, 1};
  double  prev_angles[3] = {};
  double  wrong_accum[3] = {};

  static constexpr double SWAP_THRESHOLD_DEG = 100.0;

  void init_pid() {
    for (auto &pid: pid_controllers) {
      pid.update_limits(-3500, 3500);
      pid.update_dt(0.05);
    }
  }

  // Reset wrong-way accumulator when a new target is commanded for servo idx
  void reset_dir_accum(int idx) {
    wrong_accum[idx] = 0;
  }

  // ---- PID speed computation ----

  static int16_t compute_speed(
    xcore::pid_controller_t<uint32_t> &pid,
    const double                      &target_angle,
    const double                      &current_angle) {
    double speed = pid.update(target_angle, current_angle, 0.02);
    speed        = constrain(speed, -3500.0, 3500.0);

    return static_cast<int16_t>(speed);
  }

  // ---- Main servo PID update with automatic direction-swap ----

  void servo_pid_update(const numeric_vector<3> &target_angles) {
    static xcore::NbDelay delay(10, millis);

    delay([&]() {
      for (size_t i = 0; i < 3; ++i) {
        last_angles[i]  = driver.read_angle(i);
        double err      = target_angles[i] - last_angles[i];
        double motion   = last_angles[i] - prev_angles[i];
        int16_t speed   = 0;

        if (std::abs(err) >= 2.0) {
          bool moving_wrong = (err > 0 && motion < 0) || (err < 0 && motion > 0);

          if (moving_wrong && std::abs(last_speeds[i]) > 100) {
            wrong_accum[i] += std::abs(motion);
            if (wrong_accum[i] >= SWAP_THRESHOLD_DEG) {
              dirs[i]        = -dirs[i];
              wrong_accum[i] = 0;
            }
          } else {
            wrong_accum[i] = 0;
          }

          speed = static_cast<int16_t>(
            compute_speed(pid_controllers[i], target_angles[i], last_angles[i]) * dirs[i]);
        }

        prev_angles[i] = last_angles[i];
        last_speeds[i] = speed;
        driver.write_speed(ServoDriver::IDS[i], speed);
      }
    });
  }
};

#endif