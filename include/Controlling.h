#ifndef CONTROLLING_H
#define CONTROLLING_H

#include <Arduino.h>
#include <cmath>
#include <lib_xcore>
#include <xcore/math_module>

// vector alias for XCORE
template<size_t Size>
using numeric_vector = xcore::impl::numeric_vector_static_t<double, Size>;

struct GPSCoordinate {
  double lat;
  double lon;
};

constexpr double EARTH_RADIUS_M = 6371000.0;
constexpr double DEG_TO_RAD_VAL = 0.01745329251;
constexpr double RAD_TO_DEG_VAL = 57.2957795131;

// --- Navigation ---

double calculate_distance(GPSCoordinate p1, GPSCoordinate p2) {
  double dLat = (p2.lat - p1.lat) * DEG_TO_RAD_VAL;
  double dLon = (p2.lon - p1.lon) * DEG_TO_RAD_VAL;

  double a =
    sin(dLat / 2) * sin(dLat / 2) +
    cos(p1.lat * DEG_TO_RAD_VAL) *
      cos(p2.lat * DEG_TO_RAD_VAL) *
      sin(dLon / 2) * sin(dLon / 2);

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return EARTH_RADIUS_M * c;
}

double calculate_bearing(GPSCoordinate p1, GPSCoordinate p2) {
  double lat1 = p1.lat * DEG_TO_RAD_VAL;
  double lat2 = p2.lat * DEG_TO_RAD_VAL;
  double dLon = (p2.lon - p1.lon) * DEG_TO_RAD_VAL;

  double y = sin(dLon) * cos(lat2);
  double x =
    cos(lat1) * sin(lat2) -
    sin(lat1) * cos(lat2) * cos(dLon);

  double bearing = atan2(y, x) * RAD_TO_DEG_VAL;
  return fmod(bearing + 360.0, 360.0);
}

// --- Control Allocation: 2D command -> 3 rope outputs ---
// theta is the vehicle heading in degrees, range (-180, 180].
// The three ropes are 120 deg apart. Output values are clamped >= 0.

numeric_vector<3> compute_control_vector(const numeric_vector<2>& target, double theta)
{
  constexpr double INV_SQRT3     = 0.57735026919;
  constexpr double TWO_INV_SQRT3 = 1.15470053838;

  // Normalize theta to (-180, 180]
  while (theta >  180.0) theta -= 360.0;
  while (theta <= -180.0) theta += 360.0;

  const double X = target[0];
  const double Y = target[1];

  double u1 = 0.0, u2 = 0.0, u3 = 0.0;

  if (theta >= 0.0 && theta < 120.0) {
    // Sector 1: rope 1 and rope 2 active
    u1 = X + INV_SQRT3 * Y;
    u2 = TWO_INV_SQRT3 * Y;
  } else if (theta >= 120.0 && theta <= 180.0) {
    // Sector 2: rope 2 and rope 3 active
    u2 = -X + INV_SQRT3 * Y;
    u3 = -X - INV_SQRT3 * Y;
  } else if (theta >= -120.0 && theta < 0.0) {
    // Sector 3: rope 1 and rope 3 active
    u1 = X - INV_SQRT3 * Y;
    u3 = -TWO_INV_SQRT3 * Y;
  } else {
    // Sector 4 (theta < -120): same allocation as sector 2
    u2 = -X + INV_SQRT3 * Y;
    u3 = -X - INV_SQRT3 * Y;
  }

  numeric_vector<3> out{};
  out[0] = max(0.0, u1);
  out[1] = max(0.0, u2);
  out[2] = max(0.0, u3);
  return out;
}

// --- PID ---
// Gains and limits are kept in a separate struct so they can be tuned
// without recompiling the control function itself.

struct PID_Gains {
  float Kp             = 1.0f;
  float Ki             = 0.05f;  // small integral to correct steady-state drift
  float Kd             = 0.10f;
  float integral_limit = 30.0f;  // anti-windup symmetric clamp
  float output_limit   = 90.0f;  // servo travel limit in degrees
};

// Runtime state for one PID channel. Declare one per axis.
struct PID_State {
  float integral   = 0.0f;
  float prev_error = 0.0f;

  void reset() {
    integral   = 0.0f;
    prev_error = 0.0f;
  }
};

// Returns the clamped control output for the given error and timestep.
// dt must be in seconds. Call reset() on the state when re-entering the
// control loop after a pause to prevent a derivative spike.
float PID_Compute(float error, float dt, PID_State& st, const PID_Gains& gains)
{
  if (dt < 1.0e-6f) {
    // timestep too small to differentiate reliably; skip this tick
    return 0.0f;
  }

  // Integral with anti-windup clamp
  st.integral += error * dt;
  if (st.integral >  gains.integral_limit) st.integral =  gains.integral_limit;
  if (st.integral < -gains.integral_limit) st.integral = -gains.integral_limit;

  float derivative = (error - st.prev_error) / dt;
  st.prev_error    = error;

  float output = gains.Kp * error + gains.Ki * st.integral + gains.Kd * derivative;

  // Clamp output to servo range
  if (output >  gains.output_limit) output =  gains.output_limit;
  if (output < -gains.output_limit) output = -gains.output_limit;

  return output;
}

#endif // CONTROLLING_H