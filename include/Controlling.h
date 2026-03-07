#ifndef CONTROLLING_H
#define CONTROLLING_H

#include <Arduino.h>
#include <cmath>
#include <lib_xcore>
#include <xcore/math_module>

// --- XCORE Vector Alias ---
template<size_t Size>
using numeric_vector = xcore::impl::numeric_vector_static_t<double, Size>;

// --- Structs ---
struct GPSCoordinate {
  double lat;
  double lon;
};

// --- Math Constants ---

constexpr double EARTH_RADIUS_M = 6371000.0;
constexpr double DEG_TO_RAD_VAL = 0.01745329251;
constexpr double RAD_TO_DEG_VAL = 57.2957795131;

// -------- Distance (Haversine) --------
double calculate_distance(GPSCoordinate p1, GPSCoordinate p2)
{
  double dLat = (p2.lat - p1.lat) * DEG_TO_RAD;
  double dLon = (p2.lon - p1.lon) * DEG_TO_RAD;

  double a =
    sin(dLat/2)*sin(dLat/2) +
    cos(p1.lat*DEG_TO_RAD) *
    cos(p2.lat*DEG_TO_RAD) *
    sin(dLon/2)*sin(dLon/2);

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));

  return EARTH_RADIUS_M * c;
}

// -------- Bearing --------
double calculate_bearing(GPSCoordinate p1, GPSCoordinate p2)
{
  double lat1 = p1.lat * DEG_TO_RAD;
  double lat2 = p2.lat * DEG_TO_RAD;
  double dLon = (p2.lon - p1.lon) * DEG_TO_RAD;

  double y = sin(dLon) * cos(lat2);
  double x =
      cos(lat1) * sin(lat2) -
      sin(lat1) * cos(lat2) * cos(dLon);

  double bearing = atan2(y,x) * RAD_TO_DEG;

  return fmod(bearing + 360.0, 360.0);
}

// -------- Control Allocation --------
numeric_vector<3> compute_control_vector(const numeric_vector<2>& target, double theta)
{
  constexpr double INV_SQRT3 = 0.57735026919;
  constexpr double TWO_INV_SQRT3 = 1.15470053838;

  const double X = target[0];
  const double Y = target[1];

  double u1=0,u2=0,u3=0;

  if(theta >= 0 && theta < 120)
      u1 = X + INV_SQRT3*Y;
  else if(theta >= -120 && theta < 0)
      u1 = X - INV_SQRT3*Y;

  if(theta >= 0 && theta < 120)
      u2 = TWO_INV_SQRT3*Y;
  else if((theta >=120 && theta<=180) || (theta>-180 && theta<-120))
      u2 = -X + INV_SQRT3*Y;

  if(theta >= -120 && theta < 0)
      u3 = -TWO_INV_SQRT3*Y;
  else if((theta >=120 && theta<=180) || (theta>-180 && theta<-120))
      u3 = -X - INV_SQRT3*Y;

  numeric_vector<3> out{};
  out[0] = max(0.0,u1);
  out[1] = max(0.0,u2);
  out[2] = max(0.0,u3);

  return out;
}

// -------- Main Path Calculation --------
numeric_vector<3> path_calculation(GPSCoordinate current, GPSCoordinate target)
{
  // Step 1: Distance + bearing
  double distance = calculate_distance(current, target);
  double bearing  = calculate_bearing(current, target);

  // Step 2: convert 0..360 → -180..180
  if(bearing > 180)
      bearing -= 360;

  // Step 3: convert to XY vector
  numeric_vector<2> v{};
  v[0] = distance * cos(bearing * DEG_TO_RAD);
  v[1] = distance * sin(bearing * DEG_TO_RAD);

  // Step 4: control allocation
  return compute_control_vector(v, bearing);
}

void PID_Control(double error, float dt, float &output)
{
    static float integral = 0;
    static float prev_error = 0;

    float Kp = 1.0;
    float Ki = 0.0;
    float Kd = 0.0;

    // Integral
    integral += error * dt;

    // Derivative
    float derivative = (error - prev_error) / dt;

    // PID output
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    prev_error = error;
}

#endif //CONTROLLING_H