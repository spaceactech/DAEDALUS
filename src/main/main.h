#ifndef ROCKET_AVIONICS_TEMPLATE_MAIN_H
#define ROCKET_AVIONICS_TEMPLATE_MAIN_H

#include <Arduino.h>  // Arduino Framework
#include <math.h>

/* Telemetry control */
bool telemetry_enabled = false;

/* Simulation mode */
bool     simEnabled   = false;
bool     simActivated = false;
float    simPressure  = 101325.0f;  // default: sea-level in Pa, overwritten by SIMP command
String   rx_message = "";

/* ACK / NACK counters */
uint32_t last_ack;
uint32_t last_nack;

struct valid {
  bool ina;
  bool lis;
  bool m10s;
  bool tof;
} pvalid;

extern void ReadINA();

extern void ReadMAG();

extern void ReadTOF();

extern void AutoZeroAlt();

extern void HandleCommand(const String &rx);

/* =========================
   2D Vector structure
   ========================= */
struct Vec2 {
  float x;
  float y;
};

/* =========================
   Unit vectors (0°, 120°, 240°)
   ========================= */
const Vec2 U1 = {1.0f, 0.0f};
const Vec2 U2 = {-0.5f, 0.8660254f};
const Vec2 U3 = {-0.5f, -0.8660254f};

/* =========================
   Dot product
   ========================= */
float dot(const Vec2 &a, const Vec2 &b) {
  return a.x * b.x + a.y * b.y;
}

/* =========================
   Projection
   ========================= */
Vec2 project(const Vec2 &v, const Vec2 &u) {
  float k = dot(v, u);
  return {k * u.x, k * u.y};
}

/* =========================
   Heading-based projection
   ========================= */
Vec2 sectorProjectionFromHeading(float heading_deg) {

  // Normalize heading to [-180, 180)
  while (heading_deg >= 180.0f) heading_deg -= 360.0f;
  while (heading_deg < -180.0f) heading_deg += 360.0f;

  // Create unit vector from heading
  float rad = heading_deg * PI / 180.0f;
  Vec2  v   = {cos(rad), sin(rad)};

  // Sector logic (π = 180°)
  if (heading_deg >= 0.0f && heading_deg < 120.0f) {
    return project(v, U1);
  } else if (heading_deg >= 120.0f || heading_deg < -120.0f) {
    return project(v, U2);
  } else {
    return project(v, U3);
  }
}

#endif  //ROCKET_AVIONICS_TEMPLATE_MAIN_H
