#include <Arduino.h>
#include <cmath>

// --- XCORE Vector Alias ---
template<size_t Size>
using numeric_vector =
  LIB_XCORE_NAMESPACE::impl::numeric_vector_static_t<real_t, Size>;

// --- Structs ---

struct GPSCoordinate {
  double lat;
  double lon;
};

// --- Math Constants ---

constexpr double EARTH_RADIUS_M = 6371000.0;
constexpr double DEG_TO_RAD_VAL = 0.01745329251;
constexpr double RAD_TO_DEG_VAL = 57.2957795131;

// --- Navigation Helpers ---

double calculate_distance(GPSCoordinate p1, GPSCoordinate p2) {
  double dLat = (p2.lat - p1.lat) * DEG_TO_RAD_VAL;
  double dLon = (p2.lon - p1.lon) * DEG_TO_RAD_VAL;

  // Haversine formula component:
  // a = sin²(Δlat / 2)
  //   + cos(lat1) · cos(lat2) · sin²(Δlon / 2)
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

// --- Control Allocation (2D → 3 actuators) ---

numeric_vector<3> compute_control_vector(const numeric_vector<2>& target, double theta)
{
    constexpr double INV_SQRT3 = 0.57735026919;
    constexpr double TWO_INV_SQRT3 = 1.15470053838;
    constexpr double PItheta = 180.F;

    const double X = target[0];
    const double Y = target[1];
    
    double u1 = 0.0, u2 = 0.0, u3 = 0.0;

    // ----- u1 -----
    if (theta >= 0.0 && theta < 2.0 * PItheta / 3.0) {
        u1 = X + INV_SQRT3 * Y;
    }
    else if (theta >= -2.0 * PItheta / 3.0 && theta < 0.0) {
        u1 = X - INV_SQRT3 * Y;
    }

    // ----- u2 -----
    if (theta >= 0.0 && theta < 2.0 * PItheta / 3.0) {
        u2 = TWO_INV_SQRT3 * Y;
    }
    else if ((theta >= 2.0 * PItheta / 3.0 && theta <= PItheta) ||
             (theta > -PItheta && theta < -2.0 * PItheta / 3.0)) {
        u2 = -X + INV_SQRT3 * Y;
    }

    // ----- u3 -----
    if (theta >= -2.0 * PItheta / 3.0 && theta < 0.0) {
        u3 = -TWO_INV_SQRT3 * Y;
    }
    else if ((theta >= 2.0 * PItheta / 3.0 && theta <= PItheta) ||
             (theta > -PItheta && theta < -2.0 * PI / 3.0)) {
        u3 = -X - INV_SQRT3 * Y;
    }

    numeric_vector<3> out{};
    out[0] = max(0.0, u1);
    out[1] = max(0.0, u2);
    out[2] = max(0.0, u3);

    return out;
}


// --- Example ---

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
