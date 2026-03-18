#ifndef ROCKET_AVIONICS_TEMPLATE_ISA76_H
#define ROCKET_AVIONICS_TEMPLATE_ISA76_H

#include <./Arduino_Extended.h>

namespace isa76 {
  // Constants
  constexpr double g0 = 9.80665;    // m/s^2
  constexpr double Rd = 287.05287;  // J/(kg·K)
  constexpr double Re = 6356766.0;  // m  (gravity-equivalent Earth radius)

  // ISA1976 base (geopotential) heights [m], base temperatures [K], base pressures [Pa],
  // and lapse rates L = dT/dh [K/m] for layers up to 86 km.
  // Layers: [0–11 km], [11–20], [20–32], [32–47], [47–51], [51–71], [71–86]
  static constexpr int N         = 7;
  constexpr double     hb[N]     = {0.0, 11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0};
  constexpr double     Tb[N]     = {288.150, 216.650, 216.650, 228.650, 270.650, 270.650, 214.650};
  constexpr double     Pb_std[N] = {  // Pa at layer bases under ISA (sea-level 101325 Pa)
    101325.00, 22632.06, 5474.889, 868.0187, 110.9063, 66.93887, 3.956420};
  constexpr double     Lb[N]     = {-0.0065, 0.0, +0.0010, +0.0028, 0.0, -0.0028, -0.0020};

  // Convert geopotential -> geometric altitude (meters)
  inline double geo_from_geopot(const double H) {
    return (Re * H) / (Re - H);
  }

  inline double geopotential_from_pressure(double p_Pa, double qnh_Pa) {
    // Scale all base pressures so sea-level matches QNH
    double       Pb[N];
    const double k = qnh_Pa / 101325.0;
    for (int i = 0; i < N; ++i) Pb[i] = Pb_std[i] * k;

    // Below/above sea level handling:
    if (p_Pa > Pb[0]) {
      // Below MSL: use first (troposphere) gradient layer with h < 0
      const double L = Lb[0], T0 = Tb[0], P0 = Pb[0], h0 = hb[0];
      const double a = (Rd * L) / g0;              // ≈ -0.0065*Rd/g0 ≈ -0.190263
      const double t = std::pow(p_Pa / P0, -a);    // exponent is positive (~0.190263)
      const double H = h0 + (T0 / L) * (t - 1.0);  // L < 0 ⇒ H becomes negative for p>P0
      return H;                                    // geopotential (can be negative)
    }

    // Find layer i with Pb[i] >= p > Pb[i+1] (or top layer)
    int i = 0;
    while (i + 1 < N && p_Pa <= Pb[i + 1]) ++i;

    const double L = Lb[i], T0 = Tb[i], P0 = Pb[i], h0 = hb[i];

    if (L == 0.0) {
      // Isothermal: p = P0 * exp[-g0 (H-h0)/(Rd T0)]
      const double H = h0 - (Rd * T0 / g0) * std::log(p_Pa / P0);
      return H;
    } else {
      // Gradient: p = P0 * (T/T0)^(-g0/(Rd L)), T = T0 + L(H-h0)
      const double a = (Rd * L) / g0;
      const double t = std::pow(p_Pa / P0, -a);
      const double H = h0 + (T0 / L) * (t - 1.0);
      return H;
    }
  }
}  // namespace isa76

// Altitude above Mean Sea Level (meters) from static pressure (hPa) and QNH (hPa).
// If you don't have QNH, leave default 1013.25 for pure ISA.
inline double altitude_msl_from_pressure(const double p_hpa,
                                         const double qnh_hpa = 1013.25) noexcept {
  const double p   = std::max(0.1, p_hpa) * 100.0;
  const double qnh = std::max(0.1, qnh_hpa) * 100.0;
  const double H   = isa76::geopotential_from_pressure(p, qnh);
  return isa76::geo_from_geopot(H);
}

#endif  //ROCKET_AVIONICS_TEMPLATE_ISA76_H
