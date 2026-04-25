#ifndef MINI_FC_FIRMWARE_KALMAN_H
#define MINI_FC_FIRMWARE_KALMAN_H

#include <lib_xcore>
#include <xcore/math_module>

// Kalman Filters
constexpr size_t FILTER_ORDER = 3;
constexpr double BASE_NOISE   = 0.5;

// =============================================================================
// Altitude Kalman filter
// States  : [altitude_m, velocity_m_s, accel_m_s2]
// Meas    : [altitude_m]  — BMP581, 10 Hz, σ ≈ 0.5 m
// Predict : 100 Hz (dt = 10 ms)
//
// R  = σ² of BMP581 altitude measurement
// Q  = scalar diagonal process noise.
//      Large Q/R → K ≈ 1 (no smoothing); small Q/R → K ≈ 0 (no tracking).
//      Target K ≈ 0.55–0.65 for a balance between smoothing and flight tracking.
//      Q/R = 1 → K ≈ 0.62 (scalar 1D approximation).
// alpha: slow IAE adaptation — altitude estimates need stability over speed
// tau  : longer innovation window for the same reason
// =============================================================================
constexpr double KF_ALT_Q     = 0.25;  // (m, m/s, m/s²)²  per predict step  Q/R = 1
constexpr double KF_ALT_R     = 0.25;  // m²  (σ ≈ 0.5 m)
constexpr double KF_ALT_ALPHA = 0.10;
constexpr double KF_ALT_TAU   = 8.0;

struct FilterAlt {
  static constexpr xcore::numeric_matrix<FILTER_ORDER, 1> B  = xcore::make_numeric_matrix<FILTER_ORDER, 1>();
  static constexpr xcore::numeric_matrix<1, FILTER_ORDER> H  = xcore::make_numeric_matrix<1, FILTER_ORDER>({{1, 0, 0}});
  static constexpr xcore::numeric_vector<FILTER_ORDER>    x0 = xcore::make_numeric_vector<FILTER_ORDER>();
  static constexpr xcore::numeric_matrix<FILTER_ORDER>    P0 = xcore::numeric_matrix<FILTER_ORDER>::diagonals(1000.);

  xcore::numeric_matrix<FILTER_ORDER> F = xcore::make_numeric_matrix<FILTER_ORDER>();
  xcore::numeric_matrix<FILTER_ORDER> Q = xcore::numeric_matrix<FILTER_ORDER>::diagonals(KF_ALT_Q);
  xcore::numeric_matrix<1>            R = xcore::numeric_matrix<1>::diagonals(KF_ALT_R);

  xcore::r_iae_kalman_filter_t<FILTER_ORDER, 1, 1> kf{
    F, B, H, Q, R, x0, P0,
    /*alpha*/ KF_ALT_ALPHA,
    /*beta*/  0.00,
    /*tau*/   KF_ALT_TAU,
    /*eps*/   1.e-12};
};

// =============================================================================
// Acceleration Kalman filter
// States  : [accel_m_s2, jerk_m_s3, snap_m_s4]
// Meas    : [accel_m_s2]  — ISM330 magnitude, 100 Hz, σ ≈ 0.1 m/s²
// Predict : 100 Hz (dt = 10 ms)
//
// R  = σ² of ISM330 magnitude at ±16 g; use 0.1 m/s² conservatively → R = 0.01
// Q  = process noise for the jerk-driven 3-state model.
//      Q/R = 1 → K ≈ 0.62.  Enough smoothing to suppress IMU noise at rest
//      while the IAE adaptive R raises gain during burnout/apogee transitions.
// alpha: faster IAE adaptation — accel must track transients quickly
// tau  : shorter window, responsive to sudden changes
// =============================================================================
constexpr double KF_ACC_Q     = 0.01;  // (m/s², m/s³, m/s⁴)²  per predict step  Q/R = 1
constexpr double KF_ACC_R     = 0.01;  // (m/s²)²  (σ ≈ 0.1 m/s²)
constexpr double KF_ACC_ALPHA = 0.15;
constexpr double KF_ACC_TAU   = 4.0;

struct FilterAcc {
  static constexpr xcore::numeric_matrix<FILTER_ORDER, 1> B  = xcore::make_numeric_matrix<FILTER_ORDER, 1>();
  static constexpr xcore::numeric_matrix<1, FILTER_ORDER> H  = xcore::make_numeric_matrix<1, FILTER_ORDER>({{1, 0, 0}});
  static constexpr xcore::numeric_vector<FILTER_ORDER>    x0 = xcore::make_numeric_vector<FILTER_ORDER>();
  static constexpr xcore::numeric_matrix<FILTER_ORDER>    P0 = xcore::numeric_matrix<FILTER_ORDER>::diagonals(1000.);

  xcore::numeric_matrix<FILTER_ORDER> F = xcore::make_numeric_matrix<FILTER_ORDER>();
  xcore::numeric_matrix<FILTER_ORDER> Q = xcore::numeric_matrix<FILTER_ORDER>::diagonals(KF_ACC_Q);
  xcore::numeric_matrix<1>            R = xcore::numeric_matrix<1>::diagonals(KF_ACC_R);

  xcore::r_iae_kalman_filter_t<FILTER_ORDER, 1, 1> kf{
    F, B, H, Q, R, x0, P0,
    /*alpha*/ KF_ACC_ALPHA,
    /*beta*/  0.00,
    /*tau*/   KF_ACC_TAU,
    /*eps*/   1.e-12};
};

// Generic single-measurement filter kept for MAIN target
struct Filter1T {
  static constexpr xcore::numeric_matrix<FILTER_ORDER, 1> B  = xcore::make_numeric_matrix<FILTER_ORDER, 1>();
  static constexpr xcore::numeric_matrix<1, FILTER_ORDER> H  = xcore::make_numeric_matrix<1, FILTER_ORDER>({
    {1, 0, 0},
  });
  static constexpr xcore::numeric_vector<FILTER_ORDER>    x0 = xcore::make_numeric_vector<FILTER_ORDER>();
  static constexpr xcore::numeric_matrix<FILTER_ORDER>    P0 = xcore::numeric_matrix<FILTER_ORDER>::diagonals(1000.);

  xcore::numeric_matrix<FILTER_ORDER> F = xcore::make_numeric_matrix<FILTER_ORDER>();
  xcore::numeric_matrix<FILTER_ORDER> Q = xcore::numeric_matrix<FILTER_ORDER>::diagonals(BASE_NOISE);
  xcore::numeric_matrix<1>            R = xcore::numeric_matrix<1>::diagonals(BASE_NOISE);

  xcore::r_iae_kalman_filter_t<FILTER_ORDER, 1, 1> kf{F, B, H, Q, R, x0, P0,
                                                      /*alpha*/ 0.20,
                                                      /*beta*/  0.00,
                                                      /*tau*/   4.0,
                                                      /*eps*/   1.e-12};
};

struct Filter2T {
  static constexpr xcore::numeric_matrix<FILTER_ORDER, 1> B  = xcore::make_numeric_matrix<FILTER_ORDER, 1>();
  static constexpr xcore::numeric_matrix<2, FILTER_ORDER> H  = xcore::make_numeric_matrix<2, FILTER_ORDER>({
    {1, 0, 0},
    {0, 1, 0}
  });
  static constexpr xcore::numeric_vector<FILTER_ORDER>    x0 = xcore::make_numeric_vector<FILTER_ORDER>();
  static constexpr xcore::numeric_matrix<FILTER_ORDER>    P0 = xcore::numeric_matrix<FILTER_ORDER>::diagonals(1000.);

  xcore::numeric_matrix<FILTER_ORDER> F = xcore::make_numeric_matrix<FILTER_ORDER>();
  xcore::numeric_matrix<FILTER_ORDER> Q = xcore::numeric_matrix<FILTER_ORDER>::diagonals(BASE_NOISE);
  xcore::numeric_matrix<2>            R = xcore::numeric_matrix<2>::diagonals(BASE_NOISE);

  xcore::r_iae_kalman_filter_t<FILTER_ORDER, 2, 1> kf{F, B, H, Q, R, x0, P0,
                                                      /*alpha*/ 0.20,  // Enable Adaptive R, a > 0
                                                      /*beta*/ 0.00,   // Disable Adaptive Q, b = 0
                                                      /*tau*/ 4.0,
                                                      /*eps*/ 1.e-12};
  // xcore::kalman_filter_t<FILTER_ORDER, 2, 1> kf{F, B, H, Q, R, x0, P0};
};

#endif  //MINI_FC_FIRMWARE_KALMAN_H
