#ifndef MINI_FC_FIRMWARE_KALMAN_H
#define MINI_FC_FIRMWARE_KALMAN_H

#include <lib_xcore>
#include <xcore/math_module>

// Kalman Filters
constexpr size_t FILTER_ORDER = 3;
constexpr double BASE_NOISE   = 0.5;

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
                                                      /*alpha*/ 0.20,  // Enable Adaptive R, a > 0
                                                      /*beta*/ 0.00,   // Disable Adaptive Q, b = 0
                                                      /*tau*/ 4.0,
                                                      /*eps*/ 1.e-12};
  // xcore::kalman_filter_t<FILTER_ORDER, 1, 1> kf{F, B, H, Q, R, x0, P0};
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
