#pragma once

#include <array>

#include "common/math/vec2d.hpp"

namespace common {
namespace frenet {

// s_condition = [s, s_dot, s_ddot]
// s: longitudinal coordinate
// s_dot: ds / dt
// s_ddot: d(s_dot) / dt
// d_condition = [d, d_prime, d_pprime]
// d: lateral coordinate
// d_prime: dd / ds
// d_pprime: d(d_prime) / ds
class CartesianFrenetConverter {
 public:
  CartesianFrenetConverter() = delete;
  /**
   * Convert a vehicle state in Cartesian frame to Frenet frame
   */
  static void cartesian_to_frenet(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const double x, const double y,
                                  const double v, const double a,
                                  const double theta, const double kappa,
                                  std::array<double, 3> *const ptr_s_condition,
                                  std::array<double, 3> *const ptr_d_condition);

  /**
   * Convert a vehicle state in Frenet frame to Cartesian frame
   */
  static void frenet_to_cartesian(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const std::array<double, 3> &s_condition,
                                  const std::array<double, 3> &d_condition,
                                  double *const ptr_x, double *const ptr_y,
                                  double *const ptr_theta,
                                  double *const ptr_kappa, double *const ptr_v,
                                  double *const ptr_a);

  static double CalculateTheta(const double rtheta, const double rkappa,
                               const double l, const double dl);

  static double CalculateKappa(const double rkappa, const double rdkappa,
                               const double l, const double dl,
                               const double ddl);

  static math::Vec2d CalculateCartesianPoint(const double rtheta,
                                             const math::Vec2d &rpoint,
                                             const double l);

  /**
   * @brief: given sl, theta and road's theta, kappa, extract derivative l,
   */
  static double CalculateLateralDerivative(const double rtheta,
                                           const double theta, const double l,
                                           const double rkappa);

  /**
   * @brief: extract second derivative l
   */
  static double CalculateSecondOrderLateralDerivative(
      const double rtheta, const double theta, const double rkappa,
      const double kappa, const double rdkappa, const double l);
};

}  // namespace frenet
}  // namespace common
