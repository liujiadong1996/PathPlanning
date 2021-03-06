#pragma once

#include <cmath>

#include "common/utils/math_utils.hpp"
#include "eigen3/Eigen/Geometry"


namespace common {
namespace math {

template <typename T>
class EulerAnglesZXY {
 public:
  /**
   * @brief Constructs an identity rotation
   */
  EulerAnglesZXY() : roll_(0), pitch_(0), yaw_(0) {}

  /**
   * @brief Constructs a rotation using only yaw (around the z-axis)
   * @param yaw The yaw of the vehicle
   */
  explicit EulerAnglesZXY(T yaw) : roll_(0), pitch_(0), yaw_(yaw) {}

  /**
   * @brief Constructs a rotation using arbitrary roll, pitch and yaw
   * @param roll The roll of the car
   * @param pitch The pitch of the car
   * @param yaw The yaw of the car
   */
  EulerAnglesZXY(T roll, T pitch, T yaw)
      : roll_(roll), pitch_(pitch), yaw_(yaw) {}

  /**
   * @brief Constructs a rotation using components of a quaternion
   * @param qw Quaternion w-coordinate
   * @param qx Quaternion x-coordinate
   * @param qy Quaternion y-coordinate
   * @param qz Quaternion z-coordinate
   */
  EulerAnglesZXY(T qw, T qx, T qy, T qz)
      : roll_(std::atan2(static_cast<T>(2.0) * (qw * qy - qx * qz),
                         static_cast<T>(2.0) *
                                 (utils::Square<T>(qw) + utils::Square<T>(qz)) -
                             static_cast<T>(1.0))),
        pitch_(std::asin(static_cast<T>(2.0) * (qw * qx + qy * qz))),
        yaw_(std::atan2(static_cast<T>(2.0) * (qw * qz - qx * qy),
                        static_cast<T>(2.0) *
                                (utils::Square<T>(qw) + utils::Square<T>(qy)) -
                            static_cast<T>(1.0))) {}

  /**
   * @brief Constructs a rotation from quaternion
   * @param q Quaternion
   */
  explicit EulerAnglesZXY(const Eigen::Quaternion<T> &q)
      : EulerAnglesZXY(q.w(), q.x(), q.y(), q.z()) {}

  T roll() const { return roll_; }

  T pitch() const { return pitch_; }

  T yaw() const { return yaw_; }

  void Normalize() {
    roll_ = utils::NormalizeAngle(roll_);
    pitch_ = utils::NormalizeAngle(pitch_);
    yaw_ = utils::NormalizeAngle(yaw_);
  }

  bool IsValid() {
    Normalize();
    return pitch_ < M_PI_2 && pitch_ > -M_PI_2;
  }

  /**
   * @brief Converts to a quaternion with a non-negative scalar part
   * @return Quaternion encoding this rotation
   */
  Eigen::Quaternion<T> ToQuaternion() const {
    T coeff = static_cast<T>(0.5);
    T r = roll_ * coeff;
    T p = pitch_ * coeff;
    T y = yaw_ * coeff;

    T sr = std::sin(r);
    T sp = std::sin(p);
    T sy = std::sin(y);

    T cr = std::cos(r);
    T cp = std::cos(p);
    T cy = std::cos(y);

    T qw = cr * cp * cy - sr * sp * sy;
    T qx = cr * sp * cy - sr * cp * sy;
    T qy = cr * sp * sy + sr * cp * cy;
    T qz = cr * cp * sy + sr * sp * cy;
    if (qw < 0.0) {
      return {-qw, -qx, -qy, -qz};
    }
    return {qw, qx, qy, qz};
  }

 private:
  T roll_;
  T pitch_;
  T yaw_;
};

using EulerAnglesZXYf = EulerAnglesZXY<float>;
using EulerAnglesZXYd = EulerAnglesZXY<double>;

}  // namespace math
}  // namespace common