#pragma once

#include <cmath>

#include "common/math/vec2d.hpp"

namespace common {
namespace utils {

using common::math::Vec2d;

double Sqr(const double x);

/**
 * @brief Cross product between two 2D vectors from the common start point,
 * and end at two other points
 * @param start_point The common start point of two vectors in 2D
 * @param end_point_1 The end point of the first vector
 * @param end_point_2 The end point of the second vector
 * @return The cross product result
 */
double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);

/**
 * @brief Wrap angle to [0, 2*PI)
 * @param angle the original value of the angle
 * @return The normalized value of the value
 */
double WrapAngle(const double angle);

/**
 * @brief Normalize angle to [-PI, PI)
 * @param angle the original value of the angle
 * @return The normalized value of the angle *
 */
double NormalizeAngle(const double angle);

/**
 * @brief Get a random integer between two integer values by a random seed
 * @param s The lower bound of the random integer
 * @param t The upper bound of the random integer
 * @param random_seed The random seed
 * @return A random integer between s and t based on the input random_seed
 */
int RandomInt(const int s, const int t, unsigned int rand_seed = 1);

/**
 * @brief Get a random double between two integervalues by a random seed
 * @param s The lower bound of the random double
 * @param t The upper bound of the random double
 * @param random_seed The random seed
 * @return A random double between s and t based on the input random_seed
 */
double RandomDouble(const double s, const double t, unsigned int rand_seed = 1);

/**
 * @brief Compute squared value
 * @param value The target value to get its squared value
 * @return Squared value of the input value.
 */
template <typename T>
inline T Square(const T value) {
  return value * value;
}
}  // namespace utils
}  // namespace common