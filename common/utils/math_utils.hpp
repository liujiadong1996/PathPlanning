#pragma once

#include <cmath>

namespace common {
namespace utils {
/**
 * @brief Normalize angle to [-PI, PI)
 * @param angle the original value of the angle
 * @return The normalized value of the angle *
 */
double NormalizeAngle(const double angle);
} // namespace utils
} // namespace common