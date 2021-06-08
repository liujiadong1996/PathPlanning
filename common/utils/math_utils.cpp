#define _USE_MATH_DEFINES

#include "common/utils/math_utils.hpp"

#include <cmath>

namespace common {
namespace utils {
double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}
} // namespace utils
} // namespace common