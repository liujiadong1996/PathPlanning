#pragma once

namespace common {
namespace utils {

template <typename U, typename V> double DistanceXY(const U &u, const V &v) {
  return std::hypot(u.x() - v.x(), u.y() - v.y());
}
} // namespace utils
} // namespace common