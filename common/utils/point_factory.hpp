#pragma once

#include "common/data_structure.hpp"
#include "common/math/vec2d.hpp"

namespace common {
namespace utils {
class PointFactory {
public:
  template <typename XY> static inline math::Vec2d ToVec2d(const XY &xy) {
    return math::Vec2d(xy.x(), xy.y());
  }

  static inline common::SLPoint ToSLPoint(const double s, const double l) {
    common::SLPoint sl;
    sl.s = s;
    sl.l = l;
    return sl;
  }

  static inline common::PointENU ToPointENU(const double x, const double y,
                                            const double z = 0.0) {
    common::PointENU point_enu;
    point_enu.x = x;
    point_enu.y = y;
    point_enu.z = z;
    return point_enu;
  }

  template <typename XYZ>
  static inline common::PointENU ToPointENU(const XYZ &xyz) {
    return ToPointENU(xyz.x(), xyz.y(), xyz.z());
  }

  static inline SpeedPoint ToSpeedPoint(const double s, const double t,
                                        const double v = 0.0,
                                        const double a = 0.0,
                                        const double da = 0.0) {
    SpeedPoint speed_point;
    speed_point.s = s;
    speed_point.t = t;
    speed_point.v = v;
    speed_point.a = a;
    speed_point.da = da;
    return speed_point;
  }

  static inline common::PathPoint
  ToPathPoint(const double x, const double y, const double z = 0.0,
              const double s = 0.0, const double theta = 0.0,
              const double kappa = 0.0, const double dkappa = 0.0,
              const double ddkappa = 0.0) {
    common::PathPoint path_point;
    path_point.x = x;
    path_point.y = y;
    path_point.z = z;
    path_point.s = s;
    path_point.theta = theta;
    path_point.kappa = kappa;
    path_point.dkappa = dkappa;
    path_point.ddkappa = ddkappa;
    return path_point;
  }
};
} // namespace utils
} // namespace common