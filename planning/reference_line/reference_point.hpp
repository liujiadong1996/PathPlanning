#pragma once

#include <string>
#include <vector>

#include "common/data_structure.hpp"
#include "common/math/path.hpp"

namespace planning {
namespace reference_line {

class ReferencePoint : public common::math::MapPathPoint {
 public:
  ReferencePoint() = default;

  ReferencePoint(const MapPathPoint &map_path_point, const double kappa,
                 const double dkappa);

  common::PathPoint ToPathPoint(double s) const;

  double kappa() const;
  double dkappa() const;

  std::string DebugString() const;

  static void RemoveDuplicates(std::vector<ReferencePoint> *points);

 private:
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
};
}  // namespace reference_line
}  // namespace planning