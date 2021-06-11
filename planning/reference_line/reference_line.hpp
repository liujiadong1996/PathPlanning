#pragma once

#include <string>
#include <utility>
#include <vector>

#include "common/math/path.hpp"
#include "common/math/vec2d.hpp"
#include "planning/reference_line/reference_point.hpp"

namespace planning {
namespace reference_line {

class ReferenceLine {
public:
  ReferenceLine() = default;
  explicit ReferenceLine(const ReferenceLine &reference_line) = default;
  template <typename Iterator>
  ReferenceLine(const Iterator begin, const Iterator end)
      : reference_points_(begin, end),
        map_path_(
            std::move(std::vector<common::math::MapPathPoint>(begin, end))) {}
  explicit ReferenceLine(const std::vector<ReferencePoint> &reference_points);
  explicit ReferenceLine(const common::math::Path &hdmap_path);

  /**
   * @brief Stitich current reference line with the other reference line. The
   * stitching is to use current reference points as mush as possible
   * @return false if these two reference line cannot be stitced
   */
  bool Stitich(const ReferenceLine &other);

  bool Segment(const common::math::Vec2d &point, const double distance_backward,
               const double distance_forward);

  bool Segment(const double s, const double distance_backward,
               const double distance_forward);

  const common::math::Path &map_path() const;
  const std::vector<ReferencePoint> &reference_points() const;

  ReferencePoint GetReferencePoint(const double s) const;

  common::FrenetFramePoint
  GetFrenetPoint(const common::PathPoint &path_point) const;

  std::pair<std::array<double, 3>, std::array<double, 3>>
  ToFrenetFrame(const common::TrajectoryPoint &traj_point);

  std::vector<ReferencePoint> GetReferencePoints(double start_s,
                                                 double end_s) const;

  size_t GetNearestReferenceIndex(const double s) const;

  ReferencePoint GetNearestReferencePoint(const common::math::Vec2d &xy) const;

  std::vector<common::math::LaneSegment>
  GetLaneSegments(const double start_s, const double end_s) const;

  ReferencePoint GetNearestReferencePoint(const double s) const;

  ReferencePoint GetReferencePoint(const double x, const double y) const;

  // bool GetApproximateSLBoundary(const common::math::)

private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;
    SpeedLimit() = default;
    SpeedLimit(double start_s, double end_s, double speed_limit)
        : start_s(start_s), end_s(end_s), speed_limit(speed_limit) {}
  };

  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;
  common::math::Path map_path_;
  uint32_t priority_ = 0;
};
} // namespace reference_line
} // namespace planning