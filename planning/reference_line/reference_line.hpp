#pragma once

#include <string>
#include <utility>
#include <vector>

#include "common/data_structure.hpp"
#include "common/math/box2d.hpp"
#include "common/math/path.hpp"
#include "common/math/polygon2d.hpp"
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
  ToFrenetFrame(const common::TrajectoryPoint &traj_point) const;

  std::vector<ReferencePoint> GetReferencePoints(double start_s,
                                                 double end_s) const;

  size_t GetNearestReferenceIndex(const double s) const;

  ReferencePoint GetNearestReferencePoint(const common::math::Vec2d &xy) const;

  std::vector<common::math::LaneSegment>
  GetLaneSegments(const double start_s, const double end_s) const;

  ReferencePoint GetNearestReferencePoint(const double s) const;

  ReferencePoint GetReferencePoint(const double x, const double y) const;

  bool GetApproximateSLBoundary(const common::math::Box2d &box,
                                const double start_s, const double end_s,
                                common::SLBoundary *const sl_boundary) const;

  bool GetSLBoundary(const common::math::Box2d &box,
                     common::SLBoundary *const sl_boundary) const;
  bool GetSLBoundary(const common::math::Polygon2d &polygon,
                     common::SLBoundary *const sl_boundary) const;

  bool SLToXY(const common::SLPoint &sl_point,
              common::math::Vec2d *const xy_point) const;
  bool XYToSL(const common::math::Vec2d &xy_point,
              common::SLPoint *const sl_point) const;
  bool XYToSL(const common::PathPoint &xy,
              common::SLPoint *const sl_point) const {
    return XYToSL(common::math::Vec2d(xy.x, xy.y), sl_point);
  }
  template <class XYPoint>
  bool XYToSL(const XYPoint &xy, common::SLPoint *const sl_point) const {
    return XYToSL(common::math::Vec2d(xy.x(), xy.y()), sl_point);
  }

  bool GetLaneWidth(const double s, double *const lane_left_width,
                    double *const lane_right_width) const;

  bool GetOffsetToMap(const double s, double *const l_offset) const;

  bool GetRoadWidth(const double s, double *const road_left_width,
                    double *const road_right_width) const;

  common::Road::Type GetRoadType(const double s) const;

  void GetLaneFromS(const double s,
                    std::vector<common::map::LaneInfoConstPtr> *lanes) const;

  double GetDrivingWidth(const common::SLBoundary &sl_boundary) const;

  bool IsOnLane(const common::SLPoint &sl_point) const;
  bool IsOnLane(const common::math::Vec2d &vec2d_point) const;
  template <class XYPoint> bool IsOnLane(const XYPoint &xy) const {
    return IsOnLane(common::math::Vec2d(xy.x(), xy.y()));
  }
  bool IsOnLane(const common::SLBoundary &sl_boundary) const;

  bool IsOnRoad(const common::SLPoint &sl_point) const;
  bool IsOnRoad(const common::math::Vec2d &vec2d_point) const;
  bool IsOnRoad(const common::SLBoundary &sl_boundary) const;

  bool IsBlockRoad(const common::math::Box2d &box2d, double gap) const;

  bool HasOverlap(const common::math::Box2d &box) const;

  double Length() const { return map_path_.length(); }

  std::string DebugString() const;

  double GetSpeedLimitFromS(const double s) const;

  void AddSpeedLimit(double start_s, double end_s, double speed_limit);

  uint32_t GetPriority() const { return priority_; }

  void SetPriority(uint32_t priority) { priority_ = priority; }

  const common::math::Path &GetMapPath() const { return map_path_; }

private:
  static ReferencePoint Interpolate(const ReferencePoint &p0, const double s0,
                                    const ReferencePoint &p1, const double s1,
                                    const double s);
  ReferencePoint InterpolateWithMatchedIndex(
      const ReferencePoint &p0, const double s0, const ReferencePoint &p1,
      const double s1, const common::math::InterpolatedIndex &index) const;

  static double FindMinDistancePoint(const ReferencePoint &p0, const double s0,
                                     const ReferencePoint &p1, const double s1,
                                     const double x, const double y);

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