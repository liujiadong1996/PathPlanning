#pragma once

#include <vector>

#include "common/map/map_common.hpp"
#include "vec2d.hpp"

namespace common {
namespace math {

struct LaneWaypoint {
  LaneWaypoint() = default;
  // TODO(liujiadong) glog??
  LaneWaypoint(map::LaneInfoConstPtr lane, const double s)
      //   : lane(CHECK_NOTNULL(lane)), s(s) {}
      : lane(lane), s(s) {}
  LaneWaypoint(map::LaneInfoConstPtr lane, const double s, const double l)
      //     : lane(CHECK_NOTNULL(lane)), s(s), l(l) {}
      : lane(lane), s(s), l(l) {}
  // TODO(liujiadong) define map info
  map::LaneInfoConstPtr lane = nullptr;
  double s = 0.0;
  double l = 0.0;

  std::string DebugString() const;
};

struct LaneSegment {
  LaneSegment() = default;
  LaneSegment(map::LaneInfoConstPtr lane, const double start_s,
              const double end_s)
      // : lane(CHECK_NOTNULL(lane)), start_s(start_s), end_s(end_s) {}
      : lane(lane), start_s(start_s), end_s(end_s) {}
  map::LaneInfoConstPtr lane = nullptr;
  double start_s = 0.0;
  double end_s = 0.0;
  double Length() const { return end_s - start_s; }

  std::string DebugString() const;
};

class MapPathPoint : public Vec2d {
public:
  MapPathPoint() = default;
  MapPathPoint(const Vec2d &point, double heading)
      : Vec2d(point.x(), point.y()), heading_(heading) {}
  MapPathPoint(const Vec2d &point, double heading, LaneWaypoint lane_waypoints)
      : Vec2d(point.x(), point.y()), heading_(heading) {
    lane_waypoints_.emplace_back(std::move(lane_waypoints));
  }
  MapPathPoint(const Vec2d &point, double heading,
               std::vector<LaneWaypoint> lane_waypoints)
      : Vec2d(point.x(), point.y()), heading_(heading),
        lane_waypoints_(std::move(lane_waypoints)) {}

  double heading() const { return heading_; }
  void set_heading(const double heading) { heading_ = heading; }

  const std::vector<LaneWaypoint> &lane_waypoints() const {
    return lane_waypoints_;
  }

  void add_lane_waypoint(LaneWaypoint lane_waypoint) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  }
  void add_lane_waypoints(const std::vector<LaneWaypoint> &lane_waypoints) {
    lane_waypoints_.insert(lane_waypoints_.end(), lane_waypoints.begin(),
                           lane_waypoints.end());
  }

  void clear_lane_waypoints() { lane_waypoints_.clear(); }

  static void RemoveDuplicates(std::vector<MapPathPoint> *points);

  static std::vector<MapPathPoint>
  GetPointsFromSegment(const LaneSegment &segment);

  static std::vector<MapPathPoint> GetPointsFromLane(map::LaneInfoConstPtr lane,
                                                     const double start_s,
                                                     const double end_s);

  std::string DebugString() const;

protected:
  double heading_ = 0.0;
  std::vector<LaneWaypoint> lane_waypoints_;
};

class Path;

class PathApproximation {
  // TODO(liujiadong)
public:
protected:
};

class Path {
public:
protected:
  int num_points_ = 0;
  int num_segments_ = 0;
  std::vector<MapPathPoint> path_points_;
  std::vector<LaneSegment> lane_segments_;
  std::vector<double> lane_accumulated_s_;
  std::vector<LaneSegment> lane_segments_to_next_point_;
  std::vector<Vec2d> unit_directions_;
  double lenght_ = 0.0;
  std::vector<double> accumulated_s_;
  std::vector<LineSegment2d> segments_;
  bool use_path_approximation_ = false;
  PathApproximation approximation_;
};

} // namespace math
} // namespace common