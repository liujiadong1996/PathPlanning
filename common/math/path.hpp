#pragma once

#include <vector>

#include "adapter/vectormap_adapter.hpp"
#include "vec2d.hpp"

namespace common {
namespace math {

struct LaneWaypoint {
  LaneWaypoint() = default;
  // TODO(liujiadong) glog??
  LaneWaypoint(adapter::LaneInfoConstPtr lane, const double s)
      //   : lane(CHECK_NOTNULL(lane)), s(s) {}
      : lane(lane), s(s) {}
  LaneWaypoint(adapter::LaneInfoConstPtr lane, const double s, const double l)
      //     : lane(CHECK_NOTNULL(lane)), s(s), l(l) {}
      : lane(lane), s(s), l(l) {}
  // TODO(liujiadong) define map info
  adapter::LaneInfoConstPtr lane = nullptr;
  double s = 0.0;
  double l = 0.0;

  std::string DebugString() const;
};

struct LaneSegment {
  LaneSegment() = default;
  LaneSegment(adapter::LaneInfoConstPtr lane, const double start_s,
              const double end_s)
      // : lane(CHECK_NOTNULL(lane)), start_s(start_s), end_s(end_s) {}
      : lane(lane), start_s(start_s), end_s(end_s) {}
  adapter::LaneInfoConstPtr lane = nullptr;
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

  static std::vector<MapPathPoint>
  GetPointsFromLane(adapter::LaneInfoConstPtr lane, const double start_s,
                    const double end_s);

  std::string DebugString() const;

protected:
  double heading_ = 0.0;
  std::vector<LaneWaypoint> lane_waypoints_;
};
} // namespace math
} // namespace common