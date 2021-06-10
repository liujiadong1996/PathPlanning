#include "common/math/path.hpp"

namespace common {
namespace math {

std::string LaneWaypoint::DebugString() const {
  if (lane == nullptr) {
    return "(lane is null)";
  }
  return "id = " + lane.Id() + " s = " + std::to_string(s);
}

std::string LaneSegment::DebugString() const {
  if (lane == nullptr) {
    return "(lane is null)";
  }
  return "id = " + lane.Id() + " start_s = " + std::to_string(start_s) +
             " end_s = ",
         std::to_string(end_s);
}

std::vector<MapPathPoint>
MapPathPoint::GetPointsFromSegment(const LaneSegment &segment) {
  return GetPointsFromLane(segment.lane, segment.start_s, segment.end_s);
}

std::vector<MapPathPoint>
MapPathPoint::GetPointsFromLane(adapter::LaneInfoConstPtr lane,
                                const double start_s, const double end_s) {
  std::vector<MapPathPoint> points;
  if (start_s >= end_s) {
    return points;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points.emplace_back(lane->points()[i], lane->heading()[i],
                          LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      // TODO(liujiadong)
    }
  }
}
} // namespace math
} // namespace common