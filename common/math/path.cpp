#include "common/math/path.hpp"
#include "line_segment2d.hpp"

namespace common {
namespace math {

std::string LaneWaypoint::DebugString() const {
  if (lane == nullptr) {
    return "(lane is null)";
  }
  return "id = " + lane->id() + " s = " + std::to_string(s);
}

std::string LaneSegment::DebugString() const {
  if (lane == nullptr) {
    return "(lane is null)";
  }
  return "id = " + lane->id() + " start_s = " + std::to_string(start_s) +
             " end_s = ",
         std::to_string(end_s);
}

std::vector<MapPathPoint>
MapPathPoint::GetPointsFromSegment(const LaneSegment &segment) {
  return GetPointsFromLane(segment.lane, segment.start_s, segment.end_s);
}

std::vector<MapPathPoint>
MapPathPoint::GetPointsFromLane(map::LaneInfoConstPtr lane,
                                const double start_s, const double end_s) {
  std::vector<MapPathPoint> points;
  if (start_s >= end_s) {
    return points;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points.emplace_back(lane->points()[i], lane->headings()[i],
                          LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points.emplace_back(segment.start() + segment.unit_direction() *
                                                  (start_s - accumulate_s),
                            lane->headings()[i], LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points.emplace_back(segment.start() + segment.unit_direction() *
                                                  (end_s - accumulate_s),
                            lane->headings()[i], LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
  return points;
}

std::string MapPathPoint::DebugString() const {
  // TODO(liujiadong)
  return "";
}
} // namespace math
} // namespace common