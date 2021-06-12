#include "common/math/path.hpp"

#include <iostream>
#include <limits>

#include "common/utils/math_utils.hpp"
#include "line_segment2d.hpp"

namespace common {
namespace math {

const double kSampleDistance = 0.25;

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

bool Path::GetProjection(const common::math::Vec2d &point, double *accumulate_s,
                         double *lateral) const {
  double distance = 0.0;
  return GetProjection(point, accumulate_s, lateral, &distance);
}

bool Path::GetProjection(const Vec2d &point, double *accumulate_s,
                         double *lateral, double *min_distance) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }
  if (use_path_approximation_) {
    return approximation_.GetProjection(*this, point, accumulate_s, lateral,
                                        min_distance);
  }
  if (num_points_ < 2) {
    std::cout << "[ERROR]: num_point_s < 2!!" << std::endl;
    return false;
  }
  *min_distance = std::numeric_limits<double>::infinity();
  int min_index = 0;
  for (int i = 0; i < num_segments_; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < *min_distance) {
      min_index = i;
      *min_distance = distance;
    }
  }
  *min_distance = std::sqrt(*min_distance);
  const auto &nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index == num_segments_ - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

double PathApproximation::compute_max_error(const Path &path, const int s,
                                            const int t) {
  if (s + 1 >= t) {
    return 0.0;
  }
  const auto &points = path.path_points();
  const LineSegment2d segment(points[s], points[t]);
  double max_distance_sqr = 0.0;
  for (int i = s + 1; i < t; ++i) {
    max_distance_sqr =
        std::max(max_distance_sqr, segment.DistanceSquareTo(points[i]));
  }
  return sqrt(max_distance_sqr);
}

bool PathApproximation::is_within_max_error(const Path &path, const int s,
                                            const int t) {
  if (s + 1 >= t) {
    return true;
  }
  const auto &points = path.path_points();
  const LineSegment2d segment(points[s], points[t]);
  for (int i = s + 1; i < t; ++i) {
    if (segment.DistanceSquareTo(points[i]) > max_sqr_error_) {
      return false;
    }
  }
  return true;
}

void PathApproximation::Init(const Path &path) {
  InitDilute(path);
  InitProjections(path);
}

void PathApproximation::InitDilute(const Path &path) {
  const int num_original_points = path.num_points();
  original_ids_.clear();
  int last_idx = 0;
  while (last_idx < num_original_points - 1) {
    original_ids_.push_back(last_idx);
    int next_idx = last_idx + 1;
    int delta = 2;
    for (; last_idx + delta < num_original_points; delta *= 2) {
      if (!is_within_max_error(path, last_idx, last_idx + delta)) {
        break;
      }
      next_idx = last_idx + delta;
    }
    for (; delta > 0; delta /= 2) {
      if (next_idx + delta < num_original_points &&
          is_within_max_error(path, last_idx, next_idx + delta)) {
        next_idx += delta;
      }
    }
    last_idx = next_idx;
  }
  original_ids_.push_back(last_idx);
  num_points_ = static_cast<int>(original_ids_.size());
  if (num_points_ == 0) {
    return;
  }

  segments_.clear();
  segments_.reserve(num_points_ - 1);
  for (int i = 0; i < num_points_ - 1; ++i) {
    segments_.emplace_back(path.path_points()[original_ids_[i]],
                           path.path_points()[original_ids_[i + 1]]);
  }
  max_error_per_segment_.clear();
  max_error_per_segment_.reserve(num_points_ - 1);
  for (int i = 0; i < num_points_ - 1; ++i) {
    max_error_per_segment_.push_back(
        compute_max_error(path, original_ids_[i], original_ids_[i + 1]));
  }
}

void PathApproximation::InitProjections(const Path &path) {
  if (num_points_ == 0) {
    return;
  }
  projections_.clear();
  projections_.reserve(segments_.size() + 1);
  double s = 0.0;
  projections_.push_back(0);
  for (const auto &segment : segments_) {
    s += segment.length();
    projections_.push_back(s);
  }
  const auto &original_points = path.path_points();
  const int num_original_points = static_cast<int>(original_points.size());
  original_projections_.clear();
  original_projections_.reserve(num_original_points);
  for (size_t i = 0; i < projections_.size(); ++i) {
    original_projections_.push_back(projections_[i]);
    if (i + 1 < projections_.size()) {
      const auto &segment = segments_[i];
      for (int idx = original_ids_[i] + 1; idx < original_ids_[i + 1]; ++idx) {
        const double proj = segment.ProjectOntoUnit(original_points[idx]);
        original_projections_.push_back(
            projections_[i] + std::max(0.0, std::min(proj, segment.length())));
      }
    }
  }

  // max_p_to_left[i] = max(p[0], p[1], ... p[i]).
  max_original_projections_to_left_.resize(num_original_points);
  double last_projection = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_original_points; ++i) {
    last_projection = std::max(last_projection, original_projections_[i]);
    max_original_projections_to_left_[i] = last_projection;
  }
  for (int i = 0; i + 1 < num_original_points; ++i) {
    if (max_original_projections_to_left_[i] >
        max_original_projections_to_left_[i + 1] + kMathEpsilon) {
      return;
    }
  }

  // min_p_to_right[i] = min(p[i], p[i + 1], ... p[size - 1]).
  min_original_projections_to_right_.resize(original_projections_.size());
  last_projection = std::numeric_limits<double>::infinity();
  for (int i = num_original_points - 1; i >= 0; --i) {
    last_projection = std::min(last_projection, original_projections_[i]);
    min_original_projections_to_right_[i] = last_projection;
  }
  for (int i = 0; i + 1 < num_original_points; ++i) {
    if (min_original_projections_to_right_[i] >
        min_original_projections_to_right_[i + 1] + kMathEpsilon) {
      return;
    }
  }

  // Sample max_p_to_left by sample_distance.
  max_projection_ = projections_.back();
  num_projection_samples_ =
      static_cast<int>(max_projection_ / kSampleDistance) + 1;
  sampled_max_original_projections_to_left_.clear();
  sampled_max_original_projections_to_left_.reserve(num_projection_samples_);
  double proj = 0.0;
  int last_index = 0;
  for (int i = 0; i < num_projection_samples_; ++i) {
    while (last_index + 1 < num_original_points &&
           max_original_projections_to_left_[last_index + 1] < proj) {
      ++last_index;
    }
    sampled_max_original_projections_to_left_.push_back(last_index);
    proj += kSampleDistance;
  }
  if (sampled_max_original_projections_to_left_.size() !=
      static_cast<size_t>(num_projection_samples_)) {
    return;
  }
}

bool PathApproximation::GetProjection(const Path &path,
                                      const common::math::Vec2d &point,
                                      double *accumulate_s, double *lateral,
                                      double *min_distance) const {
  if (num_points_ == 0) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }
  double min_distance_sqr = std::numeric_limits<double>::infinity();
  int estimate_nearest_segment_idx = -1;
  std::vector<double> distance_sqr_to_segments;
  distance_sqr_to_segments.reserve(segments_.size());
  for (size_t i = 0; i < segments_.size(); ++i) {
    const double distance_sqr = segments_[i].DistanceSquareTo(point);
    distance_sqr_to_segments.push_back(distance_sqr);
    if (distance_sqr < min_distance_sqr) {
      min_distance_sqr = distance_sqr;
      estimate_nearest_segment_idx = static_cast<int>(i);
    }
  }
  if (estimate_nearest_segment_idx < 0) {
    return false;
  }
  const auto &original_segments = path.segments();
  const int num_original_segments = static_cast<int>(original_segments.size());
  const auto &original_accumulated_s = path.accumulated_s();
  double min_distance_sqr_with_error = utils::Sqr(
      sqrt(min_distance_sqr) +
      max_error_per_segment_[estimate_nearest_segment_idx] + max_error_);
  *min_distance = std::numeric_limits<double>::infinity();
  int nearest_segment_idx = -1;
  for (size_t i = 0; i < segments_.size(); ++i) {
    if (distance_sqr_to_segments[i] >= min_distance_sqr_with_error) {
      continue;
    }
    int first_segment_idx = original_ids_[i];
    int last_segment_idx = original_ids_[i + 1] - 1;
    double max_original_projection = std::numeric_limits<double>::infinity();
    if (first_segment_idx < last_segment_idx) {
      const auto &segment = segments_[i];
      const double projection = segment.ProjectOntoUnit(point);
      const double prod_sqr = utils::Sqr(segment.ProductOntoUnit(point));
      if (prod_sqr >= min_distance_sqr_with_error) {
        continue;
      }
      const double scan_distance = sqrt(min_distance_sqr_with_error - prod_sqr);
      const double min_projection = projection - scan_distance;
      max_original_projection = projections_[i] + projection + scan_distance;
      if (min_projection > 0.0) {
        const double limit = projections_[i] + min_projection;
        const int sample_index =
            std::max(0, static_cast<int>(limit / kSampleDistance));
        if (sample_index >= num_projection_samples_) {
          first_segment_idx = last_segment_idx;
        } else {
          first_segment_idx =
              std::max(first_segment_idx,
                       sampled_max_original_projections_to_left_[sample_index]);
          if (first_segment_idx >= last_segment_idx) {
            first_segment_idx = last_segment_idx;
          } else {
            while (first_segment_idx < last_segment_idx &&
                   max_original_projections_to_left_[first_segment_idx + 1] <
                       limit) {
              ++first_segment_idx;
            }
          }
        }
      }
    }
    bool min_distance_updated = false;
    bool is_within_end_point = false;
    for (int idx = first_segment_idx; idx <= last_segment_idx; ++idx) {
      if (min_original_projections_to_right_[idx] > max_original_projection) {
        break;
      }
      const auto &original_segment = original_segments[idx];
      const double x0 = point.x() - original_segment.start().x();
      const double y0 = point.y() - original_segment.start().y();
      const double ux = original_segment.unit_direction().x();
      const double uy = original_segment.unit_direction().y();
      double proj = x0 * ux + y0 * uy;
      double distance = 0.0;
      if (proj < 0.0) {
        if (is_within_end_point) {
          continue;
        }
        is_within_end_point = true;
        distance = hypot(x0, y0);
      } else if (proj <= original_segment.length()) {
        is_within_end_point = true;
        distance = std::abs(x0 * uy - y0 * ux);
      } else {
        is_within_end_point = false;
        if (idx != last_segment_idx) {
          continue;
        }
        distance = original_segment.end().DistanceTo(point);
      }
      if (distance < *min_distance) {
        min_distance_updated = true;
        *min_distance = distance;
        nearest_segment_idx = idx;
      }
    }
    if (min_distance_updated) {
      min_distance_sqr_with_error = utils::Sqr(*min_distance + max_error_);
    }
  }
  if (nearest_segment_idx >= 0) {
    const auto &segment = original_segments[nearest_segment_idx];
    double proj = segment.ProjectOntoUnit(point);
    const double prod = segment.ProductOntoUnit(point);
    if (nearest_segment_idx > 0) {
      proj = std::max(0.0, proj);
    }
    if (nearest_segment_idx + 1 < num_original_segments) {
      proj = std::min(segment.length(), proj);
    }
    *accumulate_s = original_accumulated_s[nearest_segment_idx] + proj;
    if ((nearest_segment_idx == 0 && proj < 0.0) ||
        (nearest_segment_idx + 1 == num_original_segments &&
         proj > segment.length())) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0 ? (*min_distance) : -(*min_distance));
    }
    return true;
  }
  return false;
}

bool PathApproximation::OverlapWith(const Path &path, const Box2d &box,
                                    double width) const {
  if (num_points_ == 0) {
    return false;
  }
  const Vec2d center = box.center();
  const double radius = box.diagonal() / 2.0 + width;
  const double radius_sqr = utils::Sqr(radius);
  const auto &original_segments = path.segments();
  for (size_t i = 0; i < segments_.size(); ++i) {
    const LineSegment2d &segment = segments_[i];
    const double max_error = max_error_per_segment_[i];
    const double radius_sqr_with_error = utils::Sqr(radius + max_error);
    if (segment.DistanceSquareTo(center) > radius_sqr_with_error) {
      continue;
    }
    int first_segment_idx = original_ids_[i];
    int last_segment_idx = original_ids_[i + 1] - 1;
    double max_original_projection = std::numeric_limits<double>::infinity();
    if (first_segment_idx < last_segment_idx) {
      const auto &segment = segments_[i];
      const double projection = segment.ProjectOntoUnit(center);
      const double prod_sqr = utils::Sqr(segment.ProductOntoUnit(center));
      if (prod_sqr >= radius_sqr_with_error) {
        continue;
      }
      const double scan_distance = sqrt(radius_sqr_with_error - prod_sqr);
      const double min_projection = projection - scan_distance;
      max_original_projection = projections_[i] + projection + scan_distance;
      if (min_projection > 0.0) {
        const double limit = projections_[i] + min_projection;
        const int sample_index =
            std::max(0, static_cast<int>(limit / kSampleDistance));
        if (sample_index >= num_projection_samples_) {
          first_segment_idx = last_segment_idx;
        } else {
          first_segment_idx =
              std::max(first_segment_idx,
                       sampled_max_original_projections_to_left_[sample_index]);
          if (first_segment_idx >= last_segment_idx) {
            first_segment_idx = last_segment_idx;
          } else {
            while (first_segment_idx < last_segment_idx &&
                   max_original_projections_to_left_[first_segment_idx + 1] <
                       limit) {
              ++first_segment_idx;
            }
          }
        }
      }
    }
    for (int idx = first_segment_idx; idx <= last_segment_idx; ++idx) {
      if (min_original_projections_to_right_[idx] > max_original_projection) {
        break;
      }
      const auto &original_segment = original_segments[idx];
      if (original_segment.DistanceSquareTo(center) > radius_sqr) {
        continue;
      }
      if (box.DistanceTo(original_segment) <= width) {
        return true;
      }
    }
  }
  return false;
}
} // namespace math
} // namespace common