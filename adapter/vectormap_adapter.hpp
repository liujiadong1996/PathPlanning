#pragma once

#include <memory>
#include <string>

#include "common/math/line_segment2d.hpp"
#include "common/math/vec2d.hpp"

namespace adapter {

class LaneInfo;

using Id = std::string;
using LaneInfoConstPtr = std::shared_ptr<const LaneInfo>;

struct Lane {
  Id id;
};

class LaneInfo {
public:
  const Id &id() const { return lane_.id; }
  const std::vector<common::math::Vec2d> &points() const { return points_; }
  const std::vector<double> &headings() const { return headings_; }
  const std::vector<common::math::LineSegment2d> &segments() const {
    return segments_;
  }

private:
  const Lane &lane_;
  std::vector<common::math::Vec2d> points_;
  std::vector<double> headings_;
  std::vector<common::math::LineSegment2d> segments_;
};
} // namespace adapter