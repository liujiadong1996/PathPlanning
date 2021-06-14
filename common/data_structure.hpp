#pragma once

#include <string>
#include <vector>

namespace common {

struct SLPoint {
  double s = 0.0;
  double l = 0.0;
};

struct FrenetFramePoint {
  double s = 0.0;
  double l = 0.0;
  double dl = 0.0;
  double ddl = 0.0;
};

struct SpeedPoint {
  double s = 0.0;
  double t = 0.0;
  // speed (m / s)
  double v = 0.0;
  // acceleration (m / s^2)
  double a = 0.0;
  // jerk (m / s^3)
  double da = 0.0;
};

struct PathPoint {
  // coordinates
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  // direction on the x-y plane
  double theta = 0.0;
  // curvature on the x-y plane
  double kappa = 0.0;
  // accumulated distance from beginning of the path
  double s = 0.0;

  // derivative of kappa w.r.t s
  double dkappa = 0.0;
  // derivative of derivative of kappa w.r.t s
  double ddkappa = 0.0;
  // The lane ID whter the path point is on
  std::string lane_id;

  // derivative of x and y w.r.t parametric parameter t in CosThetaReferenceLine
  double x_derivative = 0.0;
  double y_derivative = 0.0;
};

struct Path {
  std::string name;
  std::vector<PathPoint> path_point;
};

struct TrajectoryPoint {
  // path point
  PathPoint path_point;
  // linear velocity
  double v = 0.0; // [m /s]
  // line acceleration
  double a = 0.0;
  // relative time from beginning of the trajectory
  double relative_time = 0.0;
  // longitudinal jerk
  double da = 0.0;
  // The angle between vehicle front wheel and vehicle longitudinal axis
  double steer = 0.0;
};

struct Trajectory {
  std::string name;
  std::vector<TrajectoryPoint> trajectory_point;
};

struct PointENU {
  double x = 0.0; // East from the origin, in meters
  double y = 0.0; // North from the origin, in meters
  double z = 0.0; // Up from the ?? , in meters
};

struct SLBoundary {
  double start_s = 0.0;
  double end_s = 0.0;
  double start_l = 0.0;
  double end_l = 0.0;
  std::vector<SLPoint> boundary_point;
};

struct Polygon {
  std::vector<PointENU> point;
};

struct LineSegment {
  std::vector<PointENU> point;
};

struct CurveSegment {
  LineSegment line_segment;
  double s = 0.0;
  PointENU start_position;
  double heading = 0.0;
  double length = 0.0;
};

struct Curve {
  std::vector<CurveSegment> segment;
};

struct BoundaryEdge {
  Curve cureve;
  enum Type { UNKNOWN = 0, NORMAL = 1, LEFT_BOUNDARY = 2, RIGHT_BOUNDARY = 3 };
  Type type;
};

struct BoundaryPolygon {
  std::vector<BoundaryEdge> edge;
};

struct RoadBoundary {
  BoundaryPolygon outer_polygon;
  std::vector<BoundaryPolygon> hole;
};

struct RoadROIBoundary {
  std::string id;
  std::vector<RoadBoundary> road_boundaries;
};

struct RoadSection {
  std::string id;
  std::vector<std::string> lane_id;
  RoadBoundary boundary;
};

struct Road {
  std::string id;
  std::vector<RoadSection> section;
  std::string junction_id;

  enum Type { UNKNOWN = 0, HIGHWAY = 1, CITY_ROAD = 2, PARK = 3 };
  Type type;
};

} // namespace common