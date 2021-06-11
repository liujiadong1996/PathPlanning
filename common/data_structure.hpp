#include <string>

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

} // namespace common