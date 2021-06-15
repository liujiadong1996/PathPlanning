#include <vector>
#include "common/math/euler_angles_zxy.hpp"
#include "planning/reference_line/reference_line.hpp"
#include "ros/ros.h"
#include "subscriber/LaneArray.h"

void callbackGetLaneArray(const subscriber::LaneArray &msg) {
  std::cout << "Received Lanes" << msg.id << std::endl;
  std::vector<planning::reference_line::ReferencePoint> ref_points;
  for (auto &lane : msg.lanes) {
    for (auto &waypoint : lane.waypoints) {
      common::math::Vec2d position(waypoint.pose.pose.position.x,
                                   waypoint.pose.pose.position.y);
      common::math::EulerAnglesZXYd euler_angle(
          waypoint.pose.pose.orientation.w, waypoint.pose.pose.orientation.x,
          waypoint.pose.pose.orientation.y, waypoint.pose.pose.orientation.z);
      common::math::MapPathPoint map_path_point(position, euler_angle.yaw());
      planning::reference_line::ReferencePoint ref_point(map_path_point, 0.0,
                                                         0.0);
      ref_points.emplace_back(std::move(ref_point));
    }
  }
  planning::reference_line::ReferenceLine refline(ref_points);
  refline.DebugString();

  // test start and end point
  common::math::Vec2d xy_start(-26731.543, 98878.894);
  common::math::Vec2d xy_end(-26728.555, 98869.351);
  common::SLPoint sl_start;
  common::SLPoint sl_end;
  refline.XYToSL(xy_start, &sl_start);
  refline.XYToSL(xy_end, &sl_end);
  std::cout << "sl_start, s: " << sl_start.s << " , l: " << sl_start.l
            << std::endl;
  std::cout << "sl_end, s: " << sl_end.s << " , l: " << sl_end.l << std::endl;

  // test obstacle box
  common::math::Vec2d box_center(-26740.1, 98892.1);
  double heading = 2.721338;
  double length = 5.0;
  double width = 3.0;
  common::math::Box2d obs_box(box_center, heading, length, width);
  std::vector<common::math::Vec2d> corners;
  obs_box.GetAllCorners(&corners);
  for (const auto &corner : corners) {
    common::SLPoint corner_sl;
    refline.XYToSL(corner, &corner_sl);
    std::cout << "corner, s: " << corner_sl.s << " , l: " << corner_sl.l
              << std::endl;
  }
  common::SLBoundary obs_sl_boundary;
  refline.GetSLBoundary(obs_box, &obs_sl_boundary);
  std::cout << "obs_box, start_s: " << obs_sl_boundary.start_s
            << " , end_s: " << obs_sl_boundary.end_s
            << " , start_l: " << obs_sl_boundary.start_l
            << " , end_l: " << obs_sl_boundary.end_l << std::endl;
  for (const auto &sl : obs_sl_boundary.boundary_point) {
    std::cout << "[s: " << sl.s << ", l: " << sl.l << "]" << std::endl;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub =
      n.subscribe("lane_waypoints_array", 1, callbackGetLaneArray);
  ros::spin();

  return 0;
}