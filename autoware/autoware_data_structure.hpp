#pragma

#include <sstream>
#include <string>
#include <vector>

namespace autoware {

enum DIRECTION_TYPE {
  FORWARD_DIR,
  FORWARD_LEFT_DIR,
  FORWARD_RIGHT_DIR,
  BACKWARD_DIR,
  BACKWARD_LEFT_DIR,
  BACKWARD_RIGHT_DIR,
  STANDSTILL_DIR
};

enum STATE_TYPE {
  INITIAL_STATE,
  WAITING_STATE,
  FORWARD_STATE,
  STOPPING_STATE,
  EMERGENCY_STATE,
  TRAFFIC_LIGHT_STOP_STATE,
  TRAFFIC_LIGHT_WAIT_STATE,
  STOP_SIGN_STOP_STATE,
  STOP_SIGN_WAIT_STATE,
  FOLLOW_STATE,
  LANE_CHANGE_STATE,
  OBSTACLE_AVOIDANCE_STATE,
  GOAL_STATE,
  FINISH_STATE,
  YIELDING_STATE,
  BRANCH_LEFT_STATE,
  BRANCH_RIGHT_STATE
};

enum ACTION_TYPE {
  FORWARD_ACTION,
  BACKWARD_ACTION,
  STOP_ACTION,
  LEFT_TURN_ACTION,
  RIGHT_TURN_ACTION,
  U_TURN_ACTION,
  SWERVE_ACTION,
  OVERTACK_ACTION,
  START_ACTION,
  SLOWDOWN_ACTION,
  CHANGE_DESTINATION,
  WAITING_ACTION,
  DESTINATION_REACHED,
  UNKOWN_ACTION
};

enum BEH_STATE_TYPE {
  BEH_FORWARD_STATE = 0,
  BEH_STOPPING_STATE = 1,
  BEH_BRANCH_LEFT_STATE = 2,
  BEH_BRANCH_RIGHT_STATE = 3,
  BEH_YIELDING_STATE = 4,
  BEH_ACCELERATING_STATE = 5,
  BEH_SLOWDOWN_STATE = 6
};

enum SEGMENT_TYPE {
  NORMAL_ROAD_SEG,
  INTERSECTION_ROAD_SEG,
  UTURN_ROAD_SEG,
  EXIT_ROAD_SEG,
  MERGE_ROAD_SEG,
  HIGHWAY_ROAD_SEG
};

enum MARKING_TYPE {
  UNKNOWN_MARK,
  TEXT_MARK,
  AF_MARK,
  AL_MARK,
  AR_MARK,
  AFL_MARK,
  AFR_MARK,
  ALR_MARK,
  UTURN_MARK,
  NOUTURN_MARK
};

enum TrafficSignTypes {
  UNKNOWN_SIGN,
  STOP_SIGN,
  MAX_SPEED_SIGN,
  MIN_SPEED_SIGN
};

struct GPSPoint {
  double lat, x;
  double lon, y;
  double alt, z;
  double dir, a;

  GPSPoint() {
    lat = x = 0.0;
    lon = y = 0.0;
    alt = z = 0.0;
    dir = a = 0.0;
  }

  GPSPoint(const double &x, const double &y, const double &z, const double &a) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->a = a;

    lat = 0.0;
    lon = 0.0;
    alt = 0.0;
    dir = 0.0;
  }

  std::string ToString() {
    std::stringstream str;
    str.precision(12);
    str << "X: " << x << ", Y: " << y << ", Z: " << z << ", A: " << a
        << std::endl;
    str << "Lon: " << lon << ", Lat: " << lat << ", Alt: " << alt
        << ", Dir: " << dir << std::endl;
    return str.str();
  }
};

struct Rotation {
  double x;
  double y;
  double z;
  double w;

  Rotation() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    w = 0.0;
  }
};

struct WayPoint {
  GPSPoint pos;
  Rotation rot;
  double v;
  double cost;
  double time_cost;
  double total_reward;
  double collision_cost;
  double lane_change_cost;
  int lane_id;
  int id;
  int left_point_id;
  int right_point_id;
  int left_lane_id;
  int right_lane_id;
  int stop_line_id;
  DIRECTION_TYPE direction;
  STATE_TYPE state;
  BEH_STATE_TYPE beh_state;
  int original_index;

  Lane *lane_ptr;
  WayPoint *left_point_ptr;
  WayPoint *right_point_ptr;
  std::vector<int> to_ids;
  std::vector<int> from_ids;
  std::vector<WayPoint *> front_points_ptr;
  std::vector<WayPoint *> back_points_ptr;
  std::vector<std::pair<ACTION_TYPE, double>> action_cost;
};

struct Boundary {
  int id;
  int road_id;
  std::vector<GPSPoint> points;
  RoadSegment *segment_ptr;

  Boundary() {
    id = 0;
    road_id = 0;
    segment_ptr = nullptr;
  }
};

struct Curb {
  int id;
  int lane_id;
  int road_id;
  std::vector<GPSPoint> points;
  Lane *lane_ptr;
};

struct Crossing {
  int id;
  int road_id;
  std::vector<GPSPoint> points;
  RoadSegment *segment_ptr;

  Crossing() {
    id = 0;
    road_id = 0;
    segment_ptr = nullptr;
  }
};

struct StopLine {
  int id;
  int lane_id;
  int road_id;
  int traffic_light_id;
  int stop_sign_id;
  std::vector<GPSPoint> points;
  Lane *lane_ptr;
  int link_id;

  StopLine() {
    id = 0;
    lane_id = 0;
    road_id = 0;
    lane_ptr = nullptr;
    traffic_light_id = -1;
    stop_sign_id = -1;
    link_id = 0;
  }
};

struct RoadSegment {
  int id;

  SEGMENT_TYPE road_type;
  Boundary boundary;
};

struct WaitingLine {
  int id;
  int lane_id;
  int road_id;
  std::vector<GPSPoint> points;
  Lane *lane_ptr;

  WaitingLine() {
    id = 0;
    lane_id = 0;
    road_id = 0;
    lane_ptr = nullptr;
  }
};

struct TrafficSign {
  int id;
  int lane_id;
  int road_id;

  GPSPoint pos;
  TrafficSignTypes sign_type;
  double value;
  double from_value;
  double to_value;
  std::string str_value;
  timespec time_value;
  timespec from_time_value;
  timespec to_time_value;

  Lane *lane_ptr;

  TrafficSign() {
    id = 0;
    lane_id = 0;
    road_id = 0;
    sign_type = UNKNOWN_SIGN;
    value = 0.0;
    from_value = 0.0;
    to_value = 0.0;
    lane_ptr = nullptr;
  }
};

enum LaneType {
  NORMAL_LANE,
  MERGE_LANE,
  EXIT_LANE,
  BUS_LANE,
  BUS_STOP_LANE,
  EMERGENCY_LANE
};

struct Lane {
  int id;
  int road_id;
  int area_id;
  int from_area_id;
  int to_area_id;
  std::vector<int> from_ids;
  std::vector<int> to_ids;
  int num;  // lane number in the road segment from left to right
  double speed;
  double length;
  double dir;
  LaneType type;
  double width;
  std::vector<WayPoint> points;
  std::vector<TrafficLight> traffic_lights;
  std::vector<StopLine> stop_lines;
  WaitingLine waitingLine;

  std::vector<Lane *> from_lanes;
  std::vector<Lane *> to_lanes;
  Lane *left_lane_ptr;
  Lane *right_lane_ptr;

  RoadSegment *road_ptr;

  Lane() {
    id = 0;
    num = 0;
    speed = 0.0;
    length = 0.0;
    dir = 0.0;
    type = NORMAL_LANE;
    width = 0.0;
    left_lane_ptr = nullptr;
    right_lane_ptr = nullptr;
    road_ptr = nullptr;
    road_id = 0;
    area_id = 0;
    from_area_id = 0;
    to_area_id = 0;
  }
};

enum TrafficLightState {
  UNKNOWN_LIGHT,
  RED_LIGHT,
  GREEN_LIGHT,
  YELLOW_LIGHT,
  LEFT_GREEN,
  FORWARD_GREEN,
  RIGHT_GREEN,
  FLASH_YELLOW,
  FLASH_RED
};

struct TrafficLight {
  int id;
  GPSPoint pos;
  TrafficLightState light_state;
  double stopping_distance;
  std::vector<int> lane_ids;
  std::vector<Lane *> lanes_ptr;
  int link_id;

  TrafficLight() {
    stopping_distance = 2.0;
    id = 0;
    light_state = GREEN_LIGHT;
    link_id = 0;
  }

  bool CheckLane(const int id) {
    for (const int lane_id : lane_ids) {
      if (id == lane_id) {
        return true;
      }
    }
    return false;
  }
};

struct Marking {
  int id;
  int lane_id;
  int road_id;
  MARKING_TYPE mark_type;
  GPSPoint center;
  std::vector<GPSPoint> points;
  Lane *lane_ptr;

  Marking() {
    id = 0;
    lane_id = 0;
    road_id = 0;
    mark_type = UNKNOWN_MARK;
    lane_ptr = nullptr;
  }
};

struct RoadNetwork {
  std::vector<RoadSegment> road_segments;
  std::vector<TrafficLight> traffic_lights;
  std::vector<StopLine> stop_lines;
  std::vector<Curb> curbs;
  std::vector<Boundary> boundaries;
  std::vector<Crossing> crossings;
  std::vector<Marking> markings;
  std::vector<TrafficSign> signs;
};

}  // namespace autoware