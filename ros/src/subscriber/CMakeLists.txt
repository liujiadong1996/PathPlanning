cmake_minimum_required(VERSION 3.0.2)
project(subscriber)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  jsk_recognition_msgs
  sensor_msgs
)

add_message_files(
  FILES
  DTLane.msg
  Lane.msg
  LaneArray.msg
  Waypoint.msg
  WaypointState.msg
)

generate_messages(DEPENDENCIES 
  geometry_msgs
  jsk_recognition_msgs
  sensor_msgs
  std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  /home/liujiadong/ws/PathPlanning/
)

file(GLOB_RECURSE SRCS 
  /home/liujiadong/ws/PathPlanning/common/*.cpp 
  /home/liujiadong/ws/PathPlanning/planning/*.cpp)

add_executable(subscriber src/subscriber.cpp ${SRCS})
target_link_libraries(subscriber ${catkin_LIBRARIES})
# add_dependencies(subscriber generate_message_cpp)