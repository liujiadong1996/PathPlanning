cmake_minimum_required(VERSION 3.10)

# set project name
project(PathPlanning CXX)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)

##########################
# GTest
##########################
add_subdirectory(third_part/googletest)
include_directories(third_part/googletest/googletest/include)
include_directories(third_part/googletest/googlemock/include)

file(GLOB_RECURSE SRCS common/*.cpp unittest/*.cpp)
file(GLOB_RECURSE HDRS *.hpp)

include_directories(${CMAKE_CURRENT_LIST_DIR})

# add the executable
add_executable(PathPlanning ${SRCS})

target_link_libraries(PathPlanning gtest gtest_main)
target_link_libraries(PathPlanning gmock gmock_main)