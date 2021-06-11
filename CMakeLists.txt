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

file(GLOB_RECURSE SRCS common/*.cpp adapter/*.cpp)
file(GLOB_RECURSE HDRS *.hpp)

include_directories(${CMAKE_CURRENT_LIST_DIR})

# add the executable
# add_executable(unittest_cart_frenet_conversion ${SRCS})
# target_link_libraries(unittest_cart_frenet_conversion gtest gtest_main)
# target_link_libraries(unittest_cart_frenet_conversion gmock gmock_main)

file(GLOB_RECURSE EXEC_SRCS unittest/*.cpp)
foreach(execfilename ${EXEC_SRCS})
    get_filename_component(filename ${execfilename} NAME_WE)
    add_executable(${filename} ${SRCS} ${execfilename})
    target_link_libraries(${filename} gtest gtest_main)
    target_link_libraries(${filename} gmock gmock_main)
    
endforeach(execfilename ${EXEC_SRCS})
