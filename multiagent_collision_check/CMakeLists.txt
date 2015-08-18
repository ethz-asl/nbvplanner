cmake_minimum_required(VERSION 2.8.3)
project(multiagent_collision_check)

add_definitions(-std=c++11)
#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  octomap_world
)

catkin_package(
  INCLUDE_DIRS include
)
add_library(multiagent_collision_check_lib src/multiagent_collision_checker.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

target_link_libraries(multiagent_collision_check_lib
  ${catkin_LIBRARIES}
)
