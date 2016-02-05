cmake_minimum_required(VERSION 2.8.3)
project(multiagent_collision_check)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  octomap_world
  message_generation
)

add_message_files(
  FILES
  Segment.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multiagent_collision_check_lib ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
)
add_library(multiagent_collision_check_lib src/multiagent_collision_checker.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

target_link_libraries(multiagent_collision_check_lib
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
