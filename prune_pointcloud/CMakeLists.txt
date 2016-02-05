cmake_minimum_required(VERSION 2.8.3)
project(prune_pointcloud)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf
  sensor_msgs
  pcl_conversions
  pcl_ros
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
)
add_executable(prune_node src/prune_node.cc src/prune.cc)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

target_link_libraries(prune_node
  ${catkin_LIBRARIES}
)
