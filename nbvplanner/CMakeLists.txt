cmake_minimum_required(VERSION 2.8.3)
project(nbvplanner)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  geometry_msgs
  message_generation
  sensor_msgs
  rotors_gazebo
  octomap_to_mesh
  octomap_world
  tf
  kdtree
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

add_service_files(
  FILES
  nbvp_srv.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES nbvplanner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS message_runtime octomap_to_mesh
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(nbvPlanner src/nbv_planner_node.cpp src/mesh_structure.cpp src/nbvp.cpp src/rrt.cpp src/tree.cpp)
add_executable(exploration src/exploration.cpp)
add_executable(wpFollower src/wpFollower.cpp)

target_link_libraries(nbvPlanner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
target_link_libraries(exploration
  ${catkin_LIBRARIES}
)
target_link_libraries(wpFollower
  ${catkin_LIBRARIES}
)
add_dependencies(exploration ${catkin_EXPORTED_TARGETS})
add_dependencies(wpFollower ${catkin_EXPORTED_TARGETS})
