cmake_minimum_required(VERSION 2.8.3)
project(nbvplanner)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
  octomap_world
  tf
  kdtree
  multiagent_collision_check
  voxblox_ros
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)

add_service_files(
  FILES
  nbvp_srv.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES nbvplanner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs octomap_world tf kdtree voxblox_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(nbvPlannerLib src/mesh_structure.cpp src/nbvp.cpp src/rrt.cpp src/tree.cpp src/voxblox_map_manager.cpp)
add_executable(nbvPlanner src/nbv_planner_node.cpp src/mesh_structure.cpp src/nbvp.cpp src/rrt.cpp src/tree.cpp src/voxblox_map_manager.cpp)
add_dependencies(nbvPlannerLib ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(nbvPlannerLib
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
target_link_libraries(nbvPlanner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
