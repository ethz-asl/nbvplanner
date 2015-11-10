cmake_minimum_required(VERSION 2.8.3)
project(interface_nbvp_neo)

find_package(catkin REQUIRED COMPONENTS
  trajectory_msgs
  mav_msgs
  tf
  roslib
)

include_directories(${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS trajectory_msgs mav_msgs tf roslib
)

add_executable(interface src/interface.cpp)
target_link_libraries(interface
  ${catkin_LIBRARIES}
)
add_dependencies(interface ${catkin_EXPORTED_TARGETS})
