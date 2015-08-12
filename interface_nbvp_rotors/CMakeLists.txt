cmake_minimum_required(VERSION 2.8.3)
project(interface_nbvp_rotors)

find_package(catkin REQUIRED COMPONENTS
  trajectory_msgs
  mav_msgs
  tf
)

include_directories(${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS trajectory_msgs mav_msgs tf
)

add_executable(exploration src/exploration.cpp)
target_link_libraries(exploration
  ${catkin_LIBRARIES}
)
add_dependencies(exploration ${catkin_EXPORTED_TARGETS})
