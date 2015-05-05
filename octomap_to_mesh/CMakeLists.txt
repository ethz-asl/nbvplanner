cmake_minimum_required(VERSION 2.8.3)
project(octomap_to_mesh)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/OctomapToMesh.cpp
  src/OctomapToGrid.cpp
  src/cat.cpp
  src/circshift.cpp
  src/fclose.cpp
  src/fileManager.cpp
  src/fopen.cpp
  src/fprintf.cpp
  src/ind2sub.cpp
  src/OctomapToMesh_data.cpp
  src/OctomapToMesh_emxAPI.cpp
  src/OctomapToMesh_emxutil.cpp
  src/OctomapToMesh_initialize.cpp
  src/OctomapToMesh_rtwutil.cpp
  src/OctomapToMesh_terminate.cpp
  src/retrieve_meshformat.cpp
  src/rtGetInf.cpp
  src/rtGetNaN.cpp
  src/rt_nonfinite.cpp
  src/sortrows.cpp
  src/sum.cpp
  src/unique.cpp
  src/write_stl.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
)

############
# BINARIES #
############
cs_add_executable(octomap_to_mesh_node
  src/OctomapToMesh_node.cpp
)
target_link_libraries(octomap_to_mesh_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
