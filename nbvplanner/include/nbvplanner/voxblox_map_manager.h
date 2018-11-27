#ifndef NBVPLANNER_VOXBLOX_MAP_MANAGER_H_
#define NBVPLANNER_VOXBLOX_MAP_MANAGER_H_

#include <ros/ros.h>
#include <voxblox_ros/tsdf_server.h>

namespace nbvInspection {

// A small class that uses a voxblox TSDF server underneath to emulate the
// functionality of Octomap.
// NOTE: for now uses only *T*SDFs, so doesn't compute the full ESDFs and
// doesn't take advantage of faster collision checking!
class VoxbloxMapManager {
 public:
  enum VoxelStatus { kUnknown = 0, kOccupied, kFree };

  VoxbloxMapManager(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);

  VoxelStatus getVisibility(const Eigen::Vector3d& view_point,
                            const Eigen::Vector3d& voxel_to_test,
                            bool stop_at_unknown_cell) const;

  VoxelStatus getVoxelStatus(const Eigen::Vector3d& position) const;

  // Project an axis-aligned bounding box along a line.
  VoxelStatus getLineStatusBoundingBox(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& bounding_box_size) const;

  // Get the voxel status of an axis-aligned bounding box centered at the given
  // position.
  VoxelStatus getBoundingBoxStatus(
      const Eigen::Vector3d& center,
      const Eigen::Vector3d& bounding_box_size) const;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  voxblox::TsdfServer tsdf_server_;

  // Cached:
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;
};

}  // namespace nbvInspection

#endif  // NBVPLANNER_VOXBLOX_MAP_MANAGER_H_
