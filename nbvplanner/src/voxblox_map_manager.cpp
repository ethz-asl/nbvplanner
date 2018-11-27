#include "nbvplanner/voxblox_map_manager.h"

namespace nbvInspection {

VoxbloxMapManager::VoxbloxMapManager(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), tsdf_server_(nh, nh_private) {
  tsdf_layer_ = tsdf_server_.getTsdfMapPtr()->getTsdfLayerPtr();
  CHECK_NOTNULL(tsdf_layer_);
}

VoxbloxMapManager::VoxelStatus VoxbloxMapManager::getVoxelStatus(
    const Eigen::Vector3d& position) const {
  voxblox::TsdfVoxel* voxel = tsdf_layer_->getVoxelPtrByCoordinates(
      position.cast<voxblox::FloatingPoint>());

  if (voxel == nullptr) {
    return VoxelStatus::kUnknown;
  }
  if (voxel->weight < 1e-6) {
    return VoxelStatus::kUnknown;
  }
  if (voxel->distance > 0.0) {
    return VoxelStatus::kFree;
  }
  return VoxelStatus::kOccupied;
}

VoxbloxMapManager::VoxelStatus VoxbloxMapManager::getVisibility(
    const Eigen::Vector3d& view_point, const Eigen::Vector3d& voxel_to_test,
    bool stop_at_unknown_cell) const {
  // This involves doing a raycast from view point to voxel to test.
  // Let's get the global voxel coordinates of both.
  float voxel_size = tsdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  // Cut????
  voxblox::LongIndex start_voxel_idx =
      voxblox::getGridIndexFromPoint<voxblox::LongIndex>(
          view_point.cast<voxblox::FloatingPoint>(), voxel_size_inv);
  voxblox::LongIndex end_voxel_idx =
      voxblox::getGridIndexFromPoint<voxblox::LongIndex>(
          voxel_to_test.cast<voxblox::FloatingPoint>(), voxel_size_inv);
  // End cut here.

  const voxblox::Point start_scaled =
      view_point.cast<voxblox::FloatingPoint>() * voxel_size_inv;
  const voxblox::Point end_scaled =
      voxel_to_test.cast<voxblox::FloatingPoint>() * voxel_size_inv;

  voxblox::LongIndexVector global_voxel_indices;
  voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

  // Iterate over the ray.
  for (const voxblox::GlobalIndex& global_index : global_voxel_indices) {
    voxblox::TsdfVoxel* voxel =
        tsdf_layer_->getVoxelPtrByGlobalIndex(global_index);
    if (voxel == nullptr || voxel->weight < 1e-6) {
      if (stop_at_unknown_cell) {
        return VoxelStatus::kUnknown;
      }
    } else if (voxel->distance <= 0.0) {
      return VoxelStatus::kOccupied;
    }
  }
  return VoxelStatus::kFree;
}

}  // namespace nbvInspection
