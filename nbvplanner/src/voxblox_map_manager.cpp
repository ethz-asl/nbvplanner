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
    return VoxbloxMapManager::VoxelStatus::kUnknown;
  }
  if (voxel->weight < 1e-6) {
    return VoxbloxMapManager::VoxelStatus::kUnknown;
  }
  if (voxel->distance > 0.0) {
    return VoxbloxMapManager::VoxelStatus::kFree;
  }
  return VoxbloxMapManager::VoxelStatus::kOccupied;
}

}  // namespace nbvInspection
