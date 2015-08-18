#ifndef _MULTIAGENT_COLLISON_CHECKER_H_
#define _MULTIAGENT_COLLISON_CHECKER_H_

#include <vector>
#include <octomap_world/octomap_world.h>

namespace multiagent {

bool isInCollision(const Eigen::Vector4d& start, const Eigen::Vector4d& end,
                   const Eigen::Vector3d& boundingBox,
                   const std::vector<std::vector<Eigen::Vector3d>*>& agent_paths);

bool isInCollision(const Eigen::Vector4d& state, const Eigen::Vector3d& boundingBox,
                   const std::vector<std::vector<Eigen::Vector3d>*>& agent_paths);

std::vector<Eigen::Vector4d> circumnavigate(
    const Eigen::Vector4d& start, const Eigen::Vector4d& end, const Eigen::Vector3d& boundingBox,
    const volumetric_mapping::OctomapWorld& manager,
    const std::vector<std::vector<Eigen::Vector3d>*>& agent_paths);

double closestDistanceBetweenLines(const Eigen::Vector3d& start1, const Eigen::Vector3d& end1,
                                   const Eigen::Vector3d& start2, const Eigen::Vector3d& end2);
}

#endif // _MULTIAGENT_COLLISON_CHECKER_H_
