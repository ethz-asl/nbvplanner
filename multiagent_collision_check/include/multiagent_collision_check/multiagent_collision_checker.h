/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
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
