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
 
#ifndef _MULTIAGENT_COLLISON_CHECKER_CPP_
#define _MULTIAGENT_COLLISON_CHECKER_CPP_

#include <string>
#include <ros/ros.h>
#include <multiagent_collision_check/multiagent_collision_checker.h>

bool multiagent::isInCollision(const Eigen::Vector4d& start, const Eigen::Vector4d& end,
                               const Eigen::Vector3d& boundingBox,
                               const std::vector<std::vector<Eigen::Vector3d>*>& agent_paths)
{
  for (typename std::vector<std::vector<Eigen::Vector3d>*>::const_iterator it = agent_paths.begin();
      it != agent_paths.end(); it++) {
    for (int it_segment = 1; it_segment < (*it)->size(); it_segment++) {
      if (boundingBox.norm()
          > closestDistanceBetweenLines(Eigen::Vector3d(start.x(), start.y(), start.z()),
                                        Eigen::Vector3d(end.x(), end.y(), end.z()),
                                        (**it)[it_segment - 1], (**it)[it_segment])) {
        return true;
      }
    }
  }
  return false;
}

bool multiagent::isInCollision(const Eigen::Vector4d& state, const Eigen::Vector3d& boundingBox,
                               const std::vector<std::vector<Eigen::Vector3d>*>& agent_paths)
{
  for (typename std::vector<std::vector<Eigen::Vector3d>*>::const_iterator it = agent_paths.begin();
      it != agent_paths.end(); it++) {
    for (int it_segment = 1; it_segment < (*it)->size(); it_segment++) {
      if (boundingBox.norm()
          > closestDistanceBetweenLines(Eigen::Vector3d(state.x(), state.y(), state.z()),
                                        Eigen::Vector3d(state.x(), state.y(), state.z()),
                                        (**it)[it_segment - 1], (**it)[it_segment])) {
        return true;
      }
    }
  }
  return false;
}

std::vector<Eigen::Vector4d> multiagent::circumnavigate(
    const Eigen::Vector4d& start, const Eigen::Vector4d& end, const Eigen::Vector3d& boundingBox,
    const volumetric_mapping::OctomapWorld& manager,
    const std::vector<std::vector<Eigen::Vector3d>*>& agent_paths)
{
  std::vector < Eigen::Vector4d > ret;
  ROS_ERROR_THROTTLE(
      1,
      "Function std::vector<Eigen::Vector4d> multiagent::circumnavigate(...) is not implemented and will not return any path");
  // TODO: find collision free path
  return ret;
}

double multiagent::closestDistanceBetweenLines(const Eigen::Vector3d& start1,
                                               const Eigen::Vector3d& end1,
                                               const Eigen::Vector3d& start2,
                                               const Eigen::Vector3d& end2)
{
  Eigen::Vector3d segment1 = end1 - start1;
  Eigen::Vector3d segment2 = end2 - start2;
  Eigen::Vector3d segment1normalized = segment1.normalized();
  Eigen::Vector3d segment2normalized = segment2.normalized();
  Eigen::Vector3d cross = segment1normalized.cross(segment2normalized);
  double denominator = pow(cross.norm(), 2.0);
  if (denominator != 0) {
    // Lines are not parallel
    Eigen::Vector3d t = (start2 - start1);
    double numerator1 = (t.cross(segment2normalized)).dot(cross);
    double numerator2 = (t.cross(segment1normalized)).dot(cross);
    double t1 = numerator1 / denominator;
    double t2 = numerator2 / denominator;
    Eigen::Vector3d solution1 = start1 + (segment1normalized * t1);
    Eigen::Vector3d solution2 = start2 + (segment2normalized * t2);
    // Clamp results to line segments if necessary
    if (t1 < 0) {
      solution1 = start1;
    } else if (t1 > segment1.norm()) {
      solution1 = end1;
    }
    if (t2 < 0) {
      solution2 = start2;
    } else if (t2 > segment2.norm()) {
      solution2 = end2;
    }
    return (solution1 - solution2).norm();
  }
  // Parallel lines
  double d0 = segment1normalized.dot(start2 - start1);
  double d = (((d0 * segment1normalized) + start1) - start2).norm();
  // Overlapping lines?
  double d1 = segment1normalized.dot(end2 - start1);
  if (d0 <= 0 && 0 >= d1) {
    // segment2 before segment1
    if (fabs(d0) < fabs(d1)) {
      return (start2 - start1).norm();
    }
    return (end2 - start1).norm();
  } else if (d0 >= segment1.norm() && segment1.norm() <= d1) {
    // segment2 after segment1
    if (fabs(d0) < fabs(d1)) {
      return (start2 - end1).norm();
    }
    return (end2, end1).norm();
  }
  return d;
}

#endif // _MULTIAGENT_COLLISON_CHECKER_CPP_
