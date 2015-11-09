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

#ifndef _MESH_STRUCTURE_H_
#define _MESH_STRUCTURE_H_

#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <octomap_world/octomap_manager.h>

namespace mesh {

class StlMesh
{
 public:
  StlMesh();
  StlMesh(std::fstream& file);
  StlMesh(const Eigen::Vector3d x1, const Eigen::Vector3d x2, const Eigen::Vector3d x3);
  ~StlMesh();
  static void setCameraParams(std::vector<double> cameraPitch,
                              std::vector<double> cameraHorizontalFoV,
                              std::vector<double> cameraVerticalFoV, double maxDist);
  static void setPeerPose(const geometry_msgs::Pose& pose, int n_peer);
  static void setResolution(double resolution)
  {
    resolution_ = resolution;
  }
  static void setOctomapManager(volumetric_mapping::OctomapManager * manager)
  {
    manager_ = manager;
  }
  void incorporateViewFromPoseMsg(const geometry_msgs::Pose& pose, int n_peer);
  double computeInspectableArea(const tf::Transform& transform);
  void assembleMarkerArray(visualization_msgs::Marker& inspected,
                           visualization_msgs::Marker& uninspected) const;

 private:
  void incorporateViewFromTf(const tf::Transform& transform, const std::vector<bool>& unobstructed);
  void split();
  bool collapse();
  bool getVisibility(const tf::Transform& transform, bool& partialVisibility,
                     bool stop_at_unknown_cell, const std::vector<bool>& unobstructed = { }) const;

  bool isLeaf_;
  bool isHead_;
  bool isInspected_;
  std::vector<StlMesh*> children_;
  Eigen::Vector3d x1_;
  Eigen::Vector3d x2_;
  Eigen::Vector3d x3_;
  Eigen::Vector3d normal_;

  static double resolution_;
  static std::vector<double> cameraPitch_;
  static std::vector<double> cameraHorizontalFoV_;
  static std::vector<double> cameraVerticalFoV_;
  static double maxDist_;
  static std::vector<std::vector<tf::Vector3> > camBoundNormals_;
  static volumetric_mapping::OctomapManager * manager_;
  static std::vector<tf::Vector3> peer_vehicles_;
};
}

#endif // _MESH_STRUCTURE_H_
