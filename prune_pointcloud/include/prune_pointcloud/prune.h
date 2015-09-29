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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

namespace PointcloudPruning {

class Prune
{
 public:
  Prune(ros::NodeHandle& n);
  ~Prune();
  void pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pointcloudIn);
 private:
  ros::NodeHandle n_;
  ros::Publisher pcl_publisher_;
  ros::Subscriber pointcloudSub_;
  tf::TransformListener tf_listener_;
  void loadParams();
  std::vector<std::string> vehicle_tf_frames_;
  double maxDist2_;
};

}
