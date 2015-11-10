/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include <thread>
#include <chrono>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nbvplanner/nbvp_srv.h>

using namespace std;

ros::Publisher sample_pub_;

bool PlanExplorationPath(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Starting planning");
  ros::NodeHandle nh("");
  // Initialize
  double dt;
  int n_seq = 0;
  nh.param("nbvp/dt", dt, 0.01);
  ros::Time t_past_iteration;
  std::vector<double> execution;
  std::vector<double> computation;

  trajectory_msgs::MultiDOFJointTrajectory samples_array;

  time_t rawtime;
  struct tm * ptm;
  time(&rawtime);
  ptm = gmtime(&rawtime);
  std::string filename_prefix = std::to_string(ptm->tm_year + 1900) + "_"
      + std::to_string(ptm->tm_mon + 1) + "_" + std::to_string(ptm->tm_mday) + "_"
      + std::to_string(ptm->tm_hour) + "_" + std::to_string(ptm->tm_min) + "_"
      + std::to_string(ptm->tm_sec) + "_";
  t_past_iteration = ros::Time::now();
  std::string pkgPath = ros::package::getPath("nbvplanner");
  fstream file;
  file.open((pkgPath + "/data/path" + filename_prefix + ".m").c_str(), std::ios::out);
  if (!file.is_open()) {
    ROS_WARN("could not open path file");
  }
  file << "pathMatrix = [";
  ros::Time start = ros::Time::now();

  // Run the planner
  bool exploring = true;
  while (exploring && ros::ok()) {
    ros::Time now = ros::Time::now();

    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();
    n_seq++;
    double dt_current = now.toSec() - t_past_iteration.toSec();
    nbvplanner::nbvp_srv planSrv;
    planSrv.request.header.stamp = ros::Time::now();
    planSrv.request.header.frame_id = "world";
    execution.push_back((planSrv.request.header.stamp - start).toSec());
    start = ros::Time::now();
    if (ros::service::call("nbvplanner", planSrv)) {
      computation.push_back((ros::Time::now() - start).toSec());
      start = ros::Time::now();
    } else {
      ROS_WARN_THROTTLE(1, "NBV Planner is not reachable");
      exploring = false;
    }
    for (int index = 0; index < planSrv.response.path.size(); index++) {
      tf::Vector3 point = tf::Vector3(planSrv.response.path[index].position.x,
                                      planSrv.response.path[index].position.y,
                                      planSrv.response.path[index].position.z);
      mav_msgs::EigenTrajectoryPoint trajectory_point;
      trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
      trajectory_point.position_W.x() = point.x();
      trajectory_point.position_W.y() = point.y();
      trajectory_point.position_W.z() = point.z();
      tf::Pose pose;
      tf::poseMsgToTF(planSrv.response.path[index], pose);
      tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0),
                                           tf::getYaw(pose.getRotation()));
      trajectory_point.setFromYaw(tf::getYaw(quat));

      file << point.x() << ", " << point.y() << ", " << point.z() << ", " << tf::getYaw(quat)
          << ";\n";

      mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
      samples_array.points.push_back(trajectory_point_msg);
    }
    sample_pub_.publish(samples_array);
    double sleeping = dt * (double) samples_array.points.size();
    ros::Duration(sleeping).sleep();
  }

  // Clean up
  file << "];\nexecutionTime = [";
  for (int i = 0; i < execution.size(); i++) {
    file << execution[i] << ", ";
  }
  file << "];\ncomputationTime = [";
  for (int i = 0; i < computation.size(); i++) {
    file << computation[i] << ", ";
  }
  file << "];";
  file.close();
  ROS_WARN("Exploration executed!");

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interface_nbvp");
  ros::NodeHandle nh;

  sample_pub_ = nh.advertise < trajectory_msgs::MultiDOFJointTrajectory
      > (mav_msgs::default_topics::COMMAND_TRAJECTORY, 5);
  ros::ServiceServer start_planning_srv_ = nh.advertiseService("start_planning",
                                                               PlanExplorationPath);

  ROS_INFO("Planner ready");
  ros::spin();
}
