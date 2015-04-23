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
#include <eigen3/Eigen/Dense>
#include <std_srvs/Empty.h>
#include <mav_msgs/CommandTrajectoryPositionYaw.h>
#include "nbvplanner/nbvp_srv.h"
#include "tf/tf.h"

/* record:
rosbag record /clock /firefly/vi_sensor/cera_left/image_raw /firefly/ground_truth/pose
*/

int main(int argc, char** argv){
  ros::init(argc, argv, "wpFollower");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub = nh.advertise<mav_msgs::CommandTrajectoryPositionYaw>("command/trajectory_position_yaw", 10);
  ros::ServiceClient pathPlanner = nh.serviceClient<nbvplanner::nbvp_srv>("pathplanning/nbvplanner",10);
  ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }
  
  double dt = 1.0;
  std::string ns = ros::this_node::getName();
  if(!ros::param::get(ns+"/dt", dt))
  {
    ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", (ns+"/nbvp/dt").c_str());
    return -1;
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  mav_msgs::CommandTrajectoryPositionYaw trajectory_msg;
  
  std::string pkgPath = ros::package::getPath("nbvplanner");
  std::ifstream wp_file((pkgPath+"/resource/wind_turbine_path.txt").c_str());

  std::vector<Eigen::Vector4f> waypoints;

  if (wp_file.is_open()) {
    double x, y, z, yaw;
    // Only read complete waypoints.
    while (wp_file >> x >> y >> z >> yaw) {
      waypoints.push_back(Eigen::Vector4f(x, y, z, yaw));
    }
    wp_file.close();
    ROS_INFO("Read %d waypoints.", (int )waypoints.size());
  }

  ROS_INFO("Starting the planner");
  nh.param<double>("wp_x", trajectory_msg.position.x, 1.0);
  nh.param<double>("wp_y", trajectory_msg.position.y, 10.0);
  nh.param<double>("wp_z", trajectory_msg.position.z, 0.1);
  trajectory_msg.yaw = 0.0;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_pub.publish(trajectory_msg);
  ros::Duration(10.0).sleep();
  
  double VMAX = 0.5; // TODO: change every time!
  double YAWMAX = 0.75; // TODO: change every time!
  
  std::fstream file;
  file.open((pkgPath+"/data/path.m").c_str(), std::ios::out);
  if(!file.is_open())
    ROS_WARN("could not open path file");
  file<<"pathMatrix = [";
  int l = 0;
  while (ros::ok() && waypoints.size()>l+1) {
    Eigen::Vector4f dvec = waypoints[l+1] - waypoints[l];
    dvec[3] = 0.0;
    double dyaw = waypoints[l+1][3] - waypoints[l][3];
    if(dyaw>M_PI)
      dyaw-=2.0*M_PI;
    if(dyaw<-M_PI)
      dyaw+=2.0*M_PI;
    ROS_INFO("Progress: %i/%i", l, waypoints.size());
    double dtime = std::max(sqrt(dvec.dot(dvec))/VMAX, abs(dyaw)/YAWMAX);
    int iter = dtime/dt;
    for(int i = 0; i<iter; i++)
    {
      double p = ((double)i)/((double)iter);
      nh.param<double>("wp_x", trajectory_msg.position.x, waypoints[l][0]+p*dvec[0]);
      nh.param<double>("wp_y", trajectory_msg.position.y, waypoints[l][1]+p*dvec[1]);
      nh.param<double>("wp_z", trajectory_msg.position.z, waypoints[l][2]+p*dvec[2]);
      nh.param<double>("wp_yaw", trajectory_msg.yaw, waypoints[l][3]+p*dyaw);
      trajectory_msg.header.stamp = ros::Time::now();
      trajectory_pub.publish(trajectory_msg);
      file << waypoints[l][0]+p*dvec[0]<<", "<<waypoints[l][1]+p*dvec[1]<<", "<<waypoints[l][2]+p*dvec[2]<<", "<<waypoints[l][3]+p*dyaw<<", "<<trajectory_msg.header.stamp.toSec()<<";\n";
      ros::Duration(dt).sleep();
    }
    l++;
  }
  file<<"];";
  file.close();
}
