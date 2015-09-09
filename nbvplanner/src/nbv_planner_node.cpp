#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nbvplanner/nbvp.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nbvPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nbvInspection::nbvPlanner<Eigen::Matrix<double, 4, 1> > planner(nh, nh_private);

  ros::spin();
  return 0;
}
