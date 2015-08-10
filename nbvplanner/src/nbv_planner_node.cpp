#include <eigen3/Eigen/Dense>
#include <nbvplanner/nbvp.h>
#include <nbvplanner/nbvp.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "nbvPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nbvInspection::nbvPlanner<Eigen::Matrix<double, 4, 1>, RrtTree > planner(nh, nh_private);
  
  ros::spin();
  return 0;
}
