#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "nbvPlanner/nbvp.hpp"


using namespace Eigen;
using namespace nbvInspection;

typedef nbvPlanner<Vector3f> planner_t;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nbvPlanner");
  ros::NodeHandle n;
  ros::Publisher inspectionPoints = n.advertise<geometry_msgs::PoseStamped>("inspectionPoint", 1000);
  ROS_INFO("Starting NBV Planner");
  ros::Rate r(10);
  Vector3f root(0,0,0);
  int k = 0;
  while(ros::ok())
  {
    r.sleep();
    planner_t planner;
    std::vector<Vector3f> path = planner.expand(planner, 3, 5, root, &planner_t::sampleHolonomic, &planner_t::informationGainSimple);
    std::reverse(path.begin(), path.end());
    for(typename std::vector<Vector3f>::iterator it = path.begin(); it!=path.end(); it++)
    {
      geometry_msgs::PoseStamped p;
      p.header.stamp = ros::Time::now();
      p.header.seq = k; k++;
      p.header.frame_id = "/world";
      p.pose.position.x = (*it)[0];
      p.pose.position.y = (*it)[1];
      p.pose.position.z = (*it)[2];
      p.pose.orientation.x = 0.0;
      p.pose.orientation.y = 0.0;
      p.pose.orientation.z = 0.0;
      p.pose.orientation.w = 1.0;
      inspectionPoints.publish(p);
      root = path.back();
    }
  }
  ROS_INFO("done");
  return true;
}
