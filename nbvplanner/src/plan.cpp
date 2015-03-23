#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "nbvPlanner/nbvp.hpp"
#include "nbvPlanner/nbvp_srv.h"
#include "tf/transform_datatypes.h"
#include <octomap_msgs/Octomap.h>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <planning_msgs/Octomap.h>
#include <octomap_ros/conversions.h>
#include <iostream>
using namespace Eigen;
using namespace nbvInspection;

typedef Matrix<float, 4,1> stateVec_t;
typedef nbvplanner<stateVec_t> planner_t;
typedef octomap_msgs::GetOctomap OctomapSrv;

planner_t * planner;
stateVec_t * root;
stateVec_t * g_stateOld;
ros::Publisher inspectionPath;
ros::ServiceClient octomapClient;
ros::Time g_timeOld;
int g_ID;

void posCallback(const geometry_msgs::PoseStamped& pose)
{
  if(root == NULL)
  {
    root = new stateVec_t;
  }
  ros::Duration dt = pose.header.stamp - g_timeOld;
  if(dt.toSec()>0.0 || root->size()<8)
  {
    (*root)[0] = pose.pose.position.x;
    (*root)[1] = pose.pose.position.y;
    (*root)[2] = pose.pose.position.z;
    tf::Pose poseTF;
    tf::poseMsgToTF(pose.pose, poseTF);
    (*root)[3] = tf::getYaw(poseTF.getRotation());
    /*if(g_stateOld == NULL)
    {
      g_stateOld = new stateVec_t;
      (*root)[4] = 0.0;
      (*root)[5] = 0.0;
      (*root)[6] = 0.0;
      (*root)[7] = 0.0;
    }
    else
    {
      (*root)[4] = (pose.pose.position.x - (*g_stateOld)[0])/dt.toSec();
      (*root)[5] = (pose.pose.position.y - (*g_stateOld)[1])/dt.toSec();
      (*root)[6] = (pose.pose.position.z - (*g_stateOld)[2])/dt.toSec();
      (*root)[7] = ((*root)[3] - (*g_stateOld)[3])/dt.toSec();
    }*/
  }
  //*g_stateOld = *root;
  //g_timeOld = pose.header.stamp;
}

bool plannerCallback(nbvPlanner::nbvp_srv::Request& req, nbvPlanner::nbvp_srv::Response& res)
{
  ROS_INFO("Starting NBV Planner");
  ros::Rate r(10);
  OctomapSrv srv;
  //if(octomapClient.waitForExistence())
  //  ROS_WARN("Octomap service exists");
  //else
  //  ROS_WARN("Octomap service does not exist");
  if(!octomapClient.call(srv))
  {
    ROS_WARN("No octomap available");
    return true;
  }
  if(planner->octomap)
  {
    delete planner->octomap;
    planner->octomap = NULL;
  }
  if(srv.response.map.binary)
  {
    planner->octomap = octomap_msgs::binaryMsgToMap(srv.response.map);
  }
  else
  {
    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(srv.response.map);
    planner->octomap = dynamic_cast<octomap::OcTree*>(tree);
  }
    
  
  int k = 0;
  if(planner == NULL || planner->octomap == NULL || root == NULL)
    return true;
  std::vector<stateVec_t> ro; ro.push_back(*root);
  g_ID = 0;
  static const int depth = 1;
  static const int width = 16;
  planner_t::vector_t path = planner->expand(*planner, depth, width, ro, &planner_t::sampleHolonomic, &planner_t::informationGainSimple);
  std::reverse(path.begin(), path.end());
  for(planner_t::vector_t::iterator it = path.begin(); it!=path.end(); it++)
  {
    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now();
    p.header.seq = k; k++;
    p.header.frame_id = "/world";
    p.pose.position.x = (*it)[0];
    p.pose.position.y = (*it)[1];
    p.pose.position.z = (*it)[2];
    tf::Quaternion quat; quat.setEuler(0.0, 0.0, (*it)[3]);
    p.pose.orientation.x = quat.x();
    p.pose.orientation.y = quat.y();
    p.pose.orientation.z = quat.z();
    p.pose.orientation.w = quat.w();
    res.path.push_back(p.pose);
    ROS_INFO("(%2.2f,%2.2f,%2.2f,%2.2f)", (*it)[0], (*it)[1], (*it)[2], (*it)[3]);
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nbvPlanner");
  ros::NodeHandle n;
  g_timeOld = ros::Time::now();
  planner = new planner_t;
  root = NULL;
  g_stateOld = NULL;
  inspectionPath = n.advertise<visualization_msgs::Marker>("inspectionPath", 1000);
  ros::ServiceServer plannerService = n.advertiseService("nbvplanner", plannerCallback);
  ros::Subscriber pos = n.subscribe("pose", 10, posCallback);
  octomapClient = n.serviceClient<OctomapSrv>("octomap_full");
  ros::spin();
  return 0;
}

