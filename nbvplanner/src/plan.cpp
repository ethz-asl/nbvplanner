#include <ros/ros.h>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <planning_msgs/Octomap.h>
#include <octomap_ros/conversions.h>
#include <tf/transform_datatypes.h>

#include <octomap_to_mesh/rt_nonfinite.h>
#include <octomap_to_mesh/rtwtypes.h>
#include <octomap_to_mesh/OctomapToMesh_types.h>
#include <octomap_to_mesh/OctomapToMesh_struct.h>
#include <octomap_to_mesh/OctomapToGrid.h>
#include <octomap_to_mesh/OctomapToMesh.h>

#include "nbvplanner/nbvp.hpp"
#include "nbvplanner/nbvp_srv.h"

using namespace Eigen;
using namespace nbvInspection;

typedef Matrix<float, 4,1> stateVec_t;
typedef nbvPlanner<stateVec_t> planner_t;
typedef octomap_msgs::GetOctomap OctomapSrv;

planner_t * planner;
stateVec_t * root;
stateVec_t * g_stateOld;
ros::Publisher inspectionPath;
ros::Publisher treePub;
ros::ServiceClient octomapClient;
ros::Time g_timeOld;
ros::Time g_timeSinceLog;
int g_ID;
std::string pkgPath;
int iteration;

void posCallback(const geometry_msgs::PoseStamped& pose)
{
  delete planner->rootNode_;
  planner->rootNode_ = NULL;
  nbvInspection::Node<stateVec_t>::bestNode_ = NULL;
  nbvInspection::Node<stateVec_t>::bestInformationGain_ = nbvInspection::Node<stateVec_t>::ZERO_INFORMATION_GAIN_;
  
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
  g_timeOld = pose.header.stamp;
  // logging of position
  if((pose.header.stamp-g_timeSinceLog).toSec()>1.0)
  {
    std::fstream traj;
    traj.open((pkgPath+"/data/traj.m").c_str(), std::ios::out | std::ios::app);
    if(!traj.is_open())
      ROS_WARN("could not open path file");
    traj<<pose.pose.position.x<<", ";
    traj<<pose.pose.position.y<<", ";
    traj<<pose.pose.position.z<<", ";
    traj<<(*root)[3]<<", ";
    traj<<pose.header.stamp.toSec()<<";\n";
    traj.close();
    g_timeSinceLog = pose.header.stamp;
  }
}

bool plannerCallback(nbvplanner::nbvp_srv::Request& req, nbvplanner::nbvp_srv::Response& res)
{
  ROS_INFO_THROTTLE(1 , "Starting NBV Planner");
  if(!ros::ok())
  {
    ROS_INFO_THROTTLE(1 , "Exploration completed. Not planning any further moves.");
    ros::Duration(5.0).sleep();
  }
  if(!planner_t::setParams())
  {
    ROS_ERROR_THROTTLE(1 , "Could not start the planner. Parameters missing!");
    return true;
  }

  int k = 0;
  if(planner == NULL || root == NULL || planner_t::manager_ == NULL || planner_t::manager_->getMapSize().norm() <= 0.0)
  {
    ROS_ERROR_THROTTLE(1 , "Planner not set up");
    return true;
  }
  planner_t::manager_->publishAll();
  std::vector<stateVec_t> ro; ro.push_back(*root);
  g_ID = 0;
  static const int depth = 2;
  static const int width = 16;
  double IG = 0.0;
  ros::Time start = ros::Time::now();
  planner_t::vector_t path;
  if(nbvInspection::nbvPlanner<stateVec_t>::getRRTextension())
  {
    int initIter = nbvInspection::nbvPlanner<stateVec_t>::getInitIterations();
    if(initIter == 0)
    {
      ROS_ERROR("Planning aborted. Parameter initial iterations is either missing or zero");
      return true;
    }
    path = planner->expandStructured(*planner, initIter, *root, IG, &planner_t::informationGainCone);
  }
  else
  {
    if(!nbvInspection::nbvPlanner<stateVec_t>::extensionRangeSet())
    {
      ROS_ERROR("Planning aborted. Parameter extension range is either missing or zero");
      return true;
    }
    path = planner->expand(*planner, depth, width, ro, IG, &planner_t::sampleHolonomic, &planner_t::informationGainCone);
  }
  ros::Duration duration = ros::Time::now() - start;
  
  // calculate explored space
  int mappedFree = 0;
  int mappedOccupied = 0;
  //for(typename octomap::OcTree::leaf_iterator it = planner->octomap_->begin_leafs(), end = planner->octomap_->end_leafs(); it != end; it++)
  //{
  //  if(planner->octomap_->isNodeOccupied(*it))
  //    mappedOccupied++;
  //  else
  //    mappedFree++;
  //}
  double vol = 0.0;//pow(planner->octomap_->getResolution(), 3.0);
  //ROS_INFO("Total volume of %2.2f mapped. Thereof free and occupied: %2.2f/%2.2f",
  //          vol * (double)(mappedFree + mappedOccupied),
  //          vol * (double) mappedFree,
  //          vol * (double) mappedOccupied);
  
  // write planning information to file for postprocessing
  std::fstream tree;
  tree.open((pkgPath+"/data/tree.m").c_str(), std::ios::out | std::ios::app);
  if(!tree.is_open())
    ROS_WARN("could not open path file");
  tree<<"stamp{"<<iteration<<"}="<<start.toSec()<<";\n";
  tree<<"duration{"<<iteration<<"}="<<duration.toSec()<<";\n";
  tree<<"IG{"<<iteration<<"}="<<IG<<";\n";
  tree<<"tree{"<<iteration<<"}=[";
  planner->rootNode_->printToFile(tree);
  tree<<"];\n";
  tree.close();
  ROS_INFO("Replanning lasted %4.4fs and has a Gain of %2.2f, with %i iterations", duration.toSec(), IG, nbvInspection::Node<stateVec_t>::getCounter());
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
    //ROS_INFO("(%2.2f,%2.2f,%2.2f,%2.2f)", (*it)[0], (*it)[1], (*it)[2], (*it)[3]);
  }
  iteration++;
  if(!ros::ok()||iteration==50)
  {
    ROS_INFO("Exploration completed. Converting octomap to mesh for inspection planning.");
    
    //OctomapToMesh_T data;

    //if(OctomapToGrid (planner->octomap_, &data)) {
                      
      //struct_T * MeshedOctomap = NULL;
      //char_T * fileName = new char_T[50];
      //strcpy(fileName, (pkgPath+"/data/meshOut.stl").c_str());
      //OctomapToMesh(&data, fileName, MeshedOctomap);
      
      // TODO(birchera): clean up allocated memory
      /*std::fstream meshFile;
      meshFile.open((pkgPath+"/data/meshOut.m").c_str(), std::ios::out);
      meshFile << "octomap_gridX = [";
      for (int i = 0; i < (*octomap_gridX_size)[1]; i++)
        meshFile << (*octomap_gridX_data)[i] << ";\n";
      meshFile << "];\n";
      meshFile << "octomap_gridY = [";
      for (int i = 0; i < (*octomap_gridY_size)[1]; i++)
        meshFile << (*octomap_gridY_data)[i] << ";\n";
      meshFile << "];\n";
      meshFile << "octomap_gridZ = [";
      for (int i = 0; i < (*octomap_gridZ_size)[1]; i++)
        meshFile << (*octomap_gridZ_data)[i] << ";\n";
      meshFile << "];\n";
      
      meshFile << "octomap_voxels_map = zeros(" << (*octomap_gridX_size)[1] << ", " << (*octomap_gridY_size)[1] << ", " << (*octomap_gridZ_size)[1] << ");\n";
      
      for (int i = 0; i < (*octomap_gridX_size)[1]; i++)
        for (int j = 0; j < (*octomap_gridY_size)[1]; j++)
          for (int k = 0; k < (*octomap_gridZ_size)[1]; k++)
            meshFile << "octomap_voxels_map(" << i+1 << ", " << j+1 << ", " << k+1 << ") = " << (*octomap_voxels_map)->data[i+j*(*octomap_gridX_size)[1]+k*(*octomap_gridX_size)[1]*(*octomap_gridY_size)[1]] << ";\n";
      meshFile.close();*/
    //}
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nbvPlanner");
  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");
  
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  volumetric_mapping::OctomapManager manager(n, nh_private);
  planner_t::manager_ = &manager;
  
  // prepare log files for postprocessing
  pkgPath = ros::package::getPath("nbvplanner");
  std::fstream traj;
  traj.open((pkgPath+"/data/traj.m").c_str(), std::ios::out);
  if(!traj.is_open())
    ROS_WARN("could not open path file");
  traj<<"trajMatrix = [";
  traj.close();
  
  std::fstream tree;
  tree.open((pkgPath+"/data/tree.m").c_str(), std::ios::out);
  if(!tree.is_open())
    ROS_WARN("could not open path file");
  tree<<"% tree file\n";
  tree.close();
  
  // initialize global variables
  g_timeOld = ros::Time::now();
  g_timeSinceLog = ros::Time::now();
  planner = new planner_t;
  root = NULL;
  g_stateOld = NULL;
  iteration = 1;
  
  // set up the topics and services
  inspectionPath = n.advertise<visualization_msgs::Marker>("inspectionPath", 1000);
  treePub = n.advertise<geometry_msgs::PolygonStamped>("treePol", 1000);
  ros::ServiceServer plannerService = n.advertiseService("nbvplanner", plannerCallback);
  ros::Subscriber pos = n.subscribe("pose", 10, posCallback);
  octomapClient = n.serviceClient<OctomapSrv>("octomap_full");
  ros::spin();
  
  traj.open((pkgPath+"/data/traj.m").c_str(), std::ios::out | std::ios::app);
  if(!traj.is_open())
    ROS_WARN("could not open path file");
  traj<<"];";
  traj.close();
  return 0;
}

