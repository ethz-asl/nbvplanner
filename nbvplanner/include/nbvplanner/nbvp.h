#ifndef NBVP_H_
#define NBVP_H_

#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap_world/octomap_manager.h>
#include <nbvplanner/nbvp_srv.h>
#include <nbvplanner/mesh_structure.h>
#include <nbvplanner/tree.hpp>
#include <nbvplanner/rrt.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {

template<typename stateVec>
class nbvPlanner
{

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceClient octomapClient_;
  ros::Subscriber posClient_;
  ros::ServiceServer plannerService_;
  ros::Subscriber pointcloud_sub_;

  Params params_;
  mesh::StlMesh * mesh_;
  volumetric_mapping::OctomapManager * manager_;

  bool ready_;

 public:
  typedef std::vector<stateVec> vector_t;
  TreeBase<stateVec> * tree_;

  nbvPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~nbvPlanner();
  vector_t buildTree(nbvPlanner<stateVec>& instance, int I, stateVec s, double& Iout, int agentID);
  bool setParams();
  void posCallback(const geometry_msgs::PoseStamped& pose);
  bool plannerCallback(nbvplanner::nbvp_srv::Request& req, nbvplanner::nbvp_srv::Response& res);
  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
};
}

#endif // NBVP_H_
