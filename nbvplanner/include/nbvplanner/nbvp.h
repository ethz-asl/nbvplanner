#ifndef NBVP_H_
#define NBVP_H_

#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OcTree.h>
#include <octomap_world/octomap_manager.h>
#include <kdtree/kdtree.h>
#include <nbvplanner/nbvp_srv.h>
#include <nbvplanner/mesh_structure.h>
#include <nbvplanner/tree.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {

  template<typename stateVec, class Tree>
  class nbvPlanner {
  
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    stateVec root_;
    stateVec g_stateOld_;
    
    ros::ServiceClient octomapClient_;
    ros::Subscriber posClient0_;
    ros::Subscriber posClient1_;
    ros::Subscriber posClient2_;
    ros::Subscriber posClient3_;
    ros::Subscriber posClient4_;
    ros::ServiceServer plannerService0_;
    ros::ServiceServer plannerService1_;
    ros::ServiceServer plannerService2_;
    ros::ServiceServer plannerService3_;
    ros::ServiceServer plannerService4_;
    ros::Subscriber pointcloud_sub0_;
    ros::Subscriber pointcloud_sub1_;
    ros::Subscriber pointcloud_sub2_;
    ros::Subscriber pointcloud_sub3_;
    ros::Subscriber pointcloud_sub4_;

    Params params_;
    mesh::StlMesh * mesh_;
    volumetric_mapping::OctomapManager * manager_;

    bool ready_;

  public:
    typedef std::vector<stateVec> vector_t;
    typedef octomap::OcTree octomap_t;
    TreeBase * tree_;
    
    nbvPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~nbvPlanner();
    vector_t buildTree(nbvPlanner<stateVec>& instance, int I, stateVec s, double& Iout, int agentID);
    bool setParams();

    void posCallback0(const geometry_msgs::PoseStamped& pose);
    void posCallback1(const geometry_msgs::PoseStamped& pose);
    void posCallback2(const geometry_msgs::PoseStamped& pose);
    void posCallback3(const geometry_msgs::PoseStamped& pose);
    void posCallback4(const geometry_msgs::PoseStamped& pose);
    void posCallback(const geometry_msgs::PoseStamped& pose, int agentID);
    bool plannerCallback0(nbvplanner::nbvp_srv::Request& req,
                         nbvplanner::nbvp_srv::Response& res);
    bool plannerCallback1(nbvplanner::nbvp_srv::Request& req,
                         nbvplanner::nbvp_srv::Response& res);
    bool plannerCallback2(nbvplanner::nbvp_srv::Request& req,
                         nbvplanner::nbvp_srv::Response& res);
    bool plannerCallback3(nbvplanner::nbvp_srv::Request& req,
                         nbvplanner::nbvp_srv::Response& res);
    bool plannerCallback4(nbvplanner::nbvp_srv::Request& req,
                         nbvplanner::nbvp_srv::Response& res);
    bool plannerCallback(nbvplanner::nbvp_srv::Request& req,
                         nbvplanner::nbvp_srv::Response& res, int agentID);
                         
  };
}

#endif // NBVP_H_
