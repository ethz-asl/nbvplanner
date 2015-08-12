#ifndef RRTTREE_H_
#define RRTTREE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <kdtree/kdtree.h>
#include <nbvplanner/tree.h>
#include <nbvplanner/mesh_structure.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {

class RrtTree : public TreeBase<Eigen::Vector4d>
{
 public:
  typedef Eigen::Vector4d StateVec;

  RrtTree();
  RrtTree(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
  ~RrtTree();
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseStamped& pose);
  virtual void initialize();
  virtual void iterate(int iterations);
  virtual std::vector<geometry_msgs::Pose> getBestEdge();
  virtual void clear();
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious();
  virtual void memorizeBestBranch();
  void publishNode(Node<StateVec> * node);
  double gain(StateVec state);
  std::vector<geometry_msgs::Pose> samplePath(StateVec start, StateVec end);
 protected:
  kdtree * kdTree_;
  std::stack<StateVec> history_;
  std::vector<StateVec> bestBranchMemory_;
  int g_ID_;
  int iterationCount_;
  ;
  std::fstream fileTree_;
  std::fstream filePath_;
  std::fstream fileResponse_;
  std::string logFilePath_;
};
}

#endif
