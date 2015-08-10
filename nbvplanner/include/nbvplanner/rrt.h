#ifndef RRTTREE_H_
#define RRTTREE_H_

#include <nbvplanner/tree.h>

namespace nbvInspection {

class RrtTree : public Tree<Eigen::Vector4d>
{
  kdtree * kdTree_;
  std::stack<stateVec> history_;
  std::vector<StateVec> bestBranchMemory_;
  int g_ID_;
 public:
  typedef Eigen::Vector4d StateVec;

  RrtTree();
  virtual RrtTree(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
  ~RrtTree();
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseStamped& pose);
  virtual void initialize();
  virtual void iterate(int iterations);
  virtual std::vector<geometry_msgs::Pose> getBestEdge();
  virtual void clear();
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious();
  virtual void memorizeBestBranch();
  void publishNode(Node<StateVec> node);
  double gain(stateVec state);
  std::vector<geometry_msgs::Pose> samplePath(StateVec start, StateVec end);
};
}

#endif
