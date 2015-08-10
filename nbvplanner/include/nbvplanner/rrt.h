#ifndef RRTTREE_H_
#define RRTTREE_H_

#include <nbvplanner/tree.h>
#include <eigen3/Eigen/Dense>
#include <nbvplanner/mesh_structure.h>
#include <kdtree/kdtree.h>

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
};
}

#endif
