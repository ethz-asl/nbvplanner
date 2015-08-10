#ifndef TREE_H_
#define TREE_H_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <nbvplanner/mesh_structure.h>

namespace nbvInspection {

struct Params
{
  double camPitch_;
  double camHorizontal_;
  double camVertical_;
  std::vector<Eigen::Vector3d> camBoundNormals_;

  double igProbabilistic_;
  double igFree_;
  double igOccupied_;
  double igUnmapped_;
  double igArea_;
  double gainRange_;
  double degressiveCoeff_;
  double zero_gain_;

  double v_max_;
  double dyaw_max_;
  double dOvershoot_;
  double extensionRange_;
  int initIterations_;
  int cuttoffIterations_;
  double dt_;

  double minX_;
  double minY_;
  double minZ_;
  double maxX_;
  double maxY_;
  double maxZ_;
  bool softBounds_;
  Eigen::Vector3d boundingBox_;

  double meshResolution_;

  ros::Publisher inspectionPath_;
  ros::Publisher treePub_;
};

template<typename stateVec>
class Node
{
 public:
  Node();
  ~Node();
  stateVec state_;
  Node * parent_;
  std::vector<Node*> children_;
  double gain_;
  double distance_;
};

template<typename stateVec>
class TreeBase
{
 protected:
  Params params_;
  int counter_;
  double bestGain_;
  Node<stateVec> * bestNode_;
  Node<stateVec> * rootNode_;
  mesh::StlMesh * mesh_;
  volumetric_mapping::OctomapManager * manager_;
 public:
  TreeBase();
  TreeBase(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
  ~TreeBase();
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseStamped& pose) = 0;
  virtual void iterate(int iterations) = 0;
  virtual void initialize() = 0;
  virtual std::vector<geometry_msgs::Pose> getBestEdge() = 0;
  virtual void clear() = 0;
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious() = 0;
  virtual void memorizeBestBranch() = 0;
  void setParams(Params params);
  int getCounter();
  bool gainFound();
};
}

#endif
