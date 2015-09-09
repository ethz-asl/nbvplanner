#ifndef _MESH_BASE_H_
#define _MESH_BASE_H_

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

namespace mesh {

class MeshBase
{
 public:
  MeshBase();
  MeshBase(const Eigen::Vector3d x1, const Eigen::Vector3d x2, const Eigen::Vector3d x3);
  ~MeshBase();

 protected:
  void split();
  bool collapse();
  virtual double computeInspectableArea(const tf::Transform& transform) const = 0;

  bool isLeaf_;
  bool isHead_;
  bool isInspected_;
  std::vector<MeshBase*> children_;
  Eigen::Vector3d x1_;
  Eigen::Vector3d x2_;
  Eigen::Vector3d x3_;
  Eigen::Vector3d normal_;
};
}

#endif // _MESH_BASE_H_
