#ifndef _MESH_BASE_CPP_
#define _MESH_BASE_CPP_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nbvplanner/mesh_base.h>

mesh::MeshBase::MeshBase()
{

}

mesh::MeshBase::MeshBase(const Eigen::Vector3d x1, const Eigen::Vector3d x2,
                         const Eigen::Vector3d x3)
    : x1_(x1),
      x2_(x2),
      x3_(x3),
      isLeaf_(true),
      isHead_(true),
      isInspected_(false),
      normal_((x3 - x2).cross(x1 - x2) / 2.0)
{
}

mesh::MeshBase::MeshBase()
    : isLeaf_(true),
      isHead_(true),
      isInspected_(false)
{
}

mesh::MeshBase::~MeshBase()
{
  for (typename std::vector<MeshBase*>::iterator it = children_.begin(); it != children_.end();
      it++)
    delete (*it);
}

void mesh::MeshBase::split()
{
  isLeaf_ = false;
  // #1
  mesh::MeshBase * tmpNode = new mesh::MeshBase;
  tmpNode->x1_ = x1_;
  tmpNode->x2_ = 0.5 * (x1_ + x2_);
  tmpNode->x3_ = 0.5 * (x1_ + x3_);
  tmpNode->normal_ = normal_ / 4.0;
  tmpNode->isHead_ = false;
  children_.push_back(tmpNode);
  // #2
  tmpNode = new mesh::MeshBase;
  tmpNode->x1_ = x1_;
  tmpNode->x2_ = 0.5 * (x2_ + x3_);
  tmpNode->x3_ = 0.5 * (x2_ + x1_);
  tmpNode->normal_ = normal_ / 4.0;
  tmpNode->isHead_ = false;
  children_.push_back(tmpNode);
  // #3
  tmpNode = new mesh::MeshBase;
  tmpNode->x1_ = x2_;
  tmpNode->x2_ = 0.5 * (x3_ + x1_);
  tmpNode->x3_ = 0.5 * (x3_ + x2_);
  tmpNode->normal_ = normal_ / 4.0;
  tmpNode->isHead_ = false;
  children_.push_back(tmpNode);
  // #4
  tmpNode = new mesh::MeshBase;
  tmpNode->x1_ = 0.5 * (x1_ + x2_);
  tmpNode->x2_ = 0.5 * (x2_ + x3_);
  tmpNode->x3_ = 0.5 * (x3_ + x1_);
  tmpNode->normal_ = normal_ / 4.0;
  tmpNode->isHead_ = false;
  children_.push_back(tmpNode);
}

bool mesh::MeshBase::collapse()
{
  bool collapsible = true;
  if (isLeaf_)
    return true;
  // check if children are all collapsible
  for (typename std::vector<MeshBase*>::iterator it = children_.begin(); it != children_.end();
      it++) {
    if (!(*it)->collapse())
      collapsible = false;
  }
  // collapse if all children have same state
  if (collapsible) {
    bool state = children_.front()->isInspected_;
    for (typename std::vector<MeshBase*>::iterator it = children_.begin(); it != children_.end();
        it++) {
      if ((*it)->isHead_)
        return false;
      if ((*it)->isInspected_ != state)
        return false;
    }
    for (typename std::vector<MeshBase*>::iterator it = children_.begin(); it != children_.end();
        it++)
      delete (*it);
    children_.clear();
    isInspected_ = state;
    isLeaf_ = true;
  }
  return true;
}

#endif // _MESH_BASE_CPP_
