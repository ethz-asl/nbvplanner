#ifndef TREE_HPP_
#define TREE_HPP_

#include <nbvplanner/tree.h>

template<typename stateVec>
nbvInspection::Node<stateVec>::Node()
{
  parent_ = NULL;
  distance_ = DBL_MAX;
  gain_ = 0.0;
}

template<typename stateVec>
nbvInspection::Node<stateVec>::~Node()
{
  for (typename std::vector<Node<stateVec> *>::iterator it = children_.begin();
      it != children_.end(); it++) {
    delete (*it);
    (*it) = NULL;
  }
}

template<typename stateVec>
nbvInspection::TreeBase<stateVec>::TreeBase()
{
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;
  counter_ = 0;
  rootNode_ = NULL;
}

template<typename stateVec>
nbvInspection::TreeBase<stateVec>::TreeBase(mesh::StlMesh * mesh,
                                            volumetric_mapping::OctomapManager * manager)
{
  mesh_ = mesh;
  manager_ = manager;
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;
  counter_ = 0;
  rootNode_ = NULL;
}

template<typename stateVec>
nbvInspection::TreeBase<stateVec>::~TreeBase()
{
}

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::setParams(Params params)
{
  params_ = params;
}

template<typename stateVec>
int nbvInspection::TreeBase<stateVec>::getCounter()
{
  return counter_;
}

template<typename stateVec>
bool nbvInspection::TreeBase<stateVec>::gainFound()
{
  return bestGain_ > params_.zero_gain_;
}

#endif
