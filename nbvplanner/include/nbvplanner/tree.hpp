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

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::insertPointcloudWithTf(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  manager_->insertPointcloudWithTf(pointcloud);
}

template<typename stateVec>
void nbvInspection::TreeBase<stateVec>::evade(const multiagent_collision_check::Segment& segmentMsg) {
  int i;
  for(i = 0; i < agentNames_.size(); i++) {
    if(agentNames_[i] == segmentMsg.header.frame_id) {
      break;
    }
  }
  if (i == agentNames_.size()) {
    agentNames_.push_back(segmentMsg.header.frame_id);
    segments_.push_back(new std::vector<Eigen::Vector3d>);
  }
  segments_[i]->clear();
  for(typename std::vector<geometry_msgs::Pose>::const_iterator it = segmentMsg.poses.begin(); it != segmentMsg.poses.end(); it++) {
    segments_[i]->push_back(Eigen::Vector3d(it->position.x, it->position.y, it->position.z));
  }
}

#endif
