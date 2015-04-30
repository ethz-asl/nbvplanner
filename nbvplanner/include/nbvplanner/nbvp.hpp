#include "nbvplanner/nbvp.h"
#include <cfloat>
#include <cstdlib>
#include <sstream>
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PolygonStamped.h"

using namespace Eigen;

extern int g_ID;
extern ros::Publisher inspectionPath;
extern ros::Publisher treePub;

template<typename stateVec>
nbvInspection::Node<stateVec>::Node()
{
  parent_ = NULL;
  informationGain_ = 0.0;
  counter_++;
}
    
template<typename stateVec>
nbvInspection::Node<stateVec>::~Node()
{
  for(typename std::vector<Node<stateVec> *>::iterator it = children_.begin(); it != children_.end(); it++)
  {
    delete (*it);
    (*it) = NULL;
  }
  counter_--;
}
    
template<typename stateVec>
nbvInspection::Node<stateVec> * nbvInspection::Node<stateVec>::minDist(stateVec s)
{
  double bestDist = sqrt(SQ(s[0] - state_[0]) + SQ(s[1] - state_[1]) + SQ(s[2] - state_[2]));
  nbvInspection::Node<stateVec> * ret = this;
  for(typename std::vector<nbvInspection::Node<stateVec> *>::iterator it = children_.begin(); it != children_.end(); it++)
  {
    nbvInspection::Node<stateVec> * tmp = (*it)->minDist(s);
    double tmpDist = sqrt(SQ(s[0] - tmp->state_[0]) + SQ(s[1] - tmp->state_[1]) + SQ(s[2] - tmp->state_[2]));
    if(tmpDist<bestDist)
    {
      ret = tmp;
      bestDist = tmpDist;
    }
  }
  return ret;
}

template<typename stateVec>
int nbvInspection::Node<stateVec>::getCounter()
{
  return nbvInspection::Node<stateVec>::counter_;
}

template<typename stateVec>
void nbvInspection::Node<stateVec>::printToFile(std::fstream& file)
{
  if(parent_)
  {
    for(int i = 0; i<state_.size(); i++)
      file<<state_[i]<<", ";
    for(int i = 0; i<parent_->state_.size()-1; i++)
      file<<parent_->state_[i]<<", ";
    file<<parent_->state_[parent_->state_.size()-1]<<";\n";
  }
  for(typename std::vector<nbvInspection::Node<stateVec> *>::iterator it = children_.begin(); it != children_.end(); it++)
  {
    (*it)->printToFile(file);
  }
}

template<typename stateVec>
const double nbvInspection::Node<stateVec>::ZERO_INFORMATION_GAIN_ = 100.0;
template<typename stateVec>
double nbvInspection::Node<stateVec>::bestInformationGain_ = nbvInspection::Node<stateVec>::ZERO_INFORMATION_GAIN_;
template<typename stateVec>
nbvInspection::Node<stateVec> * nbvInspection::Node<stateVec>::bestNode_ = NULL;
template<typename stateVec>
int nbvInspection::Node<stateVec>::counter_ = 0;

template<typename stateVec>
nbvInspection::nbvPlanner<stateVec>::nbvPlanner()
{
  double pitch = M_PI*nbvInspection::nbvPlanner<stateVec>::camPitch_/180.0;
  double camTop = M_PI*(pitch-nbvInspection::nbvPlanner<stateVec>::camVertical_/2.0)/180.0;
  double camBottom = M_PI*(pitch+nbvInspection::nbvPlanner<stateVec>::camVertical_/2.0)/180.0;
  double side = M_PI*(nbvInspection::nbvPlanner<stateVec>::camHorizontal_)/360.0;
  Vector3f bottom(cos(camBottom), -sin(camBottom), 0.0);
  Vector3f top(cos(camTop), -sin(camTop), 0.0);
  Vector3f right(cos(side), -sin(camBottom), 0.0);
  Vector3f left(cos(side), sin(camBottom), 0.0);
  AngleAxisf m = AngleAxisf(pitch, Vector3f::UnitY());
  Vector3f rightR = m*right;
  Vector3f leftR = m*left;
  rightR.normalize();
  leftR.normalize();
  camBoundNormals_.push_back(bottom);
  camBoundNormals_.push_back(top);
  camBoundNormals_.push_back(rightR);
  camBoundNormals_.push_back(leftR);
  
  rootNode_ = NULL;
  octomap_ = NULL;
}

template<typename stateVec>
nbvInspection::nbvPlanner<stateVec>::~nbvPlanner()
{
  delete rootNode_;
  rootNode_ = NULL;
  delete octomap_;
  octomap_ = NULL;
}

template<typename stateVec>
typename nbvInspection::nbvPlanner<stateVec>::vector_t nbvInspection::nbvPlanner<stateVec>::expand(nbvPlanner<stateVec>& instance, int N, int M, nbvInspection::nbvPlanner<stateVec>::vector_t s, double& IGout, nbvInspection::nbvPlanner<stateVec>::vector_t (nbvInspection::nbvPlanner<stateVec>::*sample)(stateVec), double (nbvInspection::nbvPlanner<stateVec>::*informationGain)(stateVec))
{
  double IG = (instance.*informationGain)(s.front());
  double IGnew = 0.0;
  nbvInspection::nbvPlanner<stateVec>::vector_t path;
  nbvInspection::nbvPlanner<stateVec>::vector_t ret;
  ret = s;
  IGout = 0.0;
  
  if(N<=0)
    return ret;
    
  for(int m = 0; m<M; m++)
  {
    path = instance.expand(instance, N-1, M, (instance.*sample)(s.front()), IGnew, sample, informationGain);
    if(IG + nbvInspection::nbvPlanner<stateVec>::degressiveCoeff_ * IGnew > IGout)
    {
      path.insert(path.end(),s.begin(),s.end());
      ret = path;
      IGout = IG + nbvInspection::nbvPlanner<stateVec>::degressiveCoeff_ * IGnew;
    }
  }
  //ROS_INFO("Information Gain is: %2.2f", IGout);
  return ret;
}


template<typename stateVec>
typename nbvInspection::nbvPlanner<stateVec>::vector_t nbvInspection::nbvPlanner<stateVec>::expandStructured(nbvInspection::nbvPlanner<stateVec>& instance, int I, stateVec s, double& IGout, double (nbvInspection::nbvPlanner<stateVec>::*informationGain)(stateVec))
{
  assert(s.size() == 4);
  nbvInspection::nbvPlanner<stateVec>::vector_t ret;
  if(!rootNode_)
  {
    rootNode_ = new nbvInspection::Node<stateVec>;
    rootNode_->state_ = s;
  }
  // iterate as long as no information is found
  int localCount = 0;
  while(nbvInspection::Node<stateVec>::bestInformationGain_ <= nbvInspection::Node<stateVec>::ZERO_INFORMATION_GAIN_ || nbvInspection::Node<stateVec>::getCounter() < I)
  {
    if(nbvInspection::Node<stateVec>::getCounter()>1000)
    {
      ROS_INFO("No information gain found, shutting down");
      ros::shutdown();
      return ret;
    }
    if(localCount > 2000)
    {
    	stateVec extension(0.0, 0.0, 0.0, 0.0);
    	if(history_.size() > 0)
    	  extension = history_.top() - s;
			double wp = extension.norm() / (nbvInspection::nbvPlanner<stateVec>::v_max_ *
				  nbvInspection::nbvPlanner<stateVec>::dt_);
		  if(extension[3] < -M_PI)
		  	extension[3] += 2.0 * M_PI;
		  if(extension[3] > M_PI)
		  	extension[3] += 2.0 * M_PI;
			for(double i = 0.0; i<wp; i+=1.0)
		  	ret.push_back(s+(1.0-i/wp)*extension);
    
		  history_.pop();
			return ret;
    }
    // set up boundaries: increase size as number of iterations grows
    double radius = extensionRange_ * log(1.0 + pow((double)nbvInspection::Node<stateVec>::getCounter(), 2.0) /
                              ((double)localCount + 1.0));
    // sample position of new state
    stateVec newState;
    double dsq = 0.0;
    do
    {
      for(int i = 0; i<3; i++)
        newState[i] = 2.0*radius*(((double)rand())/((double)RAND_MAX)-0.5);
      dsq = SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]);
    } while(dsq > pow(radius, 2.0));
    // offset new state by root
    newState += s;
    nbvInspection::Node<stateVec>* newParent = rootNode_->minDist(newState);
    
    // check for collision
    octomap::point3d origin;
    origin.x() = newParent->state_[0];
    origin.y() = newParent->state_[1];
    origin.z() = newParent->state_[2];
    octomap::point3d direction;
    direction.x() = newState[0];
    direction.y() = newState[1];
    direction.z() = newState[2];
    direction -= origin;
    octomap::point3d end;
    end.x() = 0.0;
    end.y() = 0.0;
    end.z() = 0.0;
    double d = sqrt(SQ(newState[0] - newParent->state_[0]) + SQ(newState[1] - newParent->state_[1]) + SQ(newState[2] - newParent->state_[2]));
    bool ignoreUnknownCells = false; // TODO: shoud be false, but free cells are not mapped at this time
    if(!this->castRay(origin, direction, end, ignoreUnknownCells, d))
    {
      // sample the new orientation from the set of possible orientations
      newState[3] = newParent->state_[3] + 2.0 * (((double)rand())/((double)RAND_MAX)-0.5) * d * nbvInspection::nbvPlanner<stateVec>::dyaw_max_ / nbvInspection::nbvPlanner<stateVec>::v_max_;
      // create new node and inser into tree
      nbvInspection::Node<stateVec> * newNode = new nbvInspection::Node<stateVec>;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newParent->children_.push_back(newNode);
      newNode->informationGain_ = newParent->informationGain_ + (instance.*informationGain)(newNode->state_);
      // display new node
      
      visualization_msgs::Marker p;
      p.header.stamp = ros::Time::now();
      p.header.seq = g_ID;
      p.header.frame_id = "/world";
      p.id = g_ID; g_ID++;
      p.ns="vp_tree";
      p.type = visualization_msgs::Marker::ARROW;
      p.action = visualization_msgs::Marker::ADD;
      p.pose.position.x = newNode->state_[0];
      p.pose.position.y = newNode->state_[1];
      p.pose.position.z = newNode->state_[2];
      tf::Quaternion quat; quat.setEuler(0.0, 0.0, newNode->state_[3]);
      p.pose.orientation.x = quat.x();
      p.pose.orientation.y = quat.y();
      p.pose.orientation.z = quat.z();
      p.pose.orientation.w = quat.w();
      p.scale.x = std::max(newNode->informationGain_/2000.0, 0.05);
      p.scale.y = 0.1;
      p.scale.z = 0.1;
      p.color.r = 167.0/255.0;
      p.color.g = 167.0/255.0;
      p.color.b = 0.0;
      p.color.a = 1.0;
      p.lifetime = ros::Duration(0.0);
      p.frame_locked = false;
      inspectionPath.publish(p);
      
      p.id = g_ID; g_ID++;
      p.ns="vp_branches";
      p.type = visualization_msgs::Marker::ARROW;
      p.action = visualization_msgs::Marker::ADD;
      p.pose.position.x = newNode->parent_->state_[0];
      p.pose.position.y = newNode->parent_->state_[1];
      p.pose.position.z = newNode->parent_->state_[2];
      Eigen::Quaternion<float> q;
      Eigen::Vector3f init(1.0, 0.0, 0.0);
      Eigen::Vector3f dir(newNode->state_[0] - newNode->parent_->state_[0], newNode->state_[1] - newNode->parent_->state_[1], newNode->state_[2] - newNode->parent_->state_[2]);
      q.setFromTwoVectors(init , dir);
      q.normalize();
      p.pose.orientation.x = q.x();
      p.pose.orientation.y = q.y();
      p.pose.orientation.z = q.z();
      p.pose.orientation.w = q.w();
      p.scale.x = d;
      p.scale.y = 0.03;
      p.scale.z = 0.03;
      p.color.r = 100.0/255.0;
      p.color.g = 100.0/255.0;
      p.color.b = 0.7;
      p.color.a = 1.0;
      p.lifetime = ros::Duration(0.0);
      p.frame_locked = false;
      inspectionPath.publish(p);
      
      // update best IG and node if applicable
      if(newNode->informationGain_ > nbvInspection::Node<stateVec>::bestInformationGain_)
      {
        nbvInspection::Node<stateVec>::bestInformationGain_ = newNode->informationGain_;
        nbvInspection::Node<stateVec>::bestNode_ = newNode;
        
      }
    }
    localCount++;
    
  }
  // extract best path
  nbvInspection::Node<stateVec> * curr = nbvInspection::Node<stateVec>::bestNode_;
  if(curr->parent_ != NULL)
  {
    while(curr->parent_ != rootNode_ && curr->parent_ != NULL)
    {
      curr = curr->parent_;
    }
    
    double d = SQ(curr->state_[0] - curr->parent_->state_[0]) + SQ(curr->state_[1] - curr->parent_->state_[1]) + SQ(curr->state_[2] - curr->parent_->state_[2]);
    d = sqrt(d);
    double disc = nbvInspection::nbvPlanner<stateVec>::dt_ * nbvInspection::nbvPlanner<stateVec>::v_max_ / d;
    for(double it = 0.0; it < 1.0; it += disc)
    {
      ret.push_back((1.0-it)*curr->state_ + it*curr->parent_->state_);
    }
  }
  else
    ret.push_back(curr->state_);
  IGout = nbvInspection::Node<stateVec>::bestInformationGain_;
	this->history_.push(ret.back());
  return ret;
}

template<typename stateVec>
typename nbvInspection::nbvPlanner<stateVec>::vector_t nbvInspection::nbvPlanner<stateVec>::sampleHolonomic(stateVec s)
{
  assert(s.size()==4);
  nbvInspection::nbvPlanner<stateVec>::vector_t ret;
  stateVec extension;
  octomap::point3d origin;
  origin.x() = s[0];
  origin.y() = s[1];
  origin.z() = s[2];
  octomap::point3d direction;
  octomap::point3d end;
  bool ignoreUnknownCells = false; // TODO: shoud be false, but free cells are not mapped at this time
  double d = DBL_MAX;
  int iter = 0;
  do
  {
    for(int i = 0; i<extension.size()-1; i++)
      extension[i] = nbvInspection::nbvPlanner<stateVec>::extensionRange_ * (((double)rand())/((double)RAND_MAX)-0.5);
    d = sqrt(SQ(extension[0])+SQ(extension[1])+SQ(extension[2]));
    // sample yaw w.r.t. the constraints
    extension[extension.size()-1] = (nbvInspection::nbvPlanner<stateVec>::dyaw_max_ / nbvInspection::nbvPlanner<stateVec>::v_max_) * d * (((double)rand())/((double)RAND_MAX)-0.5);
    direction.x() = extension[0];
    direction.y() = extension[1];
    direction.z() = extension[2];
  }while(this->octomap_->castRay(origin, direction, end, ignoreUnknownCells, d*1.1+this->octomap_->getResolution()) && (iter++) < 100);
  if(iter>=100)
  {
    ROS_WARN("No connection found to extend tree");
    ret.push_back(s);
    return ret;
  }
  double wp = d / (nbvInspection::nbvPlanner<stateVec>::v_max_ * nbvInspection::nbvPlanner<stateVec>::dt_);
  for(double i = 0.0; i<wp; i+=1.0)
    ret.push_back(s+(1.0-i/wp)*extension);
  double IG = this->informationGainCone(s+extension);
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID;
  p.header.frame_id = "/world";
  p.id = g_ID; g_ID++;
  p.ns="vp_tree";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = ret.front()[0];
  p.pose.position.y = ret.front()[1];
  p.pose.position.z = ret.front()[2];
  tf::Quaternion quat; quat.setEuler(0.0, 0.0, ret.front()[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = std::max(IG/1000.0, 0.05);
  p.scale.y = 0.1;
  p.scale.z = 0.1;
  p.color.r = 167.0/255.0;
  p.color.g = 167.0/255.0;
  p.color.b = 0.0;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(0.0);
  p.frame_locked = false;
  inspectionPath.publish(p);
  
  geometry_msgs::PolygonStamped pol;
  pol.header.seq = g_ID; g_ID++;
  pol.header.stamp = ros::Time::now();
  pol.header.frame_id = "/world";
  geometry_msgs::Point32 point;
  point.x = s[0];
  point.y = s[1];
  point.z = s[2];
  pol.polygon.points.push_back(point);
  point.x += extension[0];
  point.y += extension[1];
  point.z += extension[2];
  pol.polygon.points.push_back(point);
  treePub.publish(pol);
  
  return ret;
}

template<typename stateVec>
typename nbvInspection::nbvPlanner<stateVec>::vector_t nbvInspection::nbvPlanner<stateVec>::sampleEuler(stateVec s)
{
  nbvInspection::nbvPlanner<stateVec>::vector_t ret;
  if(nbvInspection::nbvPlanner<stateVec>::dv_max_ == 0)
  {
    ROS_ERROR("Unable to perform planning. Parameter maximal acceleration is either missing or zero");
    return ret;
  }
  if(nbvInspection::nbvPlanner<stateVec>::dyaw_max_ == 0)
  {
    ROS_ERROR("Unable to perform planning. Parameter maximal yaw acceleration is either missing or zero");
    return ret;
  }
      
  assert(s.size()==8);
  stateVec ds;
  octomap::point3d origin;
  double stransl = sqrt(SQ(s[4])+SQ(s[5])+SQ(s[6]));
  if(stransl>SQ(nbvInspection::nbvPlanner<stateVec>::v_max_))
    for(int i = 4; i<7; i++)
      s[i] *= nbvInspection::nbvPlanner<stateVec>::v_max_/stransl;
      
  octomap::point3d direction;
  octomap::point3d end;
  bool ignoreUnknownCells = false; // TODO: shoud be false, but free cells are not mapped at this time
  double d = DBL_MAX;
  for(int i = 0; i < 10; i++)
  {
    origin.x() = s[0];
    origin.y() = s[1];
    origin.z() = s[2];
    do
    {
      // translational
      do
      {
        for(int i = 4; i<ds.size()-1; i++)
          ds[i] = nbvInspection::nbvPlanner<stateVec>::v_max_*2.0*(((double)rand())/((double)RAND_MAX)-0.5);
      }while(nbvInspection::nbvPlanner<stateVec>::dv_max_<sqrt(SQ(ds[4])+SQ(ds[5])+SQ(ds[6]))); // assure uniform sampling in sphere
      ds[0] = s[4] + nbvInspection::nbvPlanner<stateVec>::dt_*ds[4];
      ds[1] = s[5] + nbvInspection::nbvPlanner<stateVec>::dt_*ds[5];
      ds[2] = s[6] + nbvInspection::nbvPlanner<stateVec>::dt_*ds[6];
      double transls = sqrt(SQ(ds[0])+SQ(ds[1])+SQ(ds[2]));
      if(transls>nbvInspection::nbvPlanner<stateVec>::v_max) // limit speed to sMax;
        for(int i = 0; i<3; i++)
          ds[i] *= nbvInspection::nbvPlanner<stateVec>::v_max_/transls;
      // rotational
      ds[7] = nbvInspection::nbvPlanner<stateVec>::ddyaw_max_*2.0*(((double)rand())/((double)RAND_MAX)-0.5);
      ds[3] = s[7] + nbvInspection::nbvPlanner<stateVec>::dt_*ds[7];
      if(abs(ds[3])>nbvInspection::nbvPlanner<stateVec>::dyaw_max_)
        ds[3] *= nbvInspection::nbvPlanner<stateVec>::dyaw_max_/ds[3];
      
      direction.x() = ds[0];
      direction.y() = ds[1];
      direction.z() = ds[2];
    }while(this->octomap_->castRay(origin, direction, end, ignoreUnknownCells, d*1.1+octomap_->getResolution()));
    s[0] += nbvInspection::nbvPlanner<stateVec>::dt_*(s[4]+ds[0])/2.0;
    s[1] += nbvInspection::nbvPlanner<stateVec>::dt_*(s[5]+ds[1])/2.0;
    s[2] += nbvInspection::nbvPlanner<stateVec>::dt_*(s[6]+ds[2])/2.0;
    s[3] += nbvInspection::nbvPlanner<stateVec>::dt_*(s[7]+ds[3])/2.0;
    s[4] += nbvInspection::nbvPlanner<stateVec>::dt_*ds[4];
    s[5] += nbvInspection::nbvPlanner<stateVec>::dt_*ds[5];
    s[6] += nbvInspection::nbvPlanner<stateVec>::dt_*ds[6];
    s[7] += nbvInspection::nbvPlanner<stateVec>::dt_*ds[7];
    ret.push_back(s);
  }
  std::reverse(ret.begin(), ret.end());
    
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID;
  p.header.frame_id = "/world";
  p.id = g_ID; g_ID++;
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = ret.back()[0];
  p.pose.position.y = ret.back()[1];
  p.pose.position.z = ret.back()[2];
  tf::Quaternion quat; quat.setEuler(0.0, 0.0, ret.back()[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = 1.0;
  p.scale.y = 0.1;
  p.scale.z = 0.1;
  p.color.r = 167.0/255.0;
  p.color.g = 167.0/255.0;
  p.color.b = 0.0;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(0.0);
  p.frame_locked = false;
  inspectionPath.publish(p);
  return ret;
}

template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::informationGainRand(stateVec s)
{
  return ((double)rand())/((double)RAND_MAX);
}

template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::informationGainSimple(stateVec s)
{
  double gain = 0.0;
  double R = nbvInspection::nbvPlanner<stateVec>::informationGainRange_;
  double disc = octomap_->getResolution();
  octomath::Vector3 origin;
  origin.x() = s[0]; origin.y() = s[1]; origin.z() = s[2];
  bool ignoreUnknownCells = true;
  octomath::Vector3 vec;
  for(vec.x() = std::max(s[0] - R, nbvInspection::nbvPlanner<stateVec>::minX_);
      vec.x() < std::min(s[0] + R, nbvInspection::nbvPlanner<stateVec>::maxX_); vec.x() += disc)
  {
    for(vec.y() = std::max(s[1] - R, nbvInspection::nbvPlanner<stateVec>::minY_);
        vec.y() < std::min(s[1] + R, nbvInspection::nbvPlanner<stateVec>::maxY_); vec.y() += disc)
    {
      for(vec.z() = std::max(s[2] - R, nbvInspection::nbvPlanner<stateVec>::minZ_);
          vec.z() < std::min(s[2] + R, nbvInspection::nbvPlanner<stateVec>::maxZ_); vec.z() += disc)
      {
        double dsq = SQ(s[0] - vec.x())+SQ(s[1] - vec.y())+SQ(s[2] - vec.z());
        if(dsq>pow(R,2.0))// || !octomap->inBBX(vec))
          continue;
        octomap::OcTreeNode * node = octomap_->search(vec.x(), vec.y(), vec.z());
        //ROS_INFO("node: %i", (int)(long)node);
        if (node == NULL)
        {
          // Rayshooting to evaluate inspectability of cell
          octomath::Vector3 end;
          if(this->octomap_->castRay(origin, vec - origin, end, ignoreUnknownCells, sqrt(dsq)))
            gain+=nbvInspection::nbvPlanner<stateVec>::igUnmapped_;
        }
        else
        {
          if(octomap_->isNodeOccupied(node))
          { 
            // Rayshooting to evaluate inspectability of cell
            octomath::Vector3 end;
            if(!this->octomap_->castRay(origin, vec - origin, end, ignoreUnknownCells, sqrt(dsq)))
              gain+=nbvInspection::nbvPlanner<stateVec>::igOccupied_;
          }
          else
          { 
            // Rayshooting to evaluate inspectability of cell
            octomath::Vector3 end;
            if(!this->octomap_->castRay(origin, vec - origin, end, ignoreUnknownCells, sqrt(dsq)))
              gain+=nbvInspection::nbvPlanner<stateVec>::igFree_;
          }
        }
      }
    }
  }
  gain *= pow(disc, 3.0);
  
  return gain;
}

template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::informationGainCone(stateVec s)
{
  double gain = 0.0;
  double R = nbvInspection::nbvPlanner<stateVec>::informationGainRange_;
  double disc = octomap_->getResolution();
  octomath::Vector3 origin;
  origin.x() = s[0]; origin.y() = s[1]; origin.z() = s[2];
  bool ignoreUnknownCells = true;
  octomath::Vector3 vec;
  for(vec.x() = std::max(s[0] - R, nbvInspection::nbvPlanner<stateVec>::minX_);
      vec.x() < std::min(s[0] + R, nbvInspection::nbvPlanner<stateVec>::maxX_); vec.x() += disc)
  {
    for(vec.y() = std::max(s[1] - R, nbvInspection::nbvPlanner<stateVec>::minY_);
        vec.y() < std::min(s[1] + R, nbvInspection::nbvPlanner<stateVec>::maxY_); vec.y() += disc)
    {
      for(vec.z() = std::max(s[2] - R, nbvInspection::nbvPlanner<stateVec>::minZ_);
          vec.z() < std::min(s[2] + R, nbvInspection::nbvPlanner<stateVec>::maxZ_); vec.z() += disc)
      {
        double dsq = SQ(s[0] - vec.x())+SQ(s[1] - vec.y())+SQ(s[2] - vec.z());
        if(dsq>pow(R,2.0))
          continue;
          
        Vector3f dir(vec.x() - s[0], vec.y() - s[1], vec.z() - s[2]);
        bool bbreak = false;
        for(typename std::vector<Vector3f>::iterator itCBN = camBoundNormals_.begin(); itCBN!=camBoundNormals_.end(); itCBN++)
        {
          Vector3f normal = AngleAxisf(s[3], Vector3f::UnitZ())*(*itCBN);
          double val = dir.dot(normal);
          if(val<SQRT2*disc)
          {
            bbreak = true;
            break;
          }
        }
        if(bbreak)
          continue;
        
        octomap::OcTreeNode * node = octomap_->search(vec.x(), vec.y(), vec.z());
        //ROS_INFO("node: %i", (int)(long)node);
        if (node == NULL)
        {
          // Rayshooting to evaluate inspectability of cell
          octomath::Vector3 end;
          if(!this->octomap_->castRay(origin, vec - origin, end, ignoreUnknownCells, sqrt(dsq)))
            gain+=nbvInspection::nbvPlanner<stateVec>::igUnmapped_;
        }
        else
        {
          if(octomap_->isNodeOccupied(node))
          { 
            // Rayshooting to evaluate inspectability of cell
            octomath::Vector3 end;
            if(!this->octomap_->castRay(origin, vec - origin, end, ignoreUnknownCells, sqrt(dsq)))
              gain+=nbvInspection::nbvPlanner<stateVec>::igOccupied_;
          }
          else
          { 
            // Rayshooting to evaluate inspectability of cell
            octomath::Vector3 end;
            if(!this->octomap_->castRay(origin, vec - origin, end, ignoreUnknownCells, sqrt(dsq)))
              gain+=nbvInspection::nbvPlanner<stateVec>::igFree_;
          }
        }
      }
    }
  }
  gain *= pow(disc, 3.0);
  
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID;
  p.header.frame_id = "/world";
  p.id = 0;
  p.ns="workspace";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = (minX_+maxX_)/2.0;
  p.pose.position.y = (minY_+maxY_)/2.0;
  p.pose.position.z = (minZ_+maxZ_)/2.0;
  tf::Quaternion quat; quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = nbvInspection::nbvPlanner<stateVec>::maxX_ - nbvInspection::nbvPlanner<stateVec>::minX_;
  p.scale.y = nbvInspection::nbvPlanner<stateVec>::maxY_ - nbvInspection::nbvPlanner<stateVec>::minY_;
  p.scale.z = nbvInspection::nbvPlanner<stateVec>::maxZ_ - nbvInspection::nbvPlanner<stateVec>::minZ_;
  p.color.r = 200.0/255.0;
  p.color.g = 100.0/255.0;
  p.color.b = 0.0;
  p.color.a = 0.1;
  p.lifetime = ros::Duration(0.0);
  p.frame_locked = false;
  inspectionPath.publish(p);
  
  return gain;
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::castRay(octomath::Vector3 origin, octomath::Vector3 direction, octomath::Vector3& end, bool ignoreUnknownCells, double d)
{
  static const double Radius = 0.5;
  bool ignoreUnknownCellsLocal = ignoreUnknownCells;
  d += Radius + this->octomap_->getResolution();
  bool rc = this->octomap_->castRay(origin, direction, end, ignoreUnknownCellsLocal, d);
  double d_real = sqrt(SQ(end.x() - origin.x()) +
  										 SQ(end.y() - origin.y()) +
  										 SQ(end.z() - origin.z()));
  if(rc || d > d_real)
    return true;
  Eigen::Vector3f q(1.0, 1.0, 1.0);
  if(direction.x() != 0.0)
    q[0] = -(direction.y()+direction.z())/direction.x();
  else if(direction.y() != 0.0)
    q[1] = -(direction.x()+direction.z())/direction.y();
  else
    q[2] = -(direction.y()+direction.x())/direction.z();
  q.normalize();
  Eigen::Vector3f dir(direction.x(),direction.y(),direction.z());
  dir.normalize();
  for(double i = 0; i<2*M_PI; i+=M_PI/6.0)
  {
    ignoreUnknownCellsLocal = ignoreUnknownCells;
    AngleAxisf rot = AngleAxisf(i, dir);
    Eigen::Vector3f qi = rot*q;
    octomath::Vector3 origini = origin;
    origini.x()+=qi[0]*Radius;
    origini.y()+=qi[1]*Radius;
    origini.z()+=qi[2]*Radius;
		rc = this->octomap_->castRay(origini, direction, end, ignoreUnknownCellsLocal, d);
		d_real = sqrt(SQ(end.x() - origini.x()) +
							 	  SQ(end.y() - origini.y()) +
						  	  SQ(end.z() - origini.z()));
		if(rc || d > d_real)
		  return true;
  }
  return false;
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::setParams()
{
  std::string ns = ros::this_node::getName();
  bool ret = true;
  if(!ros::param::get(ns+"/system/v_max", v_max_))
  {
    ROS_WARN("No maximal system speed specified. Looking for %s", (ns+"/system/v_max").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/system/dyaw_max", dyaw_max_))
  {
    ROS_WARN("No maximal yaw speed specified. Looking for %s", (ns+"/system/yaw_max").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/system/dv_max", dv_max_))
  {
    ROS_WARN("No maximal system acceleration specified (node: only used for euler integration tree extension). Looking for %s", (ns+"/system/v_max").c_str());
  }
  if(!ros::param::get(ns+"/system/ddyaw_max", ddyaw_max_))
  {
    ROS_WARN("No maximal yaw acceleration specified (node: only used for euler integration tree extension). Looking for %s", (ns+"/system/yaw_max").c_str());
  }
  if(!ros::param::get(ns+"/system/camera/pitch", camPitch_))
  {
    ROS_WARN("No camera pitch specified. Looking for %s", (ns+"/system/camera/pitch").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/system/camera/horizontal", camHorizontal_))
  {
    ROS_WARN("No camera horizontal opening specified. Looking for %s", (ns+"/system/camera/horizontal").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/system/camera/vertical", camVertical_))
  {
    ROS_WARN("No camera vertical opening specified. Looking for %s", (ns+"/system/camera/vertical").c_str());
    ret = false;
  }
  
  if(!ros::param::get(ns+"/nbvp/information_gain/free", igFree_))
  {
    ROS_WARN("No information gain for free cells specified. Looking for %s", (ns+"/nbvp/information_gain/free").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/nbvp/information_gain/occupied", igOccupied_))
  {
    ROS_WARN("No information gain for occupied cells specified. Looking for %s", (ns+"/nbvp/information_gain/occupied").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/nbvp/information_gain/unmapped", igUnmapped_))
  {
    ROS_WARN("No information gain for unmapped cells specified. Looking for %s", (ns+"/nbvp/information_gain/unmapped").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/nbvp/sampleHolonomic/degressive_coeff", degressiveCoeff_))
  {
    ROS_WARN("No degressive factor for information gain accumulation specified. Looking for %s", (ns+"/nbvp/sampleHolonomic/degressive_coeff").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/nbvp/sampleHolonomic/extension_range", extensionRange_))
  {
    ROS_WARN("No value for maximal extension range specified (note: only needed for holonomic extension of tree). Looking for %s", (ns+"/nbvp/sampleHolonomic/extension_range").c_str());
  }
  if(!ros::param::get(ns+"/nbvp/RRT/initial_iterations", initIterations_))
  {
    ROS_WARN("No number of initial RRT iterations specified (note: only needed when RRT tree method is used). Looking for %s", (ns+"/nbvp/RRT/initial_iterations").c_str());
  }
  if(!ros::param::get(ns+"/nbvp/dt", dt_))
  {
    ROS_WARN("No time step specified. Looking for %s", (ns+"/nbvp/dt").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/nbvp/RRT_extension", RRTextension_))
  {
    ROS_WARN("No extension method specified. Looking for %s", (ns+"/nbvp/RRT_extension").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/nbvp/information_gain/range", informationGainRange_))
  {
    ROS_WARN("No information gain range specified. Looking for %s", (ns+"/nbvp/information_gain/range").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/bbx/minX", minX_))
  {
    ROS_WARN("No x-min value specified. Looking for %s", (ns+"/bbx/minX").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/bbx/minY", minY_))
  {
    ROS_WARN("No y-min value specified. Looking for %s", (ns+"/bbx/minY").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/bbx/minZ", minZ_))
  {
    ROS_WARN("No z-min value specified. Looking for %s", (ns+"/bbx/minZ").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/bbx/maxX", maxX_))
  {
    ROS_WARN("No x-max value specified. Looking for %s", (ns+"/bbx/maxX").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/bbx/maxY", maxY_))
  {
    ROS_WARN("No y-max value specified. Looking for %s", (ns+"/bbx/maxY").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/bbx/maxZ", maxZ_))
  {
    ROS_WARN("No z-max value specified. Looking for %s", (ns+"/bbx/maxZ").c_str());
    ret = false;
  }
  return ret;
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::getRRTextension()
{
  return nbvInspection::nbvPlanner<stateVec>::RRTextension_;
}

template<typename stateVec>
int nbvInspection::nbvPlanner<stateVec>::getInitIterations()
{
  return nbvInspection::nbvPlanner<stateVec>::initIterations_;
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::extensionRangeSet()
{
  return nbvInspection::nbvPlanner<stateVec>::extensionRange_ != 0.0;
}

template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::v_max_ = 1.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::dyaw_max_ = 1.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::dv_max_ = 0.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::ddyaw_max_ = 0.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::camPitch_ = 0.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::camHorizontal_ = 100.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::camVertical_ = 100.0;
    
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::igFree_ = 0.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::igOccupied_ = 0.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::igUnmapped_ = 1.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::informationGainRange_ = 1.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::degressiveCoeff_ = 1.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::extensionRange_ = 0.0;
template<typename stateVec>
int nbvInspection::nbvPlanner<stateVec>::initIterations_ = 0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::dt_ = 1.0;
template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::RRTextension_ = true;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::minX_ = 0.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::minY_ = 0.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::minZ_ = 0.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::maxX_ = 0.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::maxY_ = 0.0;
template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::maxZ_ = 0.0;
