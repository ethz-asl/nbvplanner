#include "nbvPlanner/nbvp.h"
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
  parent = NULL;
  informationGain = 0.0;
  counter++;
}
    
template<typename stateVec>
nbvInspection::Node<stateVec>::~Node()
{
  for(typename std::vector<Node<stateVec>*>::iterator it = children.begin(); it != children.end(); it++)
  {
    delete (*it);
    (*it) = NULL;
  }
  counter--;
}
    
template<typename stateVec>
nbvInspection::Node<stateVec>* nbvInspection::Node<stateVec>::minDist(stateVec s)
{
  double bestDist = sqrt(SQ(s[0] - this->state[0]) + SQ(s[1] - this->state[1]) + SQ(s[2] - this->state[2]));
  nbvInspection::Node<stateVec>* ret = this;
  for(typename std::vector<nbvInspection::Node<stateVec>*>::iterator it = this->children.begin(); it != this->children.end(); it++)
  {
    nbvInspection::Node<stateVec>* tmp = (*it)->minDist(s);
    double tmpDist = sqrt(SQ(s[0] - tmp->state[0]) + SQ(s[1] - tmp->state[1]) + SQ(s[2] - tmp->state[2]));
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
  return nbvInspection::Node<stateVec>::counter;
}

template<typename stateVec>
void nbvInspection::Node<stateVec>::printToFile(std::fstream& file)
{
  if(this->parent)
  {
    for(int i = 0; i<this->state.size(); i++)
      file<<this->state[i]<<", ";
    for(int i = 0; i<this->parent->state.size()-1; i++)
      file<<this->parent->state[i]<<", ";
    file<<this->parent->state[this->parent->state.size()-1]<<";\n";
  }
  for(typename std::vector<nbvInspection::Node<stateVec>*>::iterator it = this->children.begin(); it != this->children.end(); it++)
  {
    (*it)->printToFile(file);
  }
}

template<typename stateVec>
const double nbvInspection::Node<stateVec>::ZERO_INFORMATION_GAIN = 0.0;
template<typename stateVec>
double nbvInspection::Node<stateVec>::bestInformationGain = nbvInspection::Node<stateVec>::ZERO_INFORMATION_GAIN;
template<typename stateVec>
nbvInspection::Node<stateVec>* nbvInspection::Node<stateVec>::bestNode = NULL;
template<typename stateVec>
int nbvInspection::Node<stateVec>::counter = 0;

template<typename stateVec>
nbvInspection::nbvplanner<stateVec>::nbvplanner()
{
  double pitch = M_PI*nbvInspection::nbvplanner<stateVec>::camPitch/180.0;
  double camTop = M_PI*(pitch-nbvInspection::nbvplanner<stateVec>::camVertical/2.0)/180.0;
  double camBottom = M_PI*(pitch+nbvInspection::nbvplanner<stateVec>::camVertical/2.0)/180.0;
  double side = M_PI*(nbvInspection::nbvplanner<stateVec>::camHorizontal)/360.0;
  Vector3f bottom(cos(camBottom), -sin(camBottom), 0.0);
  Vector3f top(cos(camTop), -sin(camTop), 0.0);
  Vector3f right(cos(side), -sin(camBottom), 0.0);
  Vector3f left(cos(side), sin(camBottom), 0.0);
  AngleAxisf m = AngleAxisf(pitch, Vector3f::UnitY());
  Vector3f rightR = m*right;
  Vector3f leftR = m*left;
  rightR.normalize();
  leftR.normalize();
  camBoundNormals.push_back(bottom);
  camBoundNormals.push_back(top);
  camBoundNormals.push_back(rightR);
  camBoundNormals.push_back(leftR);
  
  rootNode = NULL;
  octomap = NULL;
}

template<typename stateVec>
nbvInspection::nbvplanner<stateVec>::~nbvplanner()
{
  delete rootNode;
  rootNode = NULL;
  delete octomap;
  octomap = NULL;
}

template<typename stateVec>
typename nbvInspection::nbvplanner<stateVec>::vector_t nbvInspection::nbvplanner<stateVec>::expand(nbvplanner<stateVec>& instance, int N, int M, nbvInspection::nbvplanner<stateVec>::vector_t s, double& IGout, nbvInspection::nbvplanner<stateVec>::vector_t (nbvInspection::nbvplanner<stateVec>::*sample)(stateVec), double (nbvInspection::nbvplanner<stateVec>::*informationGain)(stateVec))
{
  double IG = (instance.*informationGain)(s.front());
  double IGnew = 0.0;
  nbvInspection::nbvplanner<stateVec>::vector_t path;
  nbvInspection::nbvplanner<stateVec>::vector_t ret;
  ret = s;
  IGout = 0.0;
  
  if(N<=0)
    return ret;
    
  for(int m = 0; m<M; m++)
  {
    path = instance.expand(instance, N-1, M, (instance.*sample)(s.front()), IGnew, sample, informationGain);
    if(IG + nbvInspection::nbvplanner<stateVec>::degressiveCoeff * IGnew > IGout)
    {
      path.insert(path.end(),s.begin(),s.end());
      ret = path;
      IGout = IG + nbvInspection::nbvplanner<stateVec>::degressiveCoeff * IGnew;
    }
  }
  //ROS_INFO("Information Gain is: %2.2f", IGout);
  return ret;
}


template<typename stateVec>
typename nbvInspection::nbvplanner<stateVec>::vector_t nbvInspection::nbvplanner<stateVec>::expandStructured(nbvInspection::nbvplanner<stateVec>& instance, int I, stateVec s, double& IGout, double (nbvInspection::nbvplanner<stateVec>::*informationGain)(stateVec))
{
  assert(s.size() == 4);
  nbvInspection::nbvplanner<stateVec>::vector_t ret;
  if(!this->rootNode)
  {
    this->rootNode = new nbvInspection::Node<stateVec>;
    this->rootNode->state = s;
  }
  // iterate as long as no information is found
  while(nbvInspection::Node<stateVec>::bestInformationGain <= nbvInspection::Node<stateVec>::ZERO_INFORMATION_GAIN || nbvInspection::Node<stateVec>::getCounter() < I)
  {
    if(nbvInspection::Node<stateVec>::getCounter()>10000)
    {
      ros::shutdown();
      return ret;
    }
    // set up boundaries: increase size as number of iterations grows
    double radius = 5.0*log(1.0+(double)nbvInspection::Node<stateVec>::getCounter());
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
    nbvInspection::Node<stateVec>* newParent = this->rootNode->minDist(newState);
    
    // check for collision
    octomap::point3d origin;
    origin.x() = newParent->state[0];
    origin.y() = newParent->state[1];
    origin.z() = newParent->state[2];
    octomap::point3d direction;
    direction.x() = newState[0];
    direction.y() = newState[1];
    direction.z() = newState[2];
    direction -= origin;
    octomap::point3d end;
    double d = sqrt(SQ(newState[0] - newParent->state[0]) + SQ(newState[1] - newParent->state[1]) + SQ(newState[2] - newParent->state[2]));
    bool ignoreUnknownCells = true; // TODO: shoud be false, but free cells are not mapped at this time
    if(!this->castRay(origin, direction, end, ignoreUnknownCells, d))
    {
      // sample the new orientation from the set of possible orientations
      newState[3] = newParent->state[3] + 2.0 * (((double)rand())/((double)RAND_MAX)-0.5) * d * nbvInspection::nbvplanner<stateVec>::dyaw_max / nbvInspection::nbvplanner<stateVec>::v_max;
      // create new node and inser into tree
      nbvInspection::Node<stateVec>* newNode = new nbvInspection::Node<stateVec>;
      newNode->state = newState;
      newNode->parent = newParent;
      newParent->children.push_back(newNode);
      newNode->informationGain = newParent->informationGain + (instance.*informationGain)(newNode->state);
      // display new node
      
      visualization_msgs::Marker p;
      p.header.stamp = ros::Time::now();
      p.header.seq = g_ID;
      p.header.frame_id = "/world";
      p.id = g_ID; g_ID++;
      p.ns="vp_tree";
      p.type = visualization_msgs::Marker::ARROW;
      p.action = visualization_msgs::Marker::ADD;
      p.pose.position.x = newNode->state[0];
      p.pose.position.y = newNode->state[1];
      p.pose.position.z = newNode->state[2];
      tf::Quaternion quat; quat.setEuler(0.0, 0.0, newNode->state[3]);
      p.pose.orientation.x = quat.x();
      p.pose.orientation.y = quat.y();
      p.pose.orientation.z = quat.z();
      p.pose.orientation.w = quat.w();
      p.scale.x = std::max(newNode->informationGain/2000.0, 0.05);
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
      p.pose.position.x = newNode->parent->state[0];
      p.pose.position.y = newNode->parent->state[1];
      p.pose.position.z = newNode->parent->state[2];
      Eigen::Quaternion<float> q;
      Eigen::Vector3f init(1.0, 0.0, 0.0);
      Eigen::Vector3f dir(newNode->state[0] - newNode->parent->state[0], newNode->state[1] - newNode->parent->state[1], newNode->state[2] - newNode->parent->state[2]);
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
      if(newNode->informationGain > nbvInspection::Node<stateVec>::bestInformationGain)
      {
        nbvInspection::Node<stateVec>::bestInformationGain = newNode->informationGain;
        nbvInspection::Node<stateVec>::bestNode = newNode;
        
      }
    }
    
  }
  // extract best path
  nbvInspection::Node<stateVec>* curr = nbvInspection::Node<stateVec>::bestNode;
  if(curr->parent != NULL)
  {
    while(curr->parent != this->rootNode && curr->parent != NULL)
    {
      curr = curr->parent;
    }
    
    double d = SQ(curr->state[0] - curr->parent->state[0]) + SQ(curr->state[1] - curr->parent->state[1]) + SQ(curr->state[2] - curr->parent->state[2]);
    d = sqrt(d);
    double disc = nbvInspection::nbvplanner<stateVec>::dt * nbvInspection::nbvplanner<stateVec>::v_max / d;
    for(double it = 0.0; it < 1.0; it += disc)
    {
      ret.push_back((1.0-it)*curr->state + it*curr->parent->state);
    }
  }
  else
    ret.push_back(curr->state);
  IGout = nbvInspection::Node<stateVec>::bestInformationGain;
  
  return ret;
}

template<typename stateVec>
typename nbvInspection::nbvplanner<stateVec>::vector_t nbvInspection::nbvplanner<stateVec>::sampleHolonomic(stateVec s)
{
  assert(s.size()==4);
  nbvInspection::nbvplanner<stateVec>::vector_t ret;
  stateVec extension;
  octomap::point3d origin;
  origin.x() = s[0];
  origin.y() = s[1];
  origin.z() = s[2];
  octomap::point3d direction;
  octomap::point3d end;
  bool ignoreUnknownCells = true; // TODO: shoud be false, but free cells are not mapped at this time
  double d = DBL_MAX;
  int iter = 0;
  do
  {
    for(int i = 0; i<extension.size()-1; i++)
      extension[i] = nbvInspection::nbvplanner<stateVec>::extensionRange * (((double)rand())/((double)RAND_MAX)-0.5);
    d = sqrt(SQ(extension[0])+SQ(extension[1])+SQ(extension[2]));
    // sample yaw w.r.t. the constraints
    extension[extension.size()-1] = (nbvInspection::nbvplanner<stateVec>::dyaw_max / nbvInspection::nbvplanner<stateVec>::v_max) * d * (((double)rand())/((double)RAND_MAX)-0.5);
    direction.x() = extension[0];
    direction.y() = extension[1];
    direction.z() = extension[2];
  }while(this->octomap->castRay(origin, direction, end, ignoreUnknownCells, d*1.1+this->octomap->getResolution()) && (iter++) < 100);
  if(iter>=100)
  {
    //ROS_WARN("No connection found to extend tree");
    ret.push_back(s);
    return ret;
  }
  double wp = d / (nbvInspection::nbvplanner<stateVec>::v_max * nbvInspection::nbvplanner<stateVec>::dt);
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
typename nbvInspection::nbvplanner<stateVec>::vector_t nbvInspection::nbvplanner<stateVec>::sampleEuler(stateVec s)
{
  nbvInspection::nbvplanner<stateVec>::vector_t ret;
  if(nbvInspection::nbvplanner<stateVec>::dv_max == 0)
  {
    ROS_ERROR("Unable to perform planning. Parameter maximal acceleration is either missing or zero");
    return ret;
  }
  if(nbvInspection::nbvplanner<stateVec>::dyaw_max == 0)
  {
    ROS_ERROR("Unable to perform planning. Parameter maximal yaw acceleration is either missing or zero");
    return ret;
  }
      
  assert(s.size()==8);
  stateVec ds;
  octomap::point3d origin;
  double stransl = sqrt(SQ(s[4])+SQ(s[5])+SQ(s[6]));
  if(stransl>SQ(nbvInspection::nbvplanner<stateVec>::v_max))
    for(int i = 4; i<7; i++)
      s[i] *= nbvInspection::nbvplanner<stateVec>::v_max/stransl;
      
  octomap::point3d direction;
  octomap::point3d end;
  bool ignoreUnknownCells = true; // TODO: shoud be false, but free cells are not mapped at this time
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
          ds[i] = nbvInspection::nbvplanner<stateVec>::v_max*2.0*(((double)rand())/((double)RAND_MAX)-0.5);
      }while(nbvInspection::nbvplanner<stateVec>::dv_max<sqrt(SQ(ds[4])+SQ(ds[5])+SQ(ds[6]))); // assure uniform sampling in sphere
      ds[0] = s[4] + nbvInspection::nbvplanner<stateVec>::dt*ds[4];
      ds[1] = s[5] + nbvInspection::nbvplanner<stateVec>::dt*ds[5];
      ds[2] = s[6] + nbvInspection::nbvplanner<stateVec>::dt*ds[6];
      double transls = sqrt(SQ(ds[0])+SQ(ds[1])+SQ(ds[2]));
      if(transls>nbvInspection::nbvplanner<stateVec>::v_max) // limit speed to sMax;
        for(int i = 0; i<3; i++)
          ds[i] *= nbvInspection::nbvplanner<stateVec>::v_max/transls;
      // rotational
      ds[7] = nbvInspection::nbvplanner<stateVec>::ddyaw_max*2.0*(((double)rand())/((double)RAND_MAX)-0.5);
      ds[3] = s[7] + nbvInspection::nbvplanner<stateVec>::dt*ds[7];
      if(abs(ds[3])>nbvInspection::nbvplanner<stateVec>::dyaw_max)
        ds[3] *= nbvInspection::nbvplanner<stateVec>::dyaw_max/ds[3];
      
      direction.x() = ds[0];
      direction.y() = ds[1];
      direction.z() = ds[2];
    }while(false);//this->octomap->castRay(origin, direction, end, ignoreUnknownCells, d*1.1+octomap->getResolution()));
    s[0] += nbvInspection::nbvplanner<stateVec>::dt*(s[4]+ds[0])/2.0;
    s[1] += nbvInspection::nbvplanner<stateVec>::dt*(s[5]+ds[1])/2.0;
    s[2] += nbvInspection::nbvplanner<stateVec>::dt*(s[6]+ds[2])/2.0;
    s[3] += nbvInspection::nbvplanner<stateVec>::dt*(s[7]+ds[3])/2.0;
    s[4] += nbvInspection::nbvplanner<stateVec>::dt*ds[4];
    s[5] += nbvInspection::nbvplanner<stateVec>::dt*ds[5];
    s[6] += nbvInspection::nbvplanner<stateVec>::dt*ds[6];
    s[7] += nbvInspection::nbvplanner<stateVec>::dt*ds[7];
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
double nbvInspection::nbvplanner<stateVec>::informationGainRand(stateVec s)
{
  return ((double)rand())/((double)RAND_MAX);
}

template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::informationGainSimple(stateVec s)
{
  double gain = 0.0;
  double R = nbvInspection::nbvplanner<stateVec>::informationGainRange;
  double disc = octomap->getResolution();
  octomath::Vector3 origin;
  origin.x() = s[0]; origin.y() = s[1]; origin.z() = s[2];
  bool ignoreUnknownCells = true;
  octomath::Vector3 vec;
  for(vec.x() = std::max(s[0] - R, nbvInspection::nbvplanner<stateVec>::minX);
      vec.x() < std::min(s[0] + R, nbvInspection::nbvplanner<stateVec>::maxX); vec.x() += disc)
  {
    for(vec.y() = std::max(s[1] - R, nbvInspection::nbvplanner<stateVec>::minY);
        vec.y() < std::min(s[1] + R, nbvInspection::nbvplanner<stateVec>::maxY); vec.y() += disc)
    {
      for(vec.z() = std::max(s[2] - R, nbvInspection::nbvplanner<stateVec>::minZ);
          vec.z() < std::min(s[2] + R, nbvInspection::nbvplanner<stateVec>::maxZ); vec.z() += disc)
      {
        double dsq = SQ(s[0] - vec.x())+SQ(s[1] - vec.y())+SQ(s[2] - vec.z());
        if(dsq>pow(R,2.0))// || !octomap->inBBX(vec))
          continue;
        octomap::OcTreeNode* node = octomap->search(vec.x(), vec.y(), vec.z());
        //ROS_INFO("node: %i", (int)(long)node);
        if (node == NULL)
        {
          // Rayshooting to evaluate inspectability of cell
          octomath::Vector3 end;
          if(this->octomap->castRay(origin, vec - origin, end, ignoreUnknownCells, sqrt(dsq)))
            gain+=nbvInspection::nbvplanner<stateVec>::igUnmapped;
        }
        else
        {
          if(node->getOccupancy())
          { 
            // Rayshooting to evaluate inspectability of cell
            octomath::Vector3 end;
            if(!this->octomap->castRay(origin, vec - origin, end, ignoreUnknownCells, sqrt(dsq)))
              gain+=nbvInspection::nbvplanner<stateVec>::igOccupied;
          }
          else
          { 
            // Rayshooting to evaluate inspectability of cell
            octomath::Vector3 end;
            if(!this->octomap->castRay(origin, vec - origin, end, ignoreUnknownCells, sqrt(dsq)))
              gain+=nbvInspection::nbvplanner<stateVec>::igFree;
          }
        }
      }
    }
  }
  gain*=pow(disc, 3.0);
  
  return gain;
}

template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::informationGainCone(stateVec s)
{
  double gain = 0.0;
  static const double R = nbvInspection::nbvplanner<stateVec>::informationGainRange;
  double disc = octomap->getResolution();
  octomath::Vector3 origin;
  origin.x() = s[0]; origin.y() = s[1]; origin.z() = s[2];
  bool ignoreUnknownCells = true;
  octomath::Vector3 vec;
  for(vec.x() = std::max(s[0] - R, nbvInspection::nbvplanner<stateVec>::minX);
      vec.x() < std::min(s[0] + R, nbvInspection::nbvplanner<stateVec>::maxX); vec.x() += disc)
  {
    for(vec.y() = std::max(s[1] - R, nbvInspection::nbvplanner<stateVec>::minY);
        vec.y() < std::min(s[1] + R, nbvInspection::nbvplanner<stateVec>::maxY); vec.y() += disc)
    {
      for(vec.z() = std::max(s[2] - R, nbvInspection::nbvplanner<stateVec>::minZ);
          vec.z() < std::min(s[2] + R, nbvInspection::nbvplanner<stateVec>::maxZ); vec.z() += disc)
      {
        double dsq = SQ(s[0] - vec.x())+SQ(s[1] - vec.y())+SQ(s[2] - vec.z());
        if(dsq>pow(R,2.0))
          continue;
          
        Vector3f dir(vec.x() - s[0], vec.y() - s[1], vec.z() - s[2]);
        bool bbreak = false;
        for(typename std::vector<Vector3f>::iterator itCBN = camBoundNormals.begin(); itCBN!=camBoundNormals.end(); itCBN++)
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
        
        octomap::OcTreeNode* node = octomap->search(vec.x(), vec.y(), vec.z());
        //ROS_INFO("node: %i", (int)(long)node);
        if (node == NULL)
        {
          // Rayshooting to evaluate inspectability of cell
          octomath::Vector3 end;
          if(!this->octomap->castRay(origin, vec - origin, end, ignoreUnknownCells, sqrt(dsq)))
            gain+=nbvInspection::nbvplanner<stateVec>::igUnmapped;
        }
        else
        {
          if(node->getOccupancy())
          { 
            // Rayshooting to evaluate inspectability of cell
            octomath::Vector3 end;
            if(!this->octomap->castRay(origin, vec - origin, end, ignoreUnknownCells, sqrt(dsq)))
              gain+=nbvInspection::nbvplanner<stateVec>::igOccupied;
          }
          else
          { 
            // Rayshooting to evaluate inspectability of cell
            octomath::Vector3 end;
            if(!this->octomap->castRay(origin, vec - origin, end, ignoreUnknownCells, sqrt(dsq)))
              gain+=nbvInspection::nbvplanner<stateVec>::igFree;
          }
        }
      }
    }
  }
  gain*=pow(disc, 3.0);
  
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID;
  p.header.frame_id = "/world";
  p.id = 0;
  p.ns="workspace";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = (minX+maxX)/2.0;
  p.pose.position.y = (minY+maxY)/2.0;
  p.pose.position.z = (minZ+maxZ)/2.0;
  tf::Quaternion quat; quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = nbvInspection::nbvplanner<stateVec>::maxX-nbvInspection::nbvplanner<stateVec>::minX;
  p.scale.y = nbvInspection::nbvplanner<stateVec>::maxY-nbvInspection::nbvplanner<stateVec>::minY;
  p.scale.z = nbvInspection::nbvplanner<stateVec>::maxZ-nbvInspection::nbvplanner<stateVec>::minZ;
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
bool nbvInspection::nbvplanner<stateVec>::castRay(octomath::Vector3 origin, octomath::Vector3 direction, octomath::Vector3& end, bool ignoreUnknownCells, double d)
{
  static const double Radius = 1.5;
  if(this->octomap->castRay(origin, direction, end, ignoreUnknownCells, d+Radius+this->octomap->getResolution()))
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
    AngleAxisf rot = AngleAxisf(i, dir);
    Eigen::Vector3f qi = rot*q;
    octomath::Vector3 origini = origin;
    origini.x()+=qi[0]*Radius;
    origini.y()+=qi[1]*Radius;
    origini.z()+=qi[2]*Radius;
    if(this->octomap->castRay(origini, direction, end, ignoreUnknownCells, d+Radius+this->octomap->getResolution()))
      return true;
  }
  return false;
}

template<typename stateVec>
bool nbvInspection::nbvplanner<stateVec>::setParams()
{
  std::string ns = ros::this_node::getName();
  bool ret = true;
  if(!ros::param::get(ns+"/system/v_max", v_max))
  {
    ROS_WARN("No maximal system speed specified. Looking for %s", (ns+"/system/v_max").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/system/dyaw_max", dyaw_max))
  {
    ROS_WARN("No maximal yaw speed specified. Looking for %s", (ns+"/system/yaw_max").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/system/dv_max", dv_max))
  {
    ROS_WARN("No maximal system acceleration specified (node: only used for euler integration tree extension). Looking for %s", (ns+"/system/v_max").c_str());
  }
  if(!ros::param::get(ns+"/system/ddyaw_max", ddyaw_max))
  {
    ROS_WARN("No maximal yaw acceleration specified (node: only used for euler integration tree extension). Looking for %s", (ns+"/system/yaw_max").c_str());
  }
  if(!ros::param::get(ns+"/system/camera/pitch", camPitch))
  {
    ROS_WARN("No camera pitch specified. Looking for %s", (ns+"/system/camera/pitch").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/system/camera/horizontal", camHorizontal))
  {
    ROS_WARN("No camera horizontal opening specified. Looking for %s", (ns+"/system/camera/horizontal").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/system/camera/vertical", camVertical))
  {
    ROS_WARN("No camera vertical opening specified. Looking for %s", (ns+"/system/camera/vertical").c_str());
    ret = false;
  }
  
  if(!ros::param::get(ns+"/nbvp/information_gain/free", igFree))
  {
    ROS_WARN("No information gain for free cells specified. Looking for %s", (ns+"/nbvp/information_gain/free").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/nbvp/information_gain/occupied", igOccupied))
  {
    ROS_WARN("No information gain for occupied cells specified. Looking for %s", (ns+"/nbvp/information_gain/occupied").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/nbvp/information_gain/unmapped", igUnmapped))
  {
    ROS_WARN("No information gain for unmapped cells specified. Looking for %s", (ns+"/nbvp/information_gain/unmapped").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/nbvp/sampleHolonomic/extension_range", degressiveCoeff))
  {
    ROS_WARN("No degressive factor for information gain accumulation specified. Looking for %s", (ns+"/nbvp/sampleHolonomic/extension_range").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/nbvp/sampleHolonomic/extension_range", extensionRange))
  {
    ROS_WARN("No value for maximal extension range specified (note: only needed for holonomic extension of tree). Looking for %s", (ns+"/nbvp/sampleHolonomic/extension_range").c_str());
  }
  if(!ros::param::get(ns+"/nbvp/RRT/initial_iterations", initIterations))
  {
    ROS_WARN("No number of initial RRT iterations specified (note: only needed when RRT tree method is used). Looking for %s", (ns+"/nbvp/RRT/initial_iterations").c_str());
  }
  if(!ros::param::get(ns+"/nbvp/dt", dt))
  {
    ROS_WARN("No time step specified. Looking for %s", (ns+"/nbvp/dt").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/nbvp/RRT_extension", RRTextension))
  {
    ROS_WARN("No extension method specified. Looking for %s", (ns+"/nbvp/RRT_extension").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/nbvp/information_gain/range", informationGainRange))
  {
    ROS_WARN("No information gain range specified. Looking for %s", (ns+"/nbvp/information_gain/range").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/bbx/minX", minX))
  {
    ROS_WARN("No x-min value specified. Looking for %s", (ns+"/bbx/minX").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/bbx/minY", minY))
  {
    ROS_WARN("No y-min value specified. Looking for %s", (ns+"/bbx/minY").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/bbx/minZ", minZ))
  {
    ROS_WARN("No z-min value specified. Looking for %s", (ns+"/bbx/minZ").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/bbx/maxX", maxX))
  {
    ROS_WARN("No x-max value specified. Looking for %s", (ns+"/bbx/maxX").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/bbx/maxY", maxY))
  {
    ROS_WARN("No y-max value specified. Looking for %s", (ns+"/bbx/maxY").c_str());
    ret = false;
  }
  if(!ros::param::get(ns+"/bbx/maxZ", maxZ))
  {
    ROS_WARN("No z-max value specified. Looking for %s", (ns+"/bbx/maxZ").c_str());
    ret = false;
  }
  return ret;
}

template<typename stateVec>
bool nbvInspection::nbvplanner<stateVec>::getRRTextension()
{
  return nbvInspection::nbvplanner<stateVec>::RRTextension;
}

template<typename stateVec>
int nbvInspection::nbvplanner<stateVec>::getInitIterations()
{
  return nbvInspection::nbvplanner<stateVec>::initIterations;
}

template<typename stateVec>
bool nbvInspection::nbvplanner<stateVec>::extensionRangeSet()
{
  return nbvInspection::nbvplanner<stateVec>::extensionRange != 0.0;
}

template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::v_max = 1.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::dyaw_max = 1.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::dv_max = 0.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::ddyaw_max = 0.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::camPitch = 0.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::camHorizontal = 100.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::camVertical = 100.0;
    
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::igFree = 0.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::igOccupied = 0.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::igUnmapped = 1.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::informationGainRange = 1.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::degressiveCoeff = 1.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::extensionRange = 0.0;
template<typename stateVec>
int nbvInspection::nbvplanner<stateVec>::initIterations = 0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::dt = 1.0;
template<typename stateVec>
bool nbvInspection::nbvplanner<stateVec>::RRTextension = true;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::minX = 0.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::minY = 0.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::minZ = 0.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::maxX = 0.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::maxY = 0.0;
template<typename stateVec>
double nbvInspection::nbvplanner<stateVec>::maxZ = 0.0;
