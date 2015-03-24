#include "nbvPlanner/nbvp.h"
#include <cfloat>
#include <cstdlib>
#include <sstream>
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"

using namespace Eigen;

extern int g_ID;
extern ros::Publisher inspectionPath;

template<typename stateVec>
nbvInspection::nbvplanner<stateVec>::nbvplanner()
{
  double pitch = M_PI*PITCH/180.0;
  double camTop = M_PI*(PITCH-CAM_VERTICAL/2.0)/180.0;
  double camBottom = M_PI*(PITCH+CAM_VERTICAL/2.0)/180.0;
  double side = M_PI*(CAM_HORIZONTAL)/360.0;
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
  
  octomap = NULL;
}

template<typename stateVec>
nbvInspection::nbvplanner<stateVec>::~nbvplanner()
{

}

template<typename stateVec>
typename nbvInspection::nbvplanner<stateVec>::vector_t nbvInspection::nbvplanner<stateVec>::expand(nbvplanner<stateVec>& instance, int N, int M, nbvInspection::nbvplanner<stateVec>::vector_t s, nbvInspection::nbvplanner<stateVec>::vector_t (nbvInspection::nbvplanner<stateVec>::*sample)(stateVec), double (nbvInspection::nbvplanner<stateVec>::*informationGain)(stateVec))
{
  double IG = (instance.*informationGain)(s.front());
  nbvInspection::nbvplanner<stateVec>::vector_t path;
  nbvInspection::nbvplanner<stateVec>::vector_t ret;
  ret = s;
  
  if(N<=0)
    return ret;
    
  for(int m = 0; m<M; m++)
  {
    path = instance.expand(instance, N-1, M, (instance.*sample)(s.front()), sample, informationGain);
    double IGtmp = (instance.*informationGain)(path.front());
    if(IGtmp>IG)
    {
      path.insert(path.end(),s.begin(),s.end());
      ret = path;
      IG = IGtmp;
    }
  }
  ROS_INFO("Information Gain is: %2.2f", IG);
  return ret;
}

template<typename stateVec>
typename nbvInspection::nbvplanner<stateVec>::vector_t nbvInspection::nbvplanner<stateVec>::sampleHolonomic(stateVec s)
{
  assert(s.size()==4);
  static const double dt = 0.5;
  nbvInspection::nbvplanner<stateVec>::vector_t ret;
  stateVec extension;
  octomap::point3d origin;
  origin.x() = s[0];
  origin.y() = s[1];
  origin.z() = s[2];
  octomap::point3d direction;
  octomap::point3d end;
  bool ignoreUnknownCells = true;
  double d = DBL_MAX;
  do
  {
    for(int i = 0; i<extension.size()-1; i++)
      extension[i] = EXTENSION_RANGE*(((double)rand())/((double)RAND_MAX)-0.5);
    d = sqrt(SQ(extension[0])+SQ(extension[1])+SQ(extension[2]));
    // sample yaw w.r.t. the constraints
    extension[extension.size()-1] = (((double)YAWMAX)/(double)VMAX)*d*(((double)rand())/((double)RAND_MAX)-0.5);
    direction.x() = extension[0];
    direction.y() = extension[1];
    direction.z() = extension[2];
  }while(this->octomap->castRay(origin, direction, end, ignoreUnknownCells, d*1.1+1.0));
  double wp = d/(VMAX*dt);
  for(double i = 0.0; i<wp; i+=1.0)
    ret.push_back(s+(1.0-i/wp)*extension);
  double IG = this->informationGainCone(s+extension);
  ROS_INFO("IG %2.2f", IG);
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
  p.scale.x = std::max(IG/100.0, 0.05);
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
typename nbvInspection::nbvplanner<stateVec>::vector_t nbvInspection::nbvplanner<stateVec>::sampleEuler(stateVec s)
{
  assert(s.size()==8);
  nbvInspection::nbvplanner<stateVec>::vector_t ret;
  static const double sMax = 0.25;
  static const double dsMax = 0.05;
  static const double syawMax = 0.5;
  static const double dsyawMax = 0.1;
  static const double dt = 0.5;
  stateVec ds;
  octomap::point3d origin;
  double stransl = sqrt(SQ(s[4])+SQ(s[5])+SQ(s[6]));
  if(stransl>SQ(sMax))
    for(int i = 4; i<7; i++)
      s[i] *= sMax/stransl;
      
  octomap::point3d direction;
  octomap::point3d end;
  bool ignoreUnknownCells = false;
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
          ds[i] = dsMax*2.0*(((double)rand())/((double)RAND_MAX)-0.5);
      }while(dsMax<sqrt(SQ(ds[4])+SQ(ds[5])+SQ(ds[6]))); // assure uniform sampling in sphere
      ds[0] = s[4] + dt*ds[4];
      ds[1] = s[5] + dt*ds[5];
      ds[2] = s[6] + dt*ds[6];
      double transls = sqrt(SQ(ds[0])+SQ(ds[1])+SQ(ds[2]));
      if(transls>sMax) // limit speed to sMax;
        for(int i = 0; i<3; i++)
          ds[i] *= sMax/transls;
      // rotational
      ds[7] = dsyawMax*2.0*(((double)rand())/((double)RAND_MAX)-0.5);
      ds[3] = s[7] + dt*ds[7];
      if(abs(ds[3])>syawMax)
        ds[3] *= syawMax/ds[3];
      
      direction.x() = ds[0];
      direction.y() = ds[1];
      direction.z() = ds[2];
    }while(false);//this->octomap->castRay(origin, direction, end, ignoreUnknownCells, d));
    s[0] += dt*(s[4]+ds[0])/2.0;
    s[1] += dt*(s[5]+ds[1])/2.0;
    s[2] += dt*(s[6]+ds[2])/2.0;
    s[3] += dt*(s[7]+ds[3])/2.0;
    s[4] += dt*ds[4];
    s[5] += dt*ds[5];
    s[6] += dt*ds[6];
    s[7] += dt*ds[7];
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
  static const double R = 10.0;
  static const double minX = -10.0;
  static const double minY = -100.0;
  static const double minZ = 0.0;
  static const double maxX = 100.0;
  static const double maxY = 10.0;
  static const double maxZ = 40.0;
  double gain = 0.0;
  double disc = octomap->getResolution();
  octomath::Vector3 origin;
  origin.x() = s[0]; origin.y() = s[1]; origin.z() = s[2];
  bool ignoreUnknownCells = true;
  octomath::Vector3 vec;
  for(vec.x() = std::max(s[0] - R, minX); vec.x() < std::min(s[0] + R, maxX); vec.x() += disc)
  {
    for(vec.y() = std::max(s[0] - R, minY); vec.y() < std::min(s[0] + R, maxY); vec.y() += disc)
    {
      for(vec.z() = std::max(s[0] - R, minZ); vec.z() < std::min(s[0] + R, maxZ); vec.z() += disc)
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
            gain+=1.0/dsq;
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
  static const double R = 10.0;
  static const double minX = -10.0;
  static const double minY = -100.0;
  static const double minZ = 0.0;
  static const double maxX = 100.0;
  static const double maxY = 10.0;
  static const double maxZ = 40.0;
  double gain = 0.0;
  double disc = octomap->getResolution();
  octomath::Vector3 origin;
  origin.x() = s[0]; origin.y() = s[1]; origin.z() = s[2];
  bool ignoreUnknownCells = false;
  octomath::Vector3 vec;
  for(vec.x() = std::max(s[0] - R, minX); vec.x() < std::min(s[0] + R, maxX); vec.x() += disc)
  {
    for(vec.y() = std::max(s[0] - R, minY); vec.y() < std::min(s[0] + R, maxY); vec.y() += disc)
    {
      for(vec.z() = std::max(s[0] - R, minZ); vec.z() < std::min(s[0] + R, maxZ); vec.z() += disc)
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
            gain+=1.0;// /dsq;
        }
      }
    }
  }
  gain*=pow(disc, 3.0);
  
  return gain;
}

