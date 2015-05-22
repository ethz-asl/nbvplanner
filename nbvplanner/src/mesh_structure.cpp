#ifndef _MESH_STRUCTURE_CPP_
#define _MESH_STRUCTURE_CPP_

#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <octomap_world/octomap_manager.h>
#include <nbvplanner/mesh_structure.h>

mesh::StlMesh::StlMesh()
    : isLeaf_(true),
      isHead_(true),
      isInspected_(false) {
}

mesh::StlMesh::StlMesh(const Eigen::Vector3d x1, const Eigen::Vector3d x2, const Eigen::Vector3d x3)
    : x1_(x1),
      x2_(x2),
      x3_(x3),
      isLeaf_(true),
      isHead_(true),
      isInspected_(false),
      normal_((x3 - x2).cross(x1 - x2) / 2.0) {
}

mesh::StlMesh::StlMesh(std::fstream& file)
    : isLeaf_(true),
      isHead_(true),
      isInspected_(false),
      x1_(0.0, 0.0, 0.0),
      x2_(0.0, 0.0, 0.0),
      x3_(0.0, 0.0, 0.0) {
  //ROS_INFO("Loading STL");
  assert(file.is_open());
  //ROS_INFO("Loading STL 1.1");
  int MaxLine = 0;
  char* line;
  //ROS_INFO("Loading STL 1.2");
  double maxX = -DBL_MAX;
  double maxY = -DBL_MAX;
  double maxZ = -DBL_MAX;
  double minX = DBL_MAX;
  double minY = DBL_MAX;
  double minZ = DBL_MAX;
  //ROS_INFO("Loading STL 1.3");
  assert(line = (char *) malloc(MaxLine = 80));
  //ROS_INFO("Loading STL 1.4");
  file.getline(line, MaxLine);
  //ROS_INFO("Loading STL 1.5");
  //ROS_INFO("Loading STL 1.5.1, %s", line);
  //ROS_INFO("Loading STL 1.5.2, %s", strtok(line, " "));
  if (0 != strcmp(strtok(line, " "), "solid")) {
    ROS_ERROR("Invalid mesh file! Make sure the STL file is given in ascii-format.");
    return;
  }
  //ROS_INFO("Loading STL 1.6");
  assert(line = (char *) realloc(line, MaxLine));
  //ROS_INFO("Loading STL 1.7");
  file.getline(line, MaxLine);
  //ROS_INFO("Loading STL 1.8");
  int k = 0;
  //ROS_INFO("Loading STL 1.9");
  while (0 != strcmp(strtok(line, " "), "endsolid") && !ros::isShuttingDown()) {
    int q = 0;
  //ROS_INFO("Loading STL 2");
    mesh::StlMesh* newNode = new mesh::StlMesh;
    newNode->isHead_ = true;
    std::vector<Eigen::Vector3d*> vertices;
    vertices.push_back(&(newNode->x1_));
    vertices.push_back(&(newNode->x2_));
    vertices.push_back(&(newNode->x3_));
    int vertexCount = 0;
    for (int i = 0; i<7; i++) {
  //ROS_INFO("Loading STL 3");
      while (line[q] == ' ')
        q++;
  //ROS_INFO("Loading STL 4");
      if (line[q] == 'v') {
  //ROS_INFO("Loading STL 5");
        // used to rotate the mesh before processing
        const double yawTrafo = 0.0;
        // used to scale the mesh before processing
        const double scaleFactor = 100.0;
        // used to offset the mesh before processing
        const double offsetX = 0.0;       
        // used to offset the mesh before processing
        const double offsetY = 0.0;
        // used to offset the mesh before processing
        const double offsetZ = 0.0;

  //ROS_INFO("Loading STL 6");
        char* v = strtok(line+q," ");
        v = strtok(NULL," ");
        double xtmp = atof(v)/scaleFactor;
        v = strtok(NULL," ");
        double ytmp = atof(v)/scaleFactor;
        vertices[vertexCount]->x() = cos(yawTrafo)*xtmp-sin(yawTrafo)*ytmp;
        vertices[vertexCount]->y() =  sin(yawTrafo)*xtmp+cos(yawTrafo)*ytmp;
        v = strtok(NULL," ");
        vertices[vertexCount]->z() =  atof(v)/scaleFactor;
        vertices[vertexCount]->x() -= offsetX;
        vertices[vertexCount]->y() -= offsetY;
        vertices[vertexCount]->z() -= offsetZ;
        
  //ROS_INFO("Loading STL 7");
        if (maxX<vertices[vertexCount]->x())
          maxX=vertices[vertexCount]->x();
        if (maxY<vertices[vertexCount]->y())
          maxY=vertices[vertexCount]->y();
        if (maxZ<vertices[vertexCount]->z())
          maxZ=vertices[vertexCount]->z();
        if (minX>vertices[vertexCount]->x())
          minX=vertices[vertexCount]->x();
        if (minY>vertices[vertexCount]->y())
          minY=vertices[vertexCount]->y();
        if (minZ>vertices[vertexCount]->z())
          minZ=vertices[vertexCount]->z();
          
        vertexCount++;
      }
  //ROS_INFO("Loading STL 8");
      assert(line = (char *) realloc(line, MaxLine));
      file.getline(line, MaxLine);
    }
    children_.push_back(newNode);
  //ROS_INFO("Loading STL 9");
    k++;
  }
  //ROS_INFO("Loading STL 10");
  free(line);
  file.close();
  if (k > 0)
    isLeaf_ = false;
  ROS_INFO("STL file read. Contains %i elements located in (%2.2f,%2.2f)x(%2.2f,%2.2f)x(%2.2f,%2.2f)",
           k, minX, maxX, minY, maxY, minZ, maxZ);
}

mesh::StlMesh::~StlMesh() {
  for (typename std::vector<StlMesh*>::iterator it = children_.begin();
       it != children_.end(); it++)
    delete (*it);
}

void mesh::StlMesh::setCameraParams(double cameraPitch, double cameraHorizontalFoV,
                                    double cameraVerticalFoV, double maxDist) {
    cameraPitch_ = cameraPitch;
    cameraHorizontalFoV_ = cameraHorizontalFoV;
    cameraVerticalFoV_ = cameraVerticalFoV;
    maxDist_ = maxDist;
    double pitch = M_PI * cameraPitch_ / 180.0;
    double camTop = M_PI * (pitch - cameraVerticalFoV_ / 2.0) / 180.0;
    double camBottom = M_PI * (pitch + cameraVerticalFoV_ / 2.0) / 180.0;
    double side = M_PI * (cameraHorizontalFoV_) / 360.0;
    Eigen::Vector3d bottom(cos(camBottom), -sin(camBottom), 0.0);
    Eigen::Vector3d top(cos(camTop), -sin(camTop), 0.0);
    Eigen::Vector3d right(cos(side), -sin(camBottom), 0.0);
    Eigen::Vector3d left(cos(side), sin(camBottom), 0.0);
    Eigen::AngleAxisd m = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    Eigen::Vector3d rightR = m * right;
    Eigen::Vector3d leftR = m * left;
    rightR.normalize();
    leftR.normalize();
    camBoundNormals_.clear();
    camBoundNormals_.push_back(tf::Vector3(bottom.x(), bottom.y(), bottom.z()));
    camBoundNormals_.push_back(tf::Vector3(top.x(), top.y(), top.z()));
    camBoundNormals_.push_back(tf::Vector3(rightR.x(), rightR.y(), rightR.z()));
    camBoundNormals_.push_back(tf::Vector3(leftR.x(), leftR.y(), leftR.z()));
  }

void mesh::StlMesh::incoorporateViewFromPoseMsg(const geometry_msgs::Pose& pose) {
  tf::Transform transform;
  tf::Point point;
  tf::pointMsgToTF(pose.position, point);
  transform.setOrigin(point);
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(pose.orientation, quaternion);
  transform.setRotation(quaternion);
  incoorporateViewFromTf(transform);
  collapse();
}

void mesh::StlMesh::assembleMarkerArray(visualization_msgs::Marker& inspected,
                                        visualization_msgs::Marker& uninspected) const {
  if (isLeaf_) {
    if (isInspected_) {
      geometry_msgs::Point point;
      point.x = x1_.x();
      point.y = x1_.y();
      point.z = x1_.z();
      inspected.points.push_back(point);
      point.x = x2_.x();
      point.y = x2_.y();
      point.z = x2_.z();
      inspected.points.push_back(point);
      point.x = x3_.x();
      point.y = x3_.y();
      point.z = x3_.z();
      inspected.points.push_back(point);
      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 1.0;
      inspected.colors.push_back(color);
      inspected.colors.push_back(color);
      inspected.colors.push_back(color);
    } else {
      geometry_msgs::Point point;
      point.x = x1_.x();
      point.y = x1_.y();
      point.z = x1_.z();
      uninspected.points.push_back(point);
      point.x = x2_.x();
      point.y = x2_.y();
      point.z = x2_.z();
      uninspected.points.push_back(point);
      point.x = x3_.x();
      point.y = x3_.y();
      point.z = x3_.z();
      uninspected.points.push_back(point);
      std_msgs::ColorRGBA color;
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = 1.0;
      uninspected.colors.push_back(color);
      uninspected.colors.push_back(color);
      uninspected.colors.push_back(color);
    }
  } else {
    for (typename std::vector<mesh::StlMesh*>::const_iterator it = children_.begin();
       it != children_.end(); it++) {
      (*it)->assembleMarkerArray(inspected, uninspected);
    }
  }
}

void mesh::StlMesh::incoorporateViewFromTf(const tf::Transform& transform) {
  for (typename std::vector<mesh::StlMesh*>::iterator it = children_.begin();
       it != children_.end(); it++) {
    mesh::StlMesh* currentNode = *it;
    if (currentNode->isInspected_)
      continue;
    bool partialVisibility = false;
    if (currentNode->isVisible(transform, partialVisibility)) {
      currentNode->isInspected_ = true;
      if (!currentNode->isLeaf_ && !currentNode->isHead_) {
        for (typename std::vector<mesh::StlMesh*>::iterator currentChild = currentNode->children_.begin();
             currentChild != currentNode->children_.end(); currentChild++)
          delete *currentChild;
        currentNode->children_.clear();
        currentNode->isLeaf_ = true;
      }
    } else if (partialVisibility && !currentNode->isHead_) {
      if (currentNode->isLeaf_ && currentNode->normal_.norm() > resolution_)
        currentNode->split();
      currentNode->incoorporateViewFromTf(transform);
    } else if (currentNode->isHead_) {
      currentNode->incoorporateViewFromTf(transform);
    }
  }
}

double mesh::StlMesh::computeInspectableArea(const tf::Transform& transform) const {
  if (isLeaf_) {
    bool partiallyVisible;
    if (isVisible(transform, partiallyVisible))
      return normal_.norm();
    else
      return 0.0;
  }

  double ret = 0.0;
  for (typename std::vector<mesh::StlMesh*>::const_iterator it = children_.begin();
       it != children_.end(); it++)
    ret += (*it)->computeInspectableArea(transform);
  return ret;
}

void mesh::StlMesh::split() {
  isLeaf_ = false;
  // #1
  mesh::StlMesh * tmpNode = new mesh::StlMesh;
  tmpNode->x1_ = x1_;
  tmpNode->x2_ = 0.5 * (x1_ + x2_);
  tmpNode->x3_ = 0.5 * (x1_ + x3_);
  tmpNode->normal_ = normal_ / 4.0;
  tmpNode->isHead_ = false;
  tmpNode->isInspected_ = false;
  children_.push_back(tmpNode);
  // #2
  tmpNode = new mesh::StlMesh;
  tmpNode->x1_ = x1_;
  tmpNode->x2_ = 0.5 * (x2_ + x3_);
  tmpNode->x3_ = 0.5 * (x2_ + x1_);
  tmpNode->normal_ = normal_ / 4.0;
  tmpNode->isHead_ = false;
  tmpNode->isInspected_ = false;
  children_.push_back(tmpNode);
  // #3
  tmpNode = new mesh::StlMesh;
  tmpNode->x1_ = x2_;
  tmpNode->x2_ = 0.5 * (x3_ + x1_);
  tmpNode->x3_ = 0.5 * (x3_ + x2_);
  tmpNode->normal_ = normal_ / 4.0;
  tmpNode->isHead_ = false;
  tmpNode->isInspected_ = false;
  children_.push_back(tmpNode);
  // #4
  tmpNode = new mesh::StlMesh;
  tmpNode->x1_ = 0.5 * (x1_ + x2_);
  tmpNode->x2_ = 0.5 * (x2_ + x3_);
  tmpNode->x3_ = 0.5 * (x3_ + x1_);
  tmpNode->normal_ = normal_ / 4.0;
  tmpNode->isHead_ = false;
  tmpNode->isInspected_ = false;
  children_.push_back(tmpNode);
}

bool mesh::StlMesh::collapse() {
  bool collapsible = true;
  if (isLeaf_)
    return true;
  // check if children are all collapsible
  for (typename std::vector<StlMesh*>::iterator it = children_.begin();
       it != children_.end(); it++) {
    if (!(*it)->collapse())
      collapsible = false;
  }
  // collapse if all children have same state
  if (collapsible) {
    bool state = children_.front()->isInspected_;
    for (typename std::vector<StlMesh*>::iterator it = children_.begin();
         it != children_.end(); it++) {
      if ((*it)->isHead_)
        return false;
      if ((*it)->isInspected_ != state)
        return false;
    }
    for (typename std::vector<StlMesh*>::iterator it = children_.begin();
         it != children_.end(); it++)
      delete (*it);
    children_.clear();
    isInspected_ = state;
    isLeaf_ = true;
  }
  return true;
}

bool mesh::StlMesh::isVisible(const tf::Transform& transform, bool& partialVisibility) const {
  bool ret = true;
  partialVisibility = false;
  // #1
  tf::Vector3 origin = transform.getOrigin();
  tf::Vector3 transformedX1 = transform * tf::Vector3(x1_.x(), x1_.y(), x1_.z());
  if (tf::Vector3(normal_.x(), normal_.y(), normal_.z()).dot(origin - transformedX1) <= 0.0)
    return false;
  if ((origin - transformedX1).length() > maxDist_ ||
      !manager_->getVisibility(Eigen::Vector3d(origin.x(), origin.y(), origin.z()),
      Eigen::Vector3d(transformedX1.x(), transformedX1.y(), transformedX1.z()), false)) {
    ret = false;
  } else {
    bool visibility1 = true;
    for (typename std::vector<tf::Vector3>::iterator it = camBoundNormals_.begin();
         it != camBoundNormals_.end(); it++) {
      if (it->dot(transformedX1 - origin) < 0.0) {
        visibility1 = false;
        break;
      }
    }
    if (visibility1)
      partialVisibility = true;
    else
      ret = false;
  }
  // #2
  tf::Vector3 transformedX2 = transform * tf::Vector3(x1_.x(), x1_.y(), x1_.z());
  if ((origin - transformedX2).length() > maxDist_ ||
      !manager_->getVisibility(Eigen::Vector3d(origin.x(), origin.y(), origin.z()),
      Eigen::Vector3d(transformedX2.x(), transformedX2.y(), transformedX2.z()), false)) {
    ret = false;
  } else {
    bool visibility2 = true;
    for (typename std::vector<tf::Vector3>::iterator it = camBoundNormals_.begin();
         it != camBoundNormals_.end(); it++) {
      if (it->dot(transformedX2 - origin) < 0.0) {
        visibility2 = false;
        break;
      }
    }
    if (visibility2)
      partialVisibility = true;
    else
      ret = false;
  }
  if(partialVisibility && !ret)
    return ret;
  // #3
  tf::Vector3 transformedX3 = transform * tf::Vector3(x1_.x(), x1_.y(), x1_.z());
  if ((origin - transformedX3).length() > maxDist_ ||
      !manager_->getVisibility(Eigen::Vector3d(origin.x(), origin.y(), origin.z()),
      Eigen::Vector3d(transformedX3.x(), transformedX3.y(), transformedX3.z()), false)) {
    ret = false;
  } else {
    bool visibility3 = true;
    for (typename std::vector<tf::Vector3>::iterator it = camBoundNormals_.begin();
         it != camBoundNormals_.end(); it++) {
      if (it->dot(transformedX3 - origin) < 0.0) {
        visibility3 = false;
        break;
      }
    }
    if (visibility3)
      partialVisibility = true;
    else
      ret = false;
  }
  return ret;
}

double mesh::StlMesh::resolution_ = 1.0;
double mesh::StlMesh::cameraPitch_ = 15.0;
double mesh::StlMesh::cameraHorizontalFoV_ = 90.0;
double mesh::StlMesh::cameraVerticalFoV_ = 60.0;
double mesh::StlMesh::maxDist_ = 5;
std::vector<tf::Vector3> mesh::StlMesh::camBoundNormals_ = {};
volumetric_mapping::OctomapManager * mesh::StlMesh::manager_ = NULL;

#endif // _MESH_STRUCTURE_CPP_
