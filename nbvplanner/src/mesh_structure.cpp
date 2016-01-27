/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _MESH_STRUCTURE_CPP_
#define _MESH_STRUCTURE_CPP_

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nbvplanner/mesh_structure.h>

mesh::StlMesh::StlMesh()
    : isLeaf_(true),
      isHead_(true),
      isInspected_(false)
{
}

mesh::StlMesh::StlMesh(const Eigen::Vector3d x1, const Eigen::Vector3d x2, const Eigen::Vector3d x3)
    : x1_(x1),
      x2_(x2),
      x3_(x3),
      isLeaf_(true),
      isHead_(true),
      isInspected_(false),
      normal_((x3 - x2).cross(x1 - x2) / 2.0)
{
}

mesh::StlMesh::StlMesh(std::fstream& file)
    : isLeaf_(true),
      isHead_(true),
      isInspected_(false),
      x1_(0.0, 0.0, 0.0),
      x2_(0.0, 0.0, 0.0),
      x3_(0.0, 0.0, 0.0)
{
  // This routine loads the mesh and constructs the hierarchical structure.
  assert(file.is_open());
  int MaxLine = 0;
  char* line;
  double maxX = -DBL_MAX;
  double maxY = -DBL_MAX;
  double maxZ = -DBL_MAX;
  double minX = DBL_MAX;
  double minY = DBL_MAX;
  double minZ = DBL_MAX;
  line = (char * ) malloc(MaxLine = 80);
  file.getline(line, MaxLine);
  if (0 != strcmp(strtok(line, " "), "solid")) {
    ROS_ERROR("Invalid mesh file! Make sure the STL file is given in ascii-format.");
    return;
  }
  line = (char * ) realloc(line, MaxLine);
  file.getline(line, MaxLine);
  int k = 0;
  while (0 != strcmp(strtok(line, " "), "endsolid") && !ros::isShuttingDown()) {
    int q = 0;
    mesh::StlMesh* newNode = new mesh::StlMesh;
    newNode->isHead_ = true;
    std::vector<Eigen::Vector3d*> vertices;
    vertices.push_back(&(newNode->x1_));
    vertices.push_back(&(newNode->x2_));
    vertices.push_back(&(newNode->x3_));
    int vertexCount = 0;
    for (int i = 0; i < 7; i++) {
      while (line[q] == ' ')
        q++;
      if (line[q] == 'v') {
        // used to rotate the mesh before processing
        const double yawTrafo = 0.0;
        // used to scale the mesh before processing
        const double scaleFactor = 1.0;
        // used to offset the mesh before processing
        const double offsetX = 0.0;
        const double offsetY = 0.0;
        const double offsetZ = 0.0;

        char* v = strtok(line + q, " ");
        v = strtok(NULL, " ");
        double xtmp = atof(v) / scaleFactor;
        v = strtok(NULL, " ");
        double ytmp = atof(v) / scaleFactor;
        vertices[vertexCount]->x() = cos(yawTrafo) * xtmp - sin(yawTrafo) * ytmp;
        vertices[vertexCount]->y() = sin(yawTrafo) * xtmp + cos(yawTrafo) * ytmp;
        v = strtok(NULL, " ");
        vertices[vertexCount]->z() = atof(v) / scaleFactor;
        vertices[vertexCount]->x() -= offsetX;
        vertices[vertexCount]->y() -= offsetY;
        vertices[vertexCount]->z() -= offsetZ;

        if (maxX < vertices[vertexCount]->x())
          maxX = vertices[vertexCount]->x();
        if (maxY < vertices[vertexCount]->y())
          maxY = vertices[vertexCount]->y();
        if (maxZ < vertices[vertexCount]->z())
          maxZ = vertices[vertexCount]->z();
        if (minX > vertices[vertexCount]->x())
          minX = vertices[vertexCount]->x();
        if (minY > vertices[vertexCount]->y())
          minY = vertices[vertexCount]->y();
        if (minZ > vertices[vertexCount]->z())
          minZ = vertices[vertexCount]->z();

        vertexCount++;
      }
      line = (char * ) realloc(line, MaxLine);
      file.getline(line, MaxLine);
    }
    newNode->normal_ = (newNode->x3_ - newNode->x2_).cross(newNode->x1_ - newNode->x2_) / 2.0;
    children_.push_back(newNode);
    k++;
  }
  free(line);
  file.close();
  if (k > 0)
    isLeaf_ = false;
  ROS_INFO(
      "STL file read. Contains %i elements located inside (%2.2f,%2.2f)x(%2.2f,%2.2f)x(%2.2f,%2.2f)",
      k, minX, maxX, minY, maxY, minZ, maxZ);
}

mesh::StlMesh::~StlMesh()
{
  for (typename std::vector<StlMesh*>::iterator it = children_.begin(); it != children_.end(); it++)
    delete (*it);
}

void mesh::StlMesh::setCameraParams(std::vector<double> cameraPitch,
                                    std::vector<double> cameraHorizontalFoV,
                                    std::vector<double> cameraVerticalFoV, double maxDist)
{
  // Precompute the normals of the separating hyperplanes that constrain the field of view.
  cameraPitch_ = cameraPitch;
  cameraHorizontalFoV_ = cameraHorizontalFoV;
  cameraVerticalFoV_ = cameraVerticalFoV;
  maxDist_ = maxDist;
  camBoundNormals_.clear();
  for (int i = 0; i < cameraPitch_.size(); i++) {
    double pitch = M_PI * cameraPitch_[i] / 180.0;
    double camTop = (pitch - M_PI * cameraVerticalFoV_[i] / 360.0) + M_PI / 2.0;
    double camBottom = (pitch + M_PI * cameraVerticalFoV_[i] / 360.0) - M_PI / 2.0;
    double side = M_PI * (cameraHorizontalFoV_[i]) / 360.0 - M_PI / 2.0;
    Eigen::Vector3d bottom(cos(camBottom), 0.0, -sin(camBottom));
    Eigen::Vector3d top(cos(camTop), 0.0, -sin(camTop));
    Eigen::Vector3d right(cos(side), sin(side), 0.0);
    Eigen::Vector3d left(cos(side), -sin(side), 0.0);
    Eigen::AngleAxisd m = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    Eigen::Vector3d rightR = m * right;
    Eigen::Vector3d leftR = m * left;
    rightR.normalize();
    leftR.normalize();
    std::vector<tf::Vector3> camBoundNormals;
    camBoundNormals.push_back(tf::Vector3(bottom.x(), bottom.y(), bottom.z()));
    camBoundNormals.push_back(tf::Vector3(top.x(), top.y(), top.z()));
    camBoundNormals.push_back(tf::Vector3(rightR.x(), rightR.y(), rightR.z()));
    camBoundNormals.push_back(tf::Vector3(leftR.x(), leftR.y(), leftR.z()));
    camBoundNormals_.push_back(camBoundNormals);
  }
}

void mesh::StlMesh::setPeerPose(const geometry_msgs::Pose& pose, int n_peer)
{
  if (peer_vehicles_.size() > n_peer) {
    peer_vehicles_[n_peer] = tf::Vector3(pose.position.x, pose.position.y, pose.position.z);
    return;
  }
  while (peer_vehicles_.size() <= n_peer) {
    peer_vehicles_.push_back(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  }
}

void mesh::StlMesh::incorporateViewFromPoseMsg(const geometry_msgs::Pose& pose, int n_peer)
{
  tf::Transform transform;
  tf::Point point;
  tf::poseMsgToTF(pose, transform);
  // Check that no peer is within the field of view (multi agent only). Find transforms
  // for all specified vehicles by their tf frames and then check for interference.
  bool inAllFoV = true;
  std::vector<bool> unobstructed;
  for (typename std::vector<std::vector<tf::Vector3>>::iterator itCBN = camBoundNormals_.begin();
      itCBN != camBoundNormals_.end(); itCBN++) {
    bool anyInFoV = false;
    for (int it = 0; it < peer_vehicles_.size(); it++) {
      if (it == n_peer) {
        continue;
      }
      bool inFoV = true;
      tf::Vector3 viewDirection = peer_vehicles_[it]
          - tf::Vector3(pose.position.x, pose.position.y, pose.position.z);
      viewDirection.rotate(tf::Vector3(0, 0, 1), tf::getYaw(pose.orientation));
      for (std::vector<tf::Vector3>::iterator itFoVCBN = itCBN->begin(); itFoVCBN != itCBN->end();
          itFoVCBN++) {
        if (itFoVCBN->dot(viewDirection) < 0.0) {
          inFoV = false;
          break;
        }
      }
      if (inFoV) {
        anyInFoV = true;
        break;
      }
    }
    unobstructed.push_back(!anyInFoV);
    inAllFoV &= anyInFoV;
  }
  if (!inAllFoV) {
    // No total interference, can incorporate the data.
    incorporateViewFromTf(transform, unobstructed);
  }
  collapse();
}

void mesh::StlMesh::assembleMarkerArray(visualization_msgs::Marker& inspected,
                                        visualization_msgs::Marker& uninspected) const
{
  // Recursively assemble the array of markers to publish to rviz.
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

void mesh::StlMesh::incorporateViewFromTf(const tf::Transform& transform,
                                          const std::vector<bool>& unobstructed)
{
  for (typename std::vector<mesh::StlMesh*>::iterator it = children_.begin(); it != children_.end();
      it++) {
    mesh::StlMesh* currentNode = *it;
    if (currentNode->isInspected_)
      continue;
    bool partialVisibility = false;
    if (currentNode->getVisibility(transform, partialVisibility, true, unobstructed)) {
      currentNode->isInspected_ = true;
      if (!currentNode->isLeaf_) {
        for (typename std::vector<mesh::StlMesh*>::iterator currentChild = currentNode->children_
            .begin(); currentChild != currentNode->children_.end(); currentChild++)
          delete *currentChild;
        currentNode->children_.clear();
        currentNode->isLeaf_ = true;
      }
    } else if (partialVisibility) {
      if (currentNode->isLeaf_ && currentNode->normal_.norm() > 0.25 * resolution_) {
        currentNode->split();
      }
      currentNode->incorporateViewFromTf(transform, unobstructed);
    }
  }
}

double mesh::StlMesh::computeInspectableArea(const tf::Transform& transform)
{
  if (isLeaf_) {
    if (isInspected_)
      return 0.0;
    bool partiallyVisible;
    if (getVisibility(transform, partiallyVisible, false)) {
      return normal_.norm();
    } else if (partiallyVisible) {
      if (normal_.norm() < 0.25 * resolution_)
        return 0.0;
      split();
    } else {
      return 0.0;
    }
  }

  double ret = 0.0;
  for (typename std::vector<mesh::StlMesh*>::const_iterator it = children_.begin();
      it != children_.end(); it++)
    ret += (*it)->computeInspectableArea(transform);
  return ret;
}

void mesh::StlMesh::split()
{
  // Split in four equally sized triangular facets
  assert(!isInspected_);
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
  tmpNode->x1_ = x2_;
  tmpNode->x2_ = 0.5 * (x2_ + x3_);
  tmpNode->x3_ = 0.5 * (x2_ + x1_);
  tmpNode->normal_ = normal_ / 4.0;
  tmpNode->isHead_ = false;
  tmpNode->isInspected_ = false;
  children_.push_back(tmpNode);
  // #3
  tmpNode = new mesh::StlMesh;
  tmpNode->x1_ = x3_;
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

bool mesh::StlMesh::collapse()
{
  bool collapsible = true;
  if (isLeaf_)
    return true;
  // Check if children are all collapsible
  for (typename std::vector<StlMesh*>::iterator it = children_.begin(); it != children_.end();
      it++) {
    if (!(*it)->collapse())
      collapsible = false;
  }
  // Collapse if all children have same state
  if (collapsible) {
    bool state = true;  // children_.front()->isInspected_;
    for (typename std::vector<StlMesh*>::iterator it = children_.begin(); it != children_.end();
        it++) {
      if ((*it)->isHead_)
        return false;
      if ((*it)->isInspected_ != state)
        return false;
    }
    for (typename std::vector<StlMesh*>::iterator it = children_.begin(); it != children_.end();
        it++)
      delete (*it);
    children_.clear();
    isInspected_ = state;
    isLeaf_ = true;
  }
  return true;
}

bool mesh::StlMesh::getVisibility(const tf::Transform& transform, bool& partialVisibility,
                                  bool stop_at_unknown_cell,
                                  const std::vector<bool>& unobstructed) const
{
  bool ret = true;
  partialVisibility = false;
  // #1
  double yaw = tf::getYaw(transform.getRotation());
  tf::Vector3 originTransf = transform.getOrigin();
  // Check that the facet is visible from the right side.
  if (normal_.dot(x1_ - Eigen::Vector3d(originTransf.x(), originTransf.y(), originTransf.z()))
      >= 0.0)
    return false;
  for (int i = 0; i < camBoundNormals_.size(); i++) {
    if (unobstructed.size() > 0 && !unobstructed[i]) {
      continue;
    }
    tf::Vector3 transformedX1 = transform.inverse() * tf::Vector3(x1_.x(), x1_.y(), x1_.z());
    // Check that corner 1 is within the allowed distance and has free line of sight.
    if (transformedX1.length() > maxDist_
        || manager_->getVisibility(
            Eigen::Vector3d(originTransf.x(), originTransf.y(), originTransf.z()),
            Eigen::Vector3d(x1_.x(), x1_.y(), x1_.z()), stop_at_unknown_cell)
            != volumetric_mapping::OctomapWorld::CellStatus::kFree) {
      return false;
    } else {
      bool visibility1 = true;
      for (typename std::vector<tf::Vector3>::iterator it = camBoundNormals_[i].begin();
          it != camBoundNormals_[i].end(); it++) {
        if (it->dot(transformedX1) < 0.0) {
          visibility1 = false;
          break;
        }
      }
      if (visibility1) {
        partialVisibility = true;
      } else {
        ret = false;
      }
    }
    // #2
    tf::Vector3 transformedX2 = transform.inverse() * tf::Vector3(x2_.x(), x2_.y(), x2_.z());
    // Check that corner 2 is within the allowed distance and has free line of sight.
    if (transformedX2.length() > maxDist_
        || manager_->getVisibility(
            Eigen::Vector3d(originTransf.x(), originTransf.y(), originTransf.z()),
            Eigen::Vector3d(x2_.x(), x2_.y(), x2_.z()), stop_at_unknown_cell)
            != volumetric_mapping::OctomapWorld::CellStatus::kFree) {
      ret = false;
    } else {
      bool visibility2 = true;
      for (typename std::vector<tf::Vector3>::iterator it = camBoundNormals_[i].begin();
          it != camBoundNormals_[i].end(); it++) {
        if (it->dot(transformedX2) < 0.0) {
          visibility2 = false;
          break;
        }
      }
      if (visibility2) {
        partialVisibility = true;
      } else {
        ret = false;
      }
    }
    if (partialVisibility && !ret) {
      ret = true;
      continue;
    }
    // #3
    tf::Vector3 transformedX3 = transform.inverse() * tf::Vector3(x3_.x(), x3_.y(), x3_.z());
    // Check that corner 3 is within the allowed distance and has free line of sight.
    if (transformedX3.length() > maxDist_
        || manager_->getVisibility(
            Eigen::Vector3d(originTransf.x(), originTransf.y(), originTransf.z()),
            Eigen::Vector3d(x3_.x(), x3_.y(), x3_.z()), stop_at_unknown_cell)
            != volumetric_mapping::OctomapWorld::CellStatus::kFree) {
      ret = false;
    } else {
      bool visibility3 = true;
      for (typename std::vector<tf::Vector3>::iterator it = camBoundNormals_[i].begin();
          it != camBoundNormals_[i].end(); it++) {
        if (it->dot(transformedX3) < 0.0) {
          visibility3 = false;
          break;
        }
      }
      if (visibility3) {
        partialVisibility = true;
      } else {
        ret = false;
      }
    }
    if (ret) {
      return true;
    } else {
      ret = true;
    }
  }
  return false;
}

double mesh::StlMesh::resolution_ = 0.001;
std::vector<double> mesh::StlMesh::cameraPitch_ = { };
std::vector<double> mesh::StlMesh::cameraHorizontalFoV_ = { };
std::vector<double> mesh::StlMesh::cameraVerticalFoV_ = { };
double mesh::StlMesh::maxDist_ = 5;
std::vector<std::vector<tf::Vector3> > mesh::StlMesh::camBoundNormals_ = { };
volumetric_mapping::OctomapManager * mesh::StlMesh::manager_ = NULL;
std::vector<tf::Vector3> mesh::StlMesh::peer_vehicles_ = { };

#endif // _MESH_STRUCTURE_CPP_
