#ifndef NBVP_HPP_
#define NBVP_HPP_

#include <fstream>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <nbvplanner/nbvp.h>

// Convenience macro to get the absolute yaw difference
#define ANGABS(x) (fmod(fabs(x),2.0*M_PI)<M_PI?fmod(fabs(x),2.0*M_PI):2.0*M_PI-fmod(fabs(x),2.0*M_PI))

using namespace Eigen;

template<typename stateVec>
nbvInspection::nbvPlanner<stateVec>::nbvPlanner(const ros::NodeHandle& nh,
                                                const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private)
{

  manager_ = new volumetric_mapping::OctomapManager(nh_, nh_private_);

  // set up the topics and services
  params_.inspectionPath_ = nh_.advertise<visualization_msgs::Marker>("inspectionPath", 1000);
  evadePub_ = nh_.advertise<multiagent_collision_check::Segment>("/evasionSegment", 100);
  plannerService_ = nh_.advertiseService("nbvplanner",
                                         &nbvInspection::nbvPlanner<stateVec>::plannerCallback,
                                         this);
  posClient_ = nh_.subscribe("pose", 10, &nbvInspection::nbvPlanner<stateVec>::posCallback, this);

  pointcloud_sub_ = nh_.subscribe("pointcloud_throttled", 40,
                                  &nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTf,
                                  this);

  if (!setParams()) {
    ROS_ERROR("Could not start the planner. Parameters missing!");
  }

  double pitch = M_PI * params_.camPitch_ / 180.0;
  double camTop = M_PI * (pitch - params_.camVertical_ / 2.0) / 180.0 + M_PI / 2.0;
  double camBottom = M_PI * (pitch + params_.camVertical_ / 2.0) / 180.0 - M_PI / 2.0;
  double side = M_PI * (params_.camHorizontal_) / 360.0 - M_PI / 2.0;
  Vector3d bottom(cos(camBottom), 0.0, -sin(camBottom));
  Vector3d top(cos(camTop), 0.0, -sin(camTop));
  Vector3d right(cos(side), sin(side), 0.0);
  Vector3d left(cos(side), -sin(side), 0.0);
  AngleAxisd m = AngleAxisd(pitch, Vector3d::UnitY());
  Vector3d rightR = m * right;
  Vector3d leftR = m * left;
  rightR.normalize();
  leftR.normalize();
  params_.camBoundNormals_.push_back(bottom);
  params_.camBoundNormals_.push_back(top);
  params_.camBoundNormals_.push_back(rightR);
  params_.camBoundNormals_.push_back(leftR);

  std::string ns = ros::this_node::getName();
  std::string stlPath = "";
  mesh_ = NULL;
  if (ros::param::get(ns + "/stl_file_path", stlPath)) {
    if (ros::param::get(ns + "/mesh_resolution", params_.meshResolution_)) {
      std::fstream stlFile;
      stlFile.open(stlPath.c_str());
      if (stlFile.is_open()) {
        mesh_ = new mesh::StlMesh(stlFile);
        mesh_->setResolution(params_.meshResolution_);
        mesh_->setOctomapManager(manager_);
        mesh_->setCameraParams(params_.camPitch_, params_.camHorizontal_, params_.camVertical_,
                               params_.gainRange_);
      } else {
        ROS_WARN("Unable to open STL file");
      }
    } else {
      ROS_WARN("STL mesh file path specified but mesh resolution parameter missing!");
    }
  }
  tree_ = new RrtTree(mesh_, manager_);
  tree_->setParams(params_);

  evadeClient_ = nh_.subscribe("/evasionSegment", 10, &nbvInspection::TreeBase<stateVec>::evade,
                              tree_);

  ready_ = false;
}

template<typename stateVec>
nbvInspection::nbvPlanner<stateVec>::~nbvPlanner()
{
  if (manager_) {
    delete manager_;
  }
  if (mesh_) {
    delete mesh_;
  }
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::posCallback(const geometry_msgs::PoseStamped& pose)
{
  tree_->setStateFromPoseMsg(pose);
  ready_ = true;
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::plannerCallback(nbvplanner::nbvp_srv::Request& req,
                                                          nbvplanner::nbvp_srv::Response& res)
{
  ros::Time computationTime = ros::Time::now();

  if (!ros::ok()) {
    ROS_INFO_THROTTLE(1, "Exploration finished. Not planning any further moves.");
    return true;
  }

  if (!ready_ || manager_ == NULL || manager_->getMapSize().norm() <= 0.0) {
    ROS_ERROR_THROTTLE(1, "Planner not set up");
    return true;
  }
  res.path.clear();

  tree_->clear();
  tree_->initialize();
  vector_t path;
  int loopCount = 0;
  while ((!tree_->gainFound() || tree_->getCounter() < params_.initIterations_) && ros::ok()) {
    if (tree_->getCounter() > params_.cuttoffIterations_) {
      ROS_INFO("No gain found, shutting down");
      ros::shutdown();
      return true;
    }
    if (loopCount > 1000 * (tree_->getCounter() + 1)) {
      ROS_INFO("Exceeding maximum failed iterations, return to previous point!");
      res.path = tree_->getPathBackToPrevious(req.header.frame_id);
      return true;
    }
    tree_->iterate(1);
    loopCount++;
  }
  res.path = tree_->getBestEdge(req.header.frame_id);

  tree_->memorizeBestBranch();
  // Publish path to block for other agents
  multiagent_collision_check::Segment segment;
  segment.header.stamp = ros::Time::now();
  segment.header.frame_id = params_.navigationFrame_;
  if (!res.path.empty()) {
    segment.poses.push_back(res.path.front());
    segment.poses.push_back(res.path.back());
  }
  evadePub_.publish(segment);
  ROS_INFO("Path computation lasted %2.3fs", (ros::Time::now() - computationTime).toSec());
  return true;
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::setParams()
{
  std::string ns = ros::this_node::getName();
  bool ret = true;
  params_.v_max_ = 0.25;
  if (!ros::param::get(ns + "/system/v_max", params_.v_max_)) {
    ROS_WARN("No maximal system speed specified. Looking for %s. Default is 0.25.",
             (ns + "/system/v_max").c_str());
  }
  params_.dyaw_max_ = 0.5;
  if (!ros::param::get(ns + "/system/dyaw_max", params_.dyaw_max_)) {
    ROS_WARN("No maximal yaw speed specified. Looking for %s. Default is 0.5.",
             (ns + "/system/yaw_max").c_str());
  }
  params_.camPitch_ = 15;
  if (!ros::param::get(ns + "/system/camera/pitch", params_.camPitch_)) {
    ROS_WARN("No camera pitch specified. Looking for %s. Default is 15deg.",
             (ns + "/system/camera/pitch").c_str());
  }
  params_.camHorizontal_ = 90;
  if (!ros::param::get(ns + "/system/camera/horizontal", params_.camHorizontal_)) {
    ROS_WARN("No camera horizontal opening specified. Looking for %s. Default is 90deg.",
             (ns + "/system/camera/horizontal").c_str());
  }
  params_.camVertical_ = 60;
  if (!ros::param::get(ns + "/system/camera/vertical", params_.camVertical_)) {
    ROS_WARN("No camera vertical opening specified. Looking for %s. Default is 60.",
             (ns + "/system/camera/vertical").c_str());
  }
  params_.igProbabilistic_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/probabilistic", params_.igProbabilistic_)) {
    ROS_WARN(
        "No gain coefficient for probability of cells specified. Looking for %s. Default is 0.0.",
        (ns + "/nbvp/gain/probabilistic").c_str());
  }
  params_.igFree_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/free", params_.igFree_)) {
    ROS_WARN("No gain coefficient for free cells specified. Looking for %s. Default is 0.0.",
             (ns + "/nbvp/gain/free").c_str());
  }
  params_.igOccupied_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/occupied", params_.igOccupied_)) {
    ROS_WARN("No gain coefficient for occupied cells specified. Looking for %s. Default is 0.0.",
             (ns + "/nbvp/gain/occupied").c_str());
  }
  params_.igUnmapped_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/gain/unmapped", params_.igUnmapped_)) {
    ROS_WARN("No gain coefficient for unmapped cells specified. Looking for %s. Default is 1.0.",
             (ns + "/nbvp/gain/unmapped").c_str());
  }
  params_.igArea_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/gain/area", params_.igArea_)) {
    ROS_WARN("No gain coefficient for mesh area specified. Looking for %s. Default is 1.0.",
             (ns + "/nbvp/gain/area").c_str());
  }
  params_.degressiveCoeff_ = 0.25;
  if (!ros::param::get(ns + "/nbvp/gain/degressive_coeff", params_.degressiveCoeff_)) {
    ROS_WARN(
        "No degressive factor for gain accumulation specified. Looking for %s. Default is 0.25.",
        (ns + "/nbvp/gain/degressive_coeff").c_str());
  }
  params_.extensionRange_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/tree/extension_range", params_.extensionRange_)) {
    ROS_WARN("No value for maximal extension range specified. Looking for %s. Default is 1.0m.",
             (ns + "/nbvp/tree/extension_range").c_str());
  }
  params_.initIterations_ = 15;
  if (!ros::param::get(ns + "/nbvp/tree/initial_iterations", params_.initIterations_)) {
    ROS_WARN("No number of initial tree iterations specified. Looking for %s. Default is 15.",
             (ns + "/nbvp/tree/initial_iterations").c_str());
  }
  params_.dt_ = 0.1;
  if (!ros::param::get(ns + "/nbvp/dt", params_.dt_)) {
    ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.",
             (ns + "/nbvp/dt").c_str());
  }
  params_.gainRange_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/gain/range", params_.gainRange_)) {
    ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.",
             (ns + "/nbvp/gain/range").c_str());
  }
  if (!ros::param::get(ns + "/bbx/minX", params_.minX_)) {
    ROS_WARN("No x-min value specified. Looking for %s", (ns + "/bbx/minX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/minY", params_.minY_)) {
    ROS_WARN("No y-min value specified. Looking for %s", (ns + "/bbx/minY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/minZ", params_.minZ_)) {
    ROS_WARN("No z-min value specified. Looking for %s", (ns + "/bbx/minZ").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxX", params_.maxX_)) {
    ROS_WARN("No x-max value specified. Looking for %s", (ns + "/bbx/maxX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxY", params_.maxY_)) {
    ROS_WARN("No y-max value specified. Looking for %s", (ns + "/bbx/maxY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxZ", params_.maxZ_)) {
    ROS_WARN("No z-max value specified. Looking for %s", (ns + "/bbx/maxZ").c_str());
    ret = false;
  }
  params_.softBounds_ = false;
  if (!ros::param::get(ns + "/bbx/softBounds", params_.softBounds_)) {
    ROS_WARN(
        "Not specified whether scenario bounds are soft or hard. Looking for %s. Default is false",
        (ns + "/bbx/softBounds").c_str());
  }
  params_.boundingBox_[0] = 0.5;
  if (!ros::param::get(ns + "/system/bbx/x", params_.boundingBox_[0])) {
    ROS_WARN("No x size value specified. Looking for %s. Default is 0.5m.",
             (ns + "/system/bbx/x").c_str());
  }
  params_.boundingBox_[1] = 0.5;
  if (!ros::param::get(ns + "/system/bbx/y", params_.boundingBox_[1])) {
    ROS_WARN("No y size value specified. Looking for %s. Default is 0.5m.",
             (ns + "/system/bbx/y").c_str());
  }
  params_.boundingBox_[2] = 0.3;
  if (!ros::param::get(ns + "/system/bbx/z", params_.boundingBox_[2])) {
    ROS_WARN("No z size value specified. Looking for %s. Default is 0.3m.",
             (ns + "/system/bbx/z").c_str());
  }
  params_.cuttoffIterations_ = 200;
  if (!ros::param::get(ns + "/nbvp/tree/cuttoff_iterations", params_.cuttoffIterations_)) {
    ROS_WARN("No cuttoff iterations value specified. Looking for %s. Default is 200.",
             (ns + "/nbvp/tree/cuttoff_iterations").c_str());
  }
  params_.zero_gain_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/zero", params_.zero_gain_)) {
    ROS_WARN("No zero gain value specified. Looking for %s. Default is 0.0.",
             (ns + "/nbvp/gain/zero").c_str());
  }
  params_.dOvershoot_ = 0.5;
  if (!ros::param::get(ns + "/system/bbx/overshoot", params_.dOvershoot_)) {
    ROS_WARN(
        "No estimated overshoot value for collision avoidance specified. Looking for %s. Default is 0.5m.",
        (ns + "/system/bbx/overshoot").c_str());
  }
  params_.log_ = false;
  if (!ros::param::get(ns + "/nbvp/log/on", params_.log_)) {
    ROS_WARN("Logging is off by default. Turn on with %s: true", (ns + "/nbvp/log/on").c_str());
  }
  params_.log_throttle_ = 0.5;
  if (!ros::param::get(ns + "/nbvp/log/throttle", params_.log_throttle_)) {
    ROS_WARN("No throttle time for logging specified. Looking for %s. Default is 0.5s.",
             (ns + "/nbvp/log/throttle").c_str());
  }
  params_.navigationFrame_ = "world";
  if (!ros::param::get(ns + "/tf_frame", params_.navigationFrame_)) {
    ROS_WARN("No navigation frame specified. Looking for %s. Default is 'world'.",
             (ns + "/tf_frame").c_str());
  }
  params_.pcl_throttle_ = 0.0;
  if (!ros::param::get(ns + "/pcl_throttle", params_.pcl_throttle_)) {
    ROS_WARN(
        "No throttle time constant for the point cloud insertion specified. Looking for %s. Default is 0.0.",
        (ns + "/pcl_throttle").c_str());
  }
  return ret;
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTf(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  static double last = ros::Time::now().toSec();
  if (last + params_.pcl_throttle_ < ros::Time::now().toSec()) {
    tree_->insertPointcloudWithTf(pointcloud);
    last += params_.pcl_throttle_;
  }
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::evasionCallback(
    const multiagent_collision_check::Segment& segmentMsg)
{
  tree_->evade(segmentMsg);
}

#endif // NBVP_HPP_
