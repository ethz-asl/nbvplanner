#include <ros/ros.h>
#include <prune_pointcloud/prune.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define SQ(x) ((x)*(x))

PointcloudPruning::Prune::Prune(ros::NodeHandle& n)
{
  n_ = n;
  pointcloudSub_ = n_.subscribe("pointcloudIn", 40, &PointcloudPruning::Prune::pointcloud, this);
  poseSub0_ = n_.subscribe("pose0", 40, &PointcloudPruning::Prune::pose, this);
  poseSub1_ = n_.subscribe("pose1", 40, &PointcloudPruning::Prune::pose, this);
  poseSub2_ = n_.subscribe("pose2", 40, &PointcloudPruning::Prune::pose, this);
  poseSub3_ = n_.subscribe("pose3", 40, &PointcloudPruning::Prune::pose, this);
  poseSub4_ = n_.subscribe("pose4", 40, &PointcloudPruning::Prune::pose, this);
  poseSub5_ = n_.subscribe("pose5", 40, &PointcloudPruning::Prune::pose, this);
  poseSub6_ = n_.subscribe("pose6", 40, &PointcloudPruning::Prune::pose, this);
  poseSub7_ = n_.subscribe("pose7", 40, &PointcloudPruning::Prune::pose, this);
  poseSub8_ = n_.subscribe("pose8", 40, &PointcloudPruning::Prune::pose, this);
  poseSub9_ = n_.subscribe("pose9", 40, &PointcloudPruning::Prune::pose, this);
  pcl_publisher_ = n_.advertise<sensor_msgs::PointCloud2>("pointcloudOut", 1, true);
}

PointcloudPruning::Prune::~Prune()
{
}

void PointcloudPruning::Prune::pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pointcloudIn)
{
  std::vector < tf::Vector3 > agents;
  for (typename std::vector<std::pair<std::string, double> >::iterator it =
      vehicle_tf_frames_.begin(); it != vehicle_tf_frames_.end(); it++) {
    tf::StampedTransform tf_transform;
    ros::Time time_to_lookup = pointcloudIn->header.stamp;
    if (!tf_listener_.canTransform(it->first, pointcloudIn->header.frame_id, time_to_lookup)) {
      time_to_lookup = ros::Time(0);
      ROS_WARN("Using latest TF transform instead of timestamp match.");
    }
    try {
      tf_listener_.lookupTransform(it->first, pointcloudIn->header.frame_id, time_to_lookup,
                                   tf_transform);
    } catch (tf::TransformException& ex) {
      ROS_ERROR_STREAM("Error getting TF transform from sensor data: " << ex.what());
      return;
    }
    agents.push_back(tf_transform.getOrigin());
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pointcloudIn, *cloud);
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it) {
    for (typename std::vector<tf::Vector3>::iterator itPose = agents.begin();
        itPose != agents.end(); itPose++) {
      if (SQ(it->x - itPose->x()) + SQ(it->y - itPose->x()) + SQ(it->z - itPose->x()) < maxDist2_) {
        cloud->erase(it);
      }
    }
  }
  sensor_msgs::PointCloud2::Ptr pointcloudOut(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud, *pointcloudOut);
  pcl_publisher_.publish(pointcloudOut);
}

void PointcloudPruning::Prune::pose(const geometry_msgs::PoseWithCovarianceStamped& pose, int i)
{
  double now = ros::Time::now().toSec();
  for (typename std::vector<std::pair<std::string, double> >::iterator it =
      vehicle_tf_frames_.begin(); it != vehicle_tf_frames_.end(); it++) {
    if (it->first == pose.header.frame_id) {
      return;
    }
    if (now - it->second > store_duration_) {
      vehicle_tf_frames_.erase(it);
    }
  }
  vehicle_tf_frames_.push_back(
      std::pair<std::string, double>(pose.header.frame_id, ros::Time::now().toSec()));
}

void PointcloudPruning::Prune::pose0(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  pose(pose, 0);
}

void PointcloudPruning::Prune::pose1(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  pose(pose, 1);
}

void PointcloudPruning::Prune::pose2(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  pose(pose, 2);
}

void PointcloudPruning::Prune::pose3(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  pose(pose, 3);
}

void PointcloudPruning::Prune::pose4(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  pose(pose, 4);
}

void PointcloudPruning::Prune::pose5(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  pose(pose, 5);
}

void PointcloudPruning::Prune::pose6(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  pose(pose, 6);
}

void PointcloudPruning::Prune::pose7(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  pose(pose, 7);
}

void PointcloudPruning::Prune::pose8(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  pose(pose, 8);
}

void PointcloudPruning::Prune::pose9(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  pose(pose, 9);
}

void PointcloudPruning::Prune::loadParams()
{

  n_.param("max_dist", maxDist2_, 0.5);
  n_.param("store_duration", store_duration_, 2.0);
  maxDist2_ = pow(maxDist2_, 2.0);
}
