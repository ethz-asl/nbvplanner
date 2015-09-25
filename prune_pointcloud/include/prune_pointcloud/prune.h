#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

namespace PointcloudPruning {

class Prune
{
 public:
  Prune(ros::NodeHandle& n);
  ~Prune();
  void pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pointcloudIn);
 private:
  ros::NodeHandle n_;
  ros::Publisher pcl_publisher_;
  ros::Subscriber pointcloudSub_;
  tf::TransformListener tf_listener_;
  void loadParams();
  std::vector<std::string> vehicle_tf_frames_;
  double maxDist2_;
};

}
