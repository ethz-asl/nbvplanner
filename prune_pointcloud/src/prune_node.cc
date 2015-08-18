#include <ros/ros.h>
#include <prune_pointcloud/prune.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pruner");

  ros::NodeHandle n;
  PointcloudPruning::Prune prune(n);

  ros::spin();

  return 0;
}
