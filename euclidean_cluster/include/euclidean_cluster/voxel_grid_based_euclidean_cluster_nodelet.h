

#pragma once
#include "ros/ros.h"

#include "nodelet/nodelet.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/message_filter.h"
// #include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>


namespace euclidean_cluster
{
class VoxelGridBasedEuclideanClusterNodelet : public nodelet::Nodelet
{

public:
  VoxelGridBasedEuclideanClusterNodelet();

private:
  virtual void onInit();

  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &input_msg);

  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher cluster_pub_;
  ros::Publisher debug_pub_;

  // ROS Parameters
  std::string target_frame_;
  int min_cluster_size_;
  int max_cluster_size_;
  float tolerance_;

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;
  float voxel_leaf_size_;

};

} // namespace euclidean_cluster
