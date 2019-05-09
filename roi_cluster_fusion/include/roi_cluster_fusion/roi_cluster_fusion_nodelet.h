#pragma once
#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/pass_through.h"
#include "sensor_msgs/CameraInfo.h"
#include "autoware_msgs/DynamicObjectWithFeatureArray.h"
#include "autoware_msgs/DynamicObjectWithFeature.h"
#include <memory>

namespace roi_cluster_fusion
{
class RoiClusterFusionNodelet : public nodelet::Nodelet
{

public:
  RoiClusterFusionNodelet();

private:
  virtual void onInit();

  void fusionCallback(const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_cluster_msg,
                      const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_roi0_msg,
                      const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_roi1_msg,
                      const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_roi2_msg,
                      const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_roi3_msg,
                      const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_roi4_msg,
                      const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_roi5_msg,
                      const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_roi6_msg,
                      const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_roi7_msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &input_camera_info_msg, const int id);
  double calcIoU(const sensor_msgs::RegionOfInterest &roi_1,
                 const sensor_msgs::RegionOfInterest &roi_2);
  double calcIoUX(const sensor_msgs::RegionOfInterest &roi_1,
                  const sensor_msgs::RegionOfInterest &roi_2);
  double calcIoUY(const sensor_msgs::RegionOfInterest &roi_1,
                  const sensor_msgs::RegionOfInterest &roi_2);

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher labeled_cluster_pub_;
  std::vector<std::shared_ptr<ros::Subscriber> > v_camera_info_sub_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  message_filters::Subscriber<autoware_msgs::DynamicObjectWithFeatureArray> cluster_sub_;
  std::vector<std::shared_ptr<message_filters::Subscriber<autoware_msgs::DynamicObjectWithFeatureArray> > > v_roi_sub_;
  message_filters::PassThrough<autoware_msgs::DynamicObjectWithFeatureArray> passthrough_;
  typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::DynamicObjectWithFeatureArray,
                                                          autoware_msgs::DynamicObjectWithFeatureArray,
                                                          autoware_msgs::DynamicObjectWithFeatureArray,
                                                          autoware_msgs::DynamicObjectWithFeatureArray,
                                                          autoware_msgs::DynamicObjectWithFeatureArray,
                                                          autoware_msgs::DynamicObjectWithFeatureArray,
                                                          autoware_msgs::DynamicObjectWithFeatureArray,
                                                          autoware_msgs::DynamicObjectWithFeatureArray,
                                                          autoware_msgs::DynamicObjectWithFeatureArray>
      SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_ptr_;
  inline void
  dummyCallback(const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input)
  {
    autoware_msgs::DynamicObjectWithFeatureArray dummy;
    dummy.header.stamp = input->header.stamp;
    passthrough_.add(boost::make_shared<autoware_msgs::DynamicObjectWithFeatureArray>(dummy));
      }
  // ROS Parameters
  bool use_iou_x_;
  bool use_iou_y_;
  bool use_iou_;
  double iou_threshold_;
  int rois_number_;
  std::map<int, sensor_msgs::CameraInfo> m_camera_info_;
};

} // namespace roi_cluster_fusion
