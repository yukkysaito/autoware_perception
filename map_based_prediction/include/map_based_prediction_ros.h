/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef MAP_BASED_PREDICTION_ROS_H
#define MAP_BASED_PREDICTION_ROS_H

// // headers in STL
#include <iostream>
// #include <memory>
// #include <vector>

// headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// headers in PCL
#include <pcl/io/pcd_io.h>

// headers in local files
#include "vectormap_ros.h"
#include "map_based_prediction.h"
#include "autoware_msgs/DynamicObjectArray.h"
// #include "lidar_point_pillars/point_pillars.h"

class MapBasedPredictionROS
{
private:
  // rosparam
  bool has_subscribed_map_;
  // bool baselink_support_;
  // bool reproduce_result_mode_;
  // float score_threshold_;
  // float nms_overlap_threshold_;
  // std::string pfe_onnx_file_;
  // std::string rpn_onnx_file_;
  // end rosparam

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_objects_;
  ros::Publisher pub_objects_;
  ros::Publisher pub_markers_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  VectorMap vectormap_;
  MapBasedPrediction map_based_prediction_;

  void objectsCallback(const autoware_msgs::DynamicObjectArrayConstPtr& in_objects);
  
  void publishMarker(const autoware_msgs::DynamicObjectArray& out_objects);

public:
  MapBasedPredictionROS();

  /**
  * @brief Create ROS pub/sub obejct

  * @details Create/Initializing ros pub/sub object
  */
  void createROSPubSub();
};

#endif  // MAP_BASED_PREDICTION_H
