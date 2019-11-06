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


#ifndef MAP_BASED_PREDICTION_H
#define MAP_BASED_PREDICTION_H

// // headers in STL
#include <iostream>
#include <cmath>

// headers in ROS
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//headers in ROS
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <unique_id/unique_id.h>

// headers in PCL
// #include <pcl/io/pcd_io.h>

//headers in Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// headers in local files
#include "vectormap_struct.h"
#include "autoware_msgs/DynamicObjectArray.h"

// #include "lidar_point_pillars/point_pillars.h"

class MapBasedPrediction
{
private:

  // rosparam
  // bool has_subscribed_map_;
  // bool baselink_support_;
  // bool reproduce_result_mode_;
  // float score_threshold_;
  // float nms_overlap_threshold_;
  // std::string pfe_onnx_file_;
  // std::string rpn_onnx_file_;
  // end rosparam

  // ros::NodeHandle nh_;
  // ros::NodeHandle pnh_;
  // ros::Subscriber sub_objects_;
  // ros::Publisher pub_objects_;

  // tf2_ros::Buffer tf_buffer_;
  // tf2_ros::TransformListener tf_listener_;

  // VectorMap vectormap_;

  // void objectsCallback(const autoware_msgs::DynamicObjectArrayConstPtr& msg);
  bool getPredictedPath(
    const std::vector<LanePoint>& goal_path,
    const double current_yaw,
    const double current_height,
    const double current_d_position,
    const double current_d_velocity,
    const double current_s_position,
    const double current_s_velocity,
    const double target_s_position, 
    autoware_msgs::PredictedPath& path);

  bool getNearestLane(const std::vector<LanePoint>& lane_points, 
  const geometry_msgs::Pose object_pose,
  LanePoint& nearest_lane_point);

  bool calculateCartesianPositionUsingLanePoints(const std::vector<LanePoint>& goal_path, 
                           const double target_s,
                           const double target_d,
                           geometry_msgs::PoseWithCovarianceStamped& converted_position);
  bool convertFrenetPosition2CartesianPosition(
                           const double target_s,
                           const double target_d,
                           const geometry_msgs::Point& cumulated_s_position,
                           const double delta_s,
                           const double delta_yaw,
                           geometry_msgs::PoseWithCovarianceStamped& converted_position);
                           
  double calculateLikelyhood(const double desired_yaw, const double current_d, const double current_yaw);
  
  bool normalizeLikelyhood(std::vector<autoware_msgs::PredictedPath>& paths);

  void toEulerAngle(const geometry_msgs::Quaternion& q, 
                    double& roll, double& pitch, double& yaw);
public:
  MapBasedPrediction();
  
  bool doPrediction(const autoware_msgs::DynamicObjectArray& in_objects,
                    const std::vector<LanePoint>& lane_poitns,
                    autoware_msgs::DynamicObjectArray& out_objects);
                    
  
  /**
  * @brief Create ROS pub/sub obejct

  * @details Create/Initializing ros pub/sub object
  */
  // void createROSPubSub();
};

#endif  // MAP_BASED_PREDICTION_H
