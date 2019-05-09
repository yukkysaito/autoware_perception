/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 * v1.0 Yukihiro Saito
 */

#pragma once

#include <ros/ros.h>
#include "autoware_msgs/DynamicObjectWithFeatureArray.h"
#include "autoware_msgs/DynamicObjectArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <memory>
#include <vector>
#include "multi_object_tracker/tracker/model/tracker_base.hpp"
#include "multi_object_tracker/data_association/data_association.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

class MultiObjectTrackerNode
{
private: // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;
  message_filters::Subscriber<autoware_msgs::DynamicObjectWithFeatureArray> object_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> current_pose_sub_;
  typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::DynamicObjectWithFeatureArray,
                                                          geometry_msgs::PoseStamped>
      SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_ptr_;
  ros::Timer publish_timer_; // publish timer
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void measurementCallback(const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_objects_msg,
                           const geometry_msgs::PoseStamped::ConstPtr &input_current_pose_msg);
  void publishTimerCallback(const ros::TimerEvent &e);

  std::string world_frame_id_;     // tracking frame
  std::string base_link_frame_id_; // current pose child frame
  std::list<std::shared_ptr<Tracker>> list_tracker_;
  DataAssociation data_association_;

public:
  MultiObjectTrackerNode();

  ~MultiObjectTrackerNode(){};
};
