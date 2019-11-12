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

// headers in STL
#include <chrono>
#include <cmath>

// headers in PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>

//headers in ROS
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// headers in local files
#include "map_based_prediction_ros.h"

MapBasedPredictionROS::MapBasedPredictionROS():
pnh_("~"),tf_listener_(tf_buffer_)
{
  pnh_.param<bool>("map_based_prediction/has_subscribed_map", has_subscribed_map_, false);
}

void MapBasedPredictionROS::createROSPubSub()
{
  sub_objects_ = nh_.subscribe<autoware_msgs::DynamicObjectArray>("/perception/tracking/objects", 1, &MapBasedPredictionROS::objectsCallback, this);
  pub_objects_ = nh_.advertise<autoware_msgs::DynamicObjectArray>("objects", 1);
  pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("objects_path_markers", 1);
}

void MapBasedPredictionROS::objectsCallback(const autoware_msgs::DynamicObjectArrayConstPtr& in_objects)
{
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  autoware_msgs::DynamicObjectArray objects_in_map;
  /* transform to map coordinate */
  geometry_msgs::TransformStamped objects2map;
  try
  {
    objects2map = tf_buffer_.lookupTransform(
      /*target*/ "map", 
      /*src*/ in_objects->header.frame_id,
              in_objects->header.stamp, ros::Duration(0.5));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  for(const auto& object: in_objects->objects)
  { 
    geometry_msgs::Pose pose_out;
    tf2::doTransform(object.state.pose.pose, pose_out, objects2map);
    autoware_msgs::DynamicObject tmp_object;
    tmp_object = object;
    tmp_object.state.pose.pose = pose_out;
    objects_in_map.objects.push_back(tmp_object);
  }
  

  // subscribe
  if(!has_subscribed_map_)
  {
    //TODO: make another method for this part
    has_subscribed_map_ =  vectormap_.load();
    if(!has_subscribed_map_)
    {
      ROS_ERROR("Could not subscribe to vectormap");
      return;
    }
    ROS_INFO("succeeded to load vector map");
  }

  autoware_msgs::DynamicObjectArray out_objects;
  if(map_based_prediction_.doPrediction(objects_in_map, vectormap_.lane_points_, out_objects))
  {
    /* transform form map to world coordinate */
    geometry_msgs::TransformStamped map2world;
    try
    {
      map2world = tf_buffer_.lookupTransform(
        /*target*/ in_objects->header.frame_id, 
        /*src*/ "map",
        in_objects->header.stamp, ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    autoware_msgs::DynamicObjectArray objects_in_world;
    objects_in_world.header = in_objects->header;
    for(const auto& object: out_objects.objects)
    { 
      geometry_msgs::Pose pose_out;
      tf2::doTransform(object.state.pose.pose, pose_out, map2world);
      autoware_msgs::DynamicObject tmp_object;
      tmp_object = object;
      tmp_object.state.pose.pose = pose_out;
      tmp_object.state.predicted_paths.clear();
      for(const auto& path: object.state.predicted_paths)
      {
        autoware_msgs::PredictedPath tmp_path;
        tmp_path.confidence = path.confidence;
        for(const auto& path_pose: path.path)
        {
          geometry_msgs::PoseWithCovarianceStamped tmp;
          tmp = path_pose;
          geometry_msgs::Pose tmp_path_pose;
          tf2::doTransform(path_pose.pose.pose, tmp_path_pose, map2world);
          tmp.pose.pose = tmp_path_pose;
          tmp_path.path.push_back(tmp);
        }
        tmp_object.state.predicted_paths.push_back(tmp_path);
      }
      objects_in_world.objects.push_back(tmp_object);
    }
    publishMarker(objects_in_world);
  }
  else
  {
    ROS_ERROR("Faile to predict");
  }
  
  // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  // // std::cerr << "Prediction took " << time_span.count()*1000 << " msec." << std::endl;
  // // std::cerr << "prediction success" << std::endl;

}

void MapBasedPredictionROS::publishMarker(const autoware_msgs::DynamicObjectArray& out_objects)
{
  visualization_msgs::MarkerArray predicted_markers;
  // for (int i = 0; i < out_objects.objects.size(); i++)
  int unique_id = 0;
  for (const auto& object: out_objects.objects)
  {
    // std::cerr << "num parts " << object.state.prediction_paths.size() << std::endl;
    // std::cerr << "---------" << std::endl;
    for(const auto& path: object.state.predicted_paths)
    {
      // std::cerr << "confidence " << path.confidence << std::endl;
      visualization_msgs::Marker predicted_line;
      predicted_line.lifetime = ros::Duration(0.2);
      predicted_line.header = out_objects.header;
      predicted_line.ns = std::string("predicted_line");
      predicted_line.action = visualization_msgs::Marker::MODIFY;
      predicted_line.pose.orientation.w = 1.0;
      predicted_line.id = unique_id;
      predicted_line.type = visualization_msgs::Marker::LINE_STRIP;
      predicted_line.scale.x = 0.1;

      // Points are red
      predicted_line.color.r = 1.0f;
      predicted_line.color.g = 1.0f;
      predicted_line.color.a = 0.2;

      visualization_msgs::Marker predicted_points;
      predicted_points.lifetime = ros::Duration(0.2);
      predicted_points.header = out_objects.header;
      predicted_points.ns = std::string("predicted_points");
      predicted_points.action = visualization_msgs::Marker::MODIFY;
      predicted_points.pose.orientation.w = 1.0;
      predicted_points.id = unique_id;
      predicted_points.type = visualization_msgs::Marker::SPHERE_LIST;
      predicted_points.scale.x = 0.5;

      // Points are yellow
      predicted_points.color.r = 1.0f;
      predicted_points.color.g = 1.0f;
      predicted_points.color.a = path.confidence*0.5;

      for(const auto& point: path.path)
      {
        
        geometry_msgs::Point geometry_point;
        //goal point, it extracts the first pose
        geometry_point.x = point.pose.pose.position.x;
        geometry_point.y = point.pose.pose.position.y;
        geometry_point.z = object.state.pose.pose.position.z - 1.0;
        predicted_points.points.push_back(geometry_point);
        predicted_line.points.push_back(geometry_point);
      }
      
      predicted_markers.markers.push_back(predicted_line);
      predicted_markers.markers.push_back(predicted_points);
      unique_id++;
    }
  }
  // std::cerr <<"num input objects " <<out_objects.objects.size() << std::endl;
  // std::cerr <<"num markers " <<predicted_lines.markers.size() << std::endl;
  pub_markers_.publish(predicted_markers);
  pub_objects_.publish(out_objects);
}
// MapBasedPredictionROS::MapBased()
//   : private_nh_("~")
//   , has_subscribed_baselink_(false)
//   , NUM_POINT_FEATURE_(4)
//   , OUTPUT_NUM_BOX_FEATURE_(7)
//   , TRAINED_SENSOR_HEIGHT_(1.73f)
//   , NORMALIZING_INTENSITY_VALUE_(255.0f)
//   , BASELINK_FRAME_("base_link")
// {
//   //ros related param
//   private_nh_.param<bool>("baselink_support", baselink_support_, true);

//   //algorithm related params
//   private_nh_.param<bool>("reproduce_result_mode", reproduce_result_mode_, false);
//   private_nh_.param<float>("score_threshold", score_threshold_, 0.5f);
//   private_nh_.param<float>("nms_overlap_threshold", nms_overlap_threshold_, 0.5f);
//   private_nh_.param<std::string>("pfe_onnx_file", pfe_onnx_file_, "");
//   private_nh_.param<std::string>("rpn_onnx_file", rpn_onnx_file_, "");

//   point_pillars_ptr_.reset(new PointPillars(reproduce_result_mode_, score_threshold_, nms_overlap_threshold_,
//                                             pfe_onnx_file_, rpn_onnx_file_));
// }