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
#include <unique_id/unique_id.h>
#include <autoware_msgs/DynamicObject.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

class Tracker
{
protected:
  boost::uuids::uuid getUUID() { return uuid_; }
  void setType(int type) { type_ = type; }
  virtual bool measure(const autoware_msgs::DynamicObject &object, const ros::Time &time) = 0;

private:
  boost::uuids::uuid uuid_;
  int type_;
  int no_measurement_count_;
  int total_measurement_count_;
  ros::Time last_update_with_measurement_time_;

public:
  Tracker(const ros::Time &time, const int type);
  virtual ~Tracker(){};
  bool updateWithMeasurement(const autoware_msgs::DynamicObject &object, const ros::Time &measurement_time);
  bool updateWithoutMeasurement();
  int getType() { return type_; }
  int getNoMeasurementCount() { return no_measurement_count_; }
  int getTotalMeasurementCount() { return total_measurement_count_; }
  double getElapsedTimeFromLastUpdate() { return (ros::Time::now() - last_update_with_measurement_time_).toSec(); }
  virtual geometry_msgs::Point getPosition() = 0;
  virtual double getArea() = 0;
  virtual bool getEstimatedDynamicObject(const ros::Time &time, autoware_msgs::DynamicObject &object) = 0;
  virtual bool predict(const ros::Time &time) = 0;
};
