#pragma once
#include <ros/ros.h>
#include "autoware_msgs/DynamicObjectWithFeatureArray.h"
#include "autoware_msgs/DynamicObjectArray.h"
#include "autoware_msgs/Shape.h"
#include "autoware_msgs/PredictedPath.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/ColorRGBA.h"

class DynamicObjectVisualizer
{
private: // ros
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  void dynamicObjectWithFeatureCallback(const autoware_msgs::DynamicObjectWithFeatureArray::ConstPtr &input_msg);
  void dynamicObjectCallback(const autoware_msgs::DynamicObjectArray::ConstPtr &input_msg);
  bool calcBoundingBoxLineList(const autoware_msgs::Shape &shape, std::vector<geometry_msgs::Point> &points);
  bool calcCylinderLineList(const autoware_msgs::Shape &shape, std::vector<geometry_msgs::Point> &points);
  bool calcCircleLineList(const geometry_msgs::Point center, const double radius, std::vector<geometry_msgs::Point> &points, const int n = 20);
  bool calcPolygonLineList(const autoware_msgs::Shape &shape, std::vector<geometry_msgs::Point> &points);
  bool calcPathLineList(const autoware_msgs::PredictedPath &path, std::vector<geometry_msgs::Point> &points);
  bool getLabel(const autoware_msgs::Semantic &semantic, std::string &label);
  void getColor(const autoware_msgs::DynamicObject &object, std_msgs::ColorRGBA &color);
  void initColorList(std::vector<std_msgs::ColorRGBA> &colors);
  void initPose(geometry_msgs::Pose &pose);

  bool only_known_objects_;
  std::vector<std_msgs::ColorRGBA> colors_;

public:
  DynamicObjectVisualizer();
  virtual ~DynamicObjectVisualizer() {}
};