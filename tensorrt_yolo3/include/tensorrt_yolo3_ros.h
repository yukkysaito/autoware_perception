#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <autoware_msgs/DynamicObjectWithFeatureArray.h>

// STL
#include <string>
#include <memory>
#include <chrono>

//local
#include "TrtNet.h"
#include "data_reader.h"

class TensorrtYoloROS
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_image_;
  ros::Publisher pub_objects_;
  ros::Publisher pub_image_;
  
  std::unique_ptr<Tn::trtNet> net_ptr_;
  
  void imageCallback(const sensor_msgs::Image::ConstPtr& in_image);
  std::vector<float> prepareImage(cv::Mat& in_img);
  std::vector<Tn::Bbox> postProcessImg(std::vector<Yolo::Detection>& detections,
                                       const int classes, 
                                       cv::Mat& img, 
                                       autoware_msgs::DynamicObjectWithFeatureArray & out_objects);
  void doNms(std::vector<Yolo::Detection>& detections,int classes ,float nmsThresh);
  /* data */
public:
  TensorrtYoloROS(/* args */);
  ~TensorrtYoloROS();
  
  void createROSPubSub();
};
