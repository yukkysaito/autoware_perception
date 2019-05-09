#include "tensorrt_yolo3_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tensorrt_yolo3");
  TensorrtYoloROS app;
  app.createROSPubSub();
  ros::spin();

  return 0;
}