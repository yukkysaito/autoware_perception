#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roi_cluster_fusion_node");
  ros::NodeHandle private_nh("~");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "roi_cluster_fusion/roi_cluster_fusion_nodelet", remap, nargv);

  ros::spin();
  return 0;
}
