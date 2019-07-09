#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "voxel_grid_based_euclidean_cluster_node");
  ros::NodeHandle private_nh("~");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "euclidean_cluster/voxel_grid_based_euclidean_cluster_nodelet", remap, nargv);

  ros::spin();
  return 0;

}
