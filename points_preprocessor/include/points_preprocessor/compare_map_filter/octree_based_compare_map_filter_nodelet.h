#pragma once

#include "points_preprocessor/filter.h"
#include "points_preprocessor/CompareMapFilterConfig.h"
#include <pcl/octree/octree_search.h>

namespace points_preprocessor
{
class OctreeBasedCompareMapFilterNodelet : public points_preprocessor::Filter
{
protected:
  boost::shared_ptr<dynamic_reconfigure::Server<points_preprocessor::CompareMapFilterConfig> > srv_;
  virtual void filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices, PointCloud2 &output);
  virtual void subscribe();
  virtual void unsubscribe();

  bool child_init(ros::NodeHandle &nh, bool &has_service);
  void config_callback(points_preprocessor::CompareMapFilterConfig &config, uint32_t level);
  void input_target_callback(const PointCloudConstPtr &map);

private:
  ros::Subscriber sub_map_;
  PointCloudConstPtr map_ptr_;
  double distance_threshold_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr tree_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace points_preprocessor
