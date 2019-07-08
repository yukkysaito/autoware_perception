#pragma once

#include "points_preprocessor/filter.h"
#include "points_preprocessor/CompareMapFilterConfig.h"
#include <pcl/search/pcl_search.h>
#include <pcl/filters/voxel_grid.h>

namespace points_preprocessor
{
class VoxelBasedCompareMapFilterNodelet : public points_preprocessor::Filter
{
protected:
  boost::shared_ptr<dynamic_reconfigure::Server<points_preprocessor::CompareMapFilterConfig> > srv_;
  virtual void filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices, PointCloud2 &output);
  virtual void subscribe();
  virtual void unsubscribe();

  bool child_init(ros::NodeHandle &nh, bool &has_service);
  void config_callback(points_preprocessor::CompareMapFilterConfig &config, uint32_t level);
  void input_target_callback(const PointCloudConstPtr &map);
  bool is_in_voxel(const pcl::PointXYZ &src_point, const pcl::PointXYZ &target_point, const double distance_threshold,
                   const PointCloudPtr &map, /* Can not add const in PCL specification */ pcl::VoxelGrid<pcl::PointXYZ> &voxel) const;

private:
  // pcl::SegmentDifferences<pcl::PointXYZ> impl_;
  ros::Subscriber sub_map_;
  PointCloudPtr voxel_map_ptr_;
  double distance_threshold_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;
  bool set_map_in_voxel_grid_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace points_preprocessor
