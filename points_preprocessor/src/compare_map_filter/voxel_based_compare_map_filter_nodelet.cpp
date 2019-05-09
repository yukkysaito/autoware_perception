
/*
 *  Copyright (c) 2018, TierIV, Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <pluginlib/class_list_macros.h>
#include "points_preprocessor/compare_map_filter/voxel_based_compare_map_filter_nodelet.h"


#include <pcl/segmentation/segment_differences.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace points_preprocessor
{
bool VoxelBasedCompareMapFilterNodelet::child_init(ros::NodeHandle &nh, bool &has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared<dynamic_reconfigure::Server<points_preprocessor::CompareMapFilterConfig> >(nh);
  dynamic_reconfigure::Server<points_preprocessor::CompareMapFilterConfig>::CallbackType f = boost::bind(&VoxelBasedCompareMapFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  return (true);
}

void VoxelBasedCompareMapFilterNodelet::filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices,
                                     PointCloud2 &output)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (voxel_map_ptr_ == NULL)
  {
    output = *input;
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_output->points.reserve(pcl_input->points.size());
  for (size_t i = 0; i < pcl_input->points.size(); ++i)
  {
    const pcl::PointXYZ point = pcl_input->points.at(i);
    if (is_in_voxel(pcl::PointXYZ(point.x, point.y, point.z), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x, point.y - distance_threshold_, point.z - distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x, point.y - distance_threshold_, point.z), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x, point.y - distance_threshold_, point.z + distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x, point.y, point.z - distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x, point.y, point.z + distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x, point.y + distance_threshold_, point.z - distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x, point.y + distance_threshold_, point.z), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x, point.y + distance_threshold_, point.z + distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;

    if (is_in_voxel(pcl::PointXYZ(point.x - distance_threshold_, point.y - distance_threshold_, point.z - distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x - distance_threshold_, point.y - distance_threshold_, point.z), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x - distance_threshold_, point.y - distance_threshold_, point.z + distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x - distance_threshold_, point.y, point.z - distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x - distance_threshold_, point.y, point.z), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x - distance_threshold_, point.y, point.z + distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x - distance_threshold_, point.y + distance_threshold_, point.z - distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x - distance_threshold_, point.y + distance_threshold_, point.z), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x - distance_threshold_, point.y + distance_threshold_, point.z + distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;

    if (is_in_voxel(pcl::PointXYZ(point.x + distance_threshold_, point.y - distance_threshold_, point.z - distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x + distance_threshold_, point.y - distance_threshold_, point.z), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x + distance_threshold_, point.y - distance_threshold_, point.z + distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x + distance_threshold_, point.y, point.z - distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x + distance_threshold_, point.y, point.z), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x + distance_threshold_, point.y, point.z + distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x + distance_threshold_, point.y + distance_threshold_, point.z - distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x + distance_threshold_, point.y + distance_threshold_, point.z), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;
    if (is_in_voxel(pcl::PointXYZ(point.x + distance_threshold_, point.y + distance_threshold_, point.z + distance_threshold_), point, distance_threshold_, voxel_map_ptr_, voxel_grid_))
      continue;


    pcl_output->points.push_back(pcl_input->points.at(i));
  }
  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

bool VoxelBasedCompareMapFilterNodelet::is_in_voxel(const pcl::PointXYZ &src_point,
                                                    const pcl::PointXYZ &target_point,
                                                    const double distance_threshold,
                                                    const PointCloudPtr &map,
                                                    pcl::VoxelGrid<pcl::PointXYZ> &voxel) const
{
  int voxel_index = voxel.getCentroidIndexAt(voxel.getGridCoordinates(src_point.x, src_point.y, src_point.z));
  if (voxel_index != -1) // not empty voxel
  {
    const double dist_x = map->points.at(voxel_index).x - target_point.x;
    const double dist_y = map->points.at(voxel_index).y - target_point.y;
    const double dist_z = map->points.at(voxel_index).z - target_point.z;
    const double sqr_distance = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;
    if (sqr_distance < distance_threshold * distance_threshold)
    {
      return true;
    }
  }
  return false;
}

void VoxelBasedCompareMapFilterNodelet::input_target_callback(const PointCloudConstPtr &map)
{
  boost::mutex::scoped_lock lock(mutex_);
  tf_input_frame_ = map->header.frame_id;
  voxel_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(distance_threshold_, distance_threshold_, distance_threshold_);
  voxel_grid_.setInputCloud(map);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr_);
}

void VoxelBasedCompareMapFilterNodelet::subscribe()
{
  Filter::subscribe();
  sub_map_ = pnh_->subscribe("map", 1, &VoxelBasedCompareMapFilterNodelet::input_target_callback, this);
}

void VoxelBasedCompareMapFilterNodelet::unsubscribe()
{
  Filter::unsubscribe();
  sub_map_.shutdown();
}

void VoxelBasedCompareMapFilterNodelet::config_callback(points_preprocessor::CompareMapFilterConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (distance_threshold_ != config.distance_threshold)
  {
    distance_threshold_ = config.distance_threshold;
    NODELET_DEBUG("[%s::config_callback] Setting new distance threshold to: %f.", getName().c_str(), config.distance_threshold);
  }
  // ---[ These really shouldn't be here, and as soon as dynamic_reconfigure improves, we'll remove them and inherit from Filter
  if (tf_output_frame_ != config.output_frame)
  {
    tf_output_frame_ = config.output_frame;
    NODELET_DEBUG("[config_callback] Setting the output TF frame to: %s.", tf_output_frame_.c_str());
  }
  // ]---
}

} // namespace points_preprocessor

PLUGINLIB_EXPORT_CLASS(points_preprocessor::VoxelBasedCompareMapFilterNodelet, nodelet::Nodelet);