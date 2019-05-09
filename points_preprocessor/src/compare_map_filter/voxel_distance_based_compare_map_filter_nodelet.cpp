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
#include "points_preprocessor/compare_map_filter/voxel_distance_based_compare_map_filter_nodelet.h"


#include <pcl/segmentation/segment_differences.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace points_preprocessor
{
bool VoxelDistanceBasedCompareMapFilterNodelet::child_init(ros::NodeHandle &nh, bool &has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared<dynamic_reconfigure::Server<points_preprocessor::CompareMapFilterConfig> >(nh);
  dynamic_reconfigure::Server<points_preprocessor::CompareMapFilterConfig>::CallbackType f = boost::bind(&VoxelDistanceBasedCompareMapFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  return (true);
}

void VoxelDistanceBasedCompareMapFilterNodelet::filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices,
                                     PointCloud2 &output)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (voxel_map_ptr_ == NULL || map_ptr_ == NULL || tree_ == NULL)
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
    const int index = voxel_grid_.getCentroidIndexAt(voxel_grid_.getGridCoordinates(pcl_input->points.at(i).x, pcl_input->points.at(i).y, pcl_input->points.at(i).z));
    if (index == -1) // empty voxel
    {
      std::vector<int> nn_indices(1);
      std::vector<float> nn_dists(1);
      tree_->nearestKSearch(pcl_input->points.at(i), 1, nn_indices, nn_dists);
      if ( distance_threshold_ * distance_threshold_ < nn_dists.at(0) )
      {
        pcl_output->points.push_back(pcl_input->points.at(i));
      }
    }
  }

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

void VoxelDistanceBasedCompareMapFilterNodelet::input_target_callback(const PointCloudConstPtr &map)
{
  boost::mutex::scoped_lock lock(mutex_);
  tf_input_frame_ = map->header.frame_id;
  // voxel
  voxel_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(distance_threshold_, distance_threshold_, distance_threshold_);
  voxel_grid_.setInputCloud(map);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr_);
  // kdtree
  map_ptr_ = map;
  if (!tree_)
  {
    if (map_ptr_->isOrganized())
    {
      tree_.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
    }
    else
    {
      tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
    }
  }
  tree_->setInputCloud(map_ptr_);
}

void VoxelDistanceBasedCompareMapFilterNodelet::subscribe()
{
  Filter::subscribe();
  sub_map_ = pnh_->subscribe("map", 1, &VoxelDistanceBasedCompareMapFilterNodelet::input_target_callback, this);
}

void VoxelDistanceBasedCompareMapFilterNodelet::unsubscribe()
{
  Filter::unsubscribe();
  sub_map_.shutdown();
}

void VoxelDistanceBasedCompareMapFilterNodelet::config_callback(points_preprocessor::CompareMapFilterConfig &config, uint32_t level)
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

PLUGINLIB_EXPORT_CLASS(points_preprocessor::VoxelDistanceBasedCompareMapFilterNodelet, nodelet::Nodelet);