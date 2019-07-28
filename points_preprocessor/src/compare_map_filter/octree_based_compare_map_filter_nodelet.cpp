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
#include "points_preprocessor/compare_map_filter/octree_based_compare_map_filter_nodelet.h"

#include <pcl/segmentation/segment_differences.h>
#include <chrono>

namespace points_preprocessor
{
bool OctreeBasedCompareMapFilterNodelet::child_init(ros::NodeHandle &nh, bool &has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared<dynamic_reconfigure::Server<points_preprocessor::CompareMapFilterConfig> >(nh);
  dynamic_reconfigure::Server<points_preprocessor::CompareMapFilterConfig>::CallbackType f = boost::bind(&OctreeBasedCompareMapFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  return (true);
}

void OctreeBasedCompareMapFilterNodelet::filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices,
                                     PointCloud2 &output)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (map_ptr_ == nullptr || tree_ == nullptr)
  {
    output = *input;
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_output->points.reserve(pcl_input->points.size());

#if 0
{  std::chrono::system_clock::time_point start, end; //
  start = std::chrono::system_clock::now();         //

  for (size_t i = 0; i < pcl_input->points.size(); ++i)
  {
    std::vector<int> v_idx;
    std::vector<float> v_squared_distance;
    if (tree_->nearestKSearch(pcl_input->points.at(i), 1, v_idx, v_squared_distance) > 0)
    {
      if (distance_threshold_ * distance_threshold_ < v_squared_distance.at(0))
      {
        pcl_output->points.push_back(pcl_input->points.at(i));
      }
    }
  }
  end = std::chrono::system_clock::now();                                                                       //
  std::cout << "1:" << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl; //
}
#endif
#if 1
{  
  std::chrono::system_clock::time_point start, end; //
  pcl_output->points.clear();               //
  start = std::chrono::system_clock::now(); //
  for (size_t i = 0; i < pcl_input->points.size(); ++i)
  {
    std::vector<int> v_idx;
    tree_->voxelSearch(pcl_input->points.at(i), v_idx);
    if (v_idx.empty())
      pcl_output->points.push_back(pcl_input->points.at(i));
  }
  end = std::chrono::system_clock::now();                                                                       //
  std::cout << "2:" << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl; //
}
#endif
#if 0
{
  std::chrono::system_clock::time_point start, end; //
  pcl_output->points.clear();               //
  start = std::chrono::system_clock::now(); //
  for (size_t i = 0; i < pcl_input->points.size(); ++i)
  {
    std::vector<int> v_idx;
    std::vector<float> v_squared_distance;
    if (tree_->radiusSearch(pcl_input->points.at(i), distance_threshold_, v_idx, v_squared_distance) == 0)
    {
      pcl_output->points.push_back(pcl_input->points.at(i));
    }
  }
  end = std::chrono::system_clock::now();                                                                       //
  std::cout << "3:" << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl; //
  }
#endif
#if 0
{  
  std::chrono::system_clock::time_point start, end; //
  pcl_output->points.clear();               //
  start = std::chrono::system_clock::now(); //
  for (size_t i = 0; i < pcl_input->points.size(); ++i)
  {
    int idx;
    float distance;
    tree_->approxNearestSearch (pcl_input->points.at(i), idx, distance);
    if (distance_threshold_ < (double)distance)
      pcl_output->points.push_back(pcl_input->points.at(i));
  }
  end = std::chrono::system_clock::now();                                                                       //
  std::cout << "4:" << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl; //
}
#endif
  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

void OctreeBasedCompareMapFilterNodelet::input_target_callback(const PointCloudConstPtr &map)
{
  boost::mutex::scoped_lock lock(mutex_);
  map_ptr_ = map;
  tf_input_frame_ = map->header.frame_id;
  tree_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(distance_threshold_));
  tree_->setInputCloud(map_ptr_);  
  tree_->addPointsFromInputCloud();
}

void OctreeBasedCompareMapFilterNodelet::subscribe()
{
  Filter::subscribe();
  sub_map_ = pnh_->subscribe("map", 1, &OctreeBasedCompareMapFilterNodelet::input_target_callback, this);
}

void OctreeBasedCompareMapFilterNodelet::unsubscribe()
{
  Filter::unsubscribe();
  sub_map_.shutdown();
}

void OctreeBasedCompareMapFilterNodelet::config_callback(points_preprocessor::CompareMapFilterConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (distance_threshold_ != config.distance_threshold)
  {
    distance_threshold_ = config.distance_threshold;
    tree_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(distance_threshold_));
    if (map_ptr_){
    tree_->setInputCloud(map_ptr_);
    tree_->addPointsFromInputCloud();
    }
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

PLUGINLIB_EXPORT_CLASS(points_preprocessor::OctreeBasedCompareMapFilterNodelet, nodelet::Nodelet);