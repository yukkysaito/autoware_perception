#pragma once
#include <vector>
// #include <stack>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>

#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/LaneArray.h"

#include "vectormap_struct.h"


// struct LanePoint


// {
//     double tx;  // translation of X axis
//     double ty;  // translation of Y axis
//     double rz;  // rotation of Z axis (yaw)
//     double sr;  // squared radius
//     std::vector<LanePoint> goal_points;
//     // std::vector<LanePoint> goal_lane_point;
// };


// struct RootedTree
// {
//   int parent_id;
//   int parent_depth;
//   int left_child_id;
//   // int right_sibling_id;
// };

struct RootedTree
{
  int parent_id;
  int parent_depth;
  int left_child_id;
  // int right_sibling_id;
  double cumulated_s; // cumulated s in frenet coordinate from the origin lane point 
  std::vector<LanePoint> cache_past_trees;
};

class VectorMap
{
  public:

    bool load();
    // bool getyaw(double x, double y, double& yaw) const;
    std::vector<LanePoint> lane_points_;  

  private:
    // vector_map_msgs::Lane getNearestLaneWithExtendedLane(const vector_map_msgs::Lane& current_lane, 
    //                                                      const vector_map_msgs::Lane& initial_lane,
    //                                                      const std::unordered_map<int, vector_map_msgs::Lane>& lanes_map,
    //                                                      const std::unordered_map<int, vector_map_msgs::Node>& nodes_map,
    //                                                      const std::unordered_map<int, vector_map_msgs::Point>& points_map);
    
    bool getNearestLaneIDWithinRadius(
      const vector_map_msgs::Lane& current_lane, 
      const double search_radius,      
      const std::unordered_map<int, vector_map_msgs::Lane>& lanes_map,      
      const std::unordered_map<int, vector_map_msgs::Node>& nodes_map,
      const std::unordered_map<int, vector_map_msgs::Point>& points_map,
      const std::vector<int>& initial_other_lanes_id,
      std::vector<int>& other_lanes_int);
    
    vector_map_msgs::Lane getInitialLane(const vector_map_msgs::Lane& current_lane,
                              const std::unordered_map<int, vector_map_msgs::Lane>& lanes_map);
    RootedTree createTree(const vector_map_msgs::Lane& lane,
                          const int depth,
                          const LanePoint&  lane_point,
                          const std::vector<LanePoint>& past_lane_points);
    
    vector_map_msgs::Lane getNearestLaneWithLaneID(const vector_map_msgs::Lane& current_lane,
                                                  const boost::shared_ptr<const vector_map_msgs::LaneArray>& lanes,
                                                  const std::unordered_map<int, vector_map_msgs::Node>& nodes_map,
                                                  const std::unordered_map<int, vector_map_msgs::Point>& points_map,
                                                  const int target_lane_id);
    // std::vector
    std::unordered_map<int, LanePoint> lane_point_map_;
};
