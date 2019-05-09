#include "vectormap_ros.h"

#include "ros/ros.h"

bool VectorMap::load()
{
  auto points = ros::topic::waitForMessage<vector_map_msgs::PointArray>("/vector_map_info/point", ros::Duration());
  if(!points)
  {
      return false;
  }

  auto dtlanes = ros::topic::waitForMessage<vector_map_msgs::DTLaneArray>("/vector_map_info/dtlane", ros::Duration());
  if(!dtlanes)
  {
      return false;
  }

  auto nodes = ros::topic::waitForMessage<vector_map_msgs::NodeArray>("/vector_map_info/node", ros::Duration());
  if(!nodes)
  {
      return false;
  }

  auto lanes = ros::topic::waitForMessage<vector_map_msgs::LaneArray>("/vector_map_info/lane", ros::Duration());
  if(!lanes)
  {
      return false;
  }

  std::unordered_map<int, vector_map_msgs::Point> points_map;
  for(const auto& point : points->data)
  {
      points_map[point.pid] = point;
  }

  std::unordered_map<int, vector_map_msgs::DTLane> dtlanes_map;
  for(const auto& dtlane : dtlanes->data)
  {
    dtlanes_map[dtlane.did] = dtlane;
  }

  std::unordered_map<int, vector_map_msgs::Node> nodes_map;
  for(const auto& node : nodes->data)
  {
    nodes_map[node.nid] = node;
  }

  std::unordered_map<int, vector_map_msgs::Lane> lanes_map;
  for(const auto& lane : lanes->data)
  {
    lanes_map[lane.lnid] = lane;
  }

  for(const auto& lane : lanes->data)
  {
    //initial lane point
    LanePoint lane_point;
    const auto& node = nodes_map[lane.bnid];
    const auto& point = points_map[node.pid];
    const auto& dtlane = dtlanes_map[lane.did];
    lane_point.tx = point.ly;
    lane_point.ty = point.bx;
    lane_point.rz = dtlane.dir;
    lane_point.sr = 1.0 + std::pow(std::max(dtlane.lw, dtlane.rw), 2);
    lane_point.type = GoalLaneType::NotGoal;
    if(lane.clossid == 0)
    {
      lane_point.is_in_intersection = false;
    }
    else
    {
      lane_point.is_in_intersection = true;
    }

    //TODO: rosparam
    const int num_search_point = 10;
    std::queue<RootedTree> queue;

    const int initial_depth = 0;
    //TODO: not good
    std::vector<LanePoint> lane_point_vector;
    lane_point.cumulated_s = 0;
    // lane_point_vector.push_back(lane_point);
    queue.push(createTree(lane, 
                          initial_depth,
                          lane_point,
                          lane_point_vector));
    
    // outside intersection
    if(lane.lcnt != 0)
    {
      // std::cerr << "num lane aaa "<< lane.lcnt << std::endl;
      for (int i = 1; i <= lane.lcnt; i++)
      {
        if(i == lane.lno || 
           std::abs(lane.lno - i) > 1)
        {
          continue;
        }
        const auto & another_lane = getNearestLaneWithLaneID(lane, 
                                                             lanes, 
                                                             nodes_map, 
                                                             points_map, 
                                                             i);

        //another lane point
        LanePoint another_lane_point;
        const auto& another_node = nodes_map[another_lane.bnid];
        const auto& another_point = points_map[another_node.pid];
        const auto& another_dtlane = dtlanes_map[another_lane.did];
        another_lane_point.tx = another_point.ly;
        another_lane_point.ty = another_point.bx;
        another_lane_point.rz = another_dtlane.dir;
        another_lane_point.sr = 1.0 + std::pow(std::max(another_dtlane.lw, another_dtlane.rw), 2);
        another_lane_point.type = GoalLaneType::NotGoal;
        if(another_lane.clossid == 0)
        {
          another_lane_point.is_in_intersection = false;
        }
        else
        {
          another_lane_point.is_in_intersection = true;
        }
        //TODO: fix later
        std::vector<LanePoint> lane_point_vector;
        // lane_point_vector.push_back(another_lane_point);
        another_lane_point.cumulated_s = 0;
        queue.push(createTree(another_lane, 
                              initial_depth, 
                              another_lane_point, 
                              lane_point_vector));
      }
    }
    
    //inside intersection
    if(lane.clossid != 0)
    {
      std::vector<int> initial_other_lanes_id;
      const auto& initial_lane = getInitialLane(lane, lanes_map);
      for (int i = 1; i <= initial_lane.lcnt; i++)
      {
        if(i == initial_lane.lno || 
           std::abs(initial_lane.lno - i) > 1)
        {
          continue;
        }
        const auto & another_lane = getNearestLaneWithLaneID(initial_lane, 
                                                             lanes, 
                                                             nodes_map, 
                                                             points_map, 
                                                             i);
        // std::cerr << "another lane closs id " << another_lane.clossid << std::endl;              
        initial_other_lanes_id.push_back(another_lane.lnid);                
      }
      // std::cerr << "other lane num " << initial_other_lanes_id.size() << std::endl;
      
      const double search_radius = 1.5;
      std::vector<int> other_lanes_id;
      getNearestLaneIDWithinRadius(lane, 
                                   1.5,
                                   lanes_map, 
                                   nodes_map, 
                                   points_map, 
                                   initial_other_lanes_id,
                                   other_lanes_id);
      // std::cerr << "other num " << other_lanes_id.size() << std::endl;
      
    }

    //breadth first search 
    while(!queue.empty())
    {
      RootedTree current_tree = queue.front(); queue.pop();
      if(current_tree.parent_depth == num_search_point || 
          current_tree.left_child_id == 0)
      {
        // std::cerr << "sss "<<current_tree.cumulated_s << std::endl;
        //TODO: dedundant
        LanePoint goal_lane_point;
        const auto& goal_lane = lanes_map[current_tree.parent_id];
        const auto& goal_node = nodes_map[goal_lane.bnid];
        const auto& goal_point = points_map[goal_node.pid];
        goal_lane_point.tx = goal_point.ly;
        goal_lane_point.ty = goal_point.bx;
        const auto& goal_dtlane = dtlanes_map[goal_lane.did];
        goal_lane_point.rz = goal_dtlane.dir;
        goal_lane_point.sr = 1.0 + std::pow(std::max(goal_dtlane.lw, goal_dtlane.rw), 2);
        if(goal_lane.clossid == 0)
        {
          goal_lane_point.is_in_intersection = false;
        }
        else
        {
          goal_lane_point.is_in_intersection = true;
        }
        
        // not appropriate implementation
        // it is better to have frenet coord for each frame 
        // for example if(diff_lane_num == -1 )-> Left is not always correct 
        // when trajectory going from lane to intersection
        int diff_lane_num = goal_lane.lno - lane.lno;
        if(diff_lane_num == -1)
        {
          goal_lane_point.type = GoalLaneType::Left;
        }
        else if(diff_lane_num == 0)
        {
          goal_lane_point.type = GoalLaneType::Straight;
        }
        else if(diff_lane_num == 1)
        {
          goal_lane_point.type = GoalLaneType::Right;
        }
        else
        {
          goal_lane_point.type = GoalLaneType::NotGoal;
        //   std::cerr << "goal lane " << goal_lane.lno << " current lane " << lane.lno << std::endl; 
        // std::cerr << "diff lane num " << diff_lane_num << std::endl;
        //   std::cerr << "something wrong " << std::endl;
        }
        lane_point.goal_points.push_back(goal_lane_point);
        lane_point.goal_paths.push_back(current_tree.cache_past_trees);
        continue;
      }


      // int next_depth = current_tree.parent_depth ++;
      // current_tree.parent_depth ++;
      int next_depth = ++current_tree.parent_depth;
      int child_tree_id = current_tree.left_child_id;
      const auto& child_lane = lanes_map[child_tree_id];

      // //TODO: dedundant
      LanePoint child_lane_point;
      const auto& child_node = nodes_map[child_lane.bnid];
      const auto& child_point = points_map[child_node.pid];
      child_lane_point.tx = child_point.ly;
      child_lane_point.ty = child_point.bx;
      const auto& child_dtlane = dtlanes_map[child_lane.did];
      child_lane_point.rz = child_dtlane.dir;
      child_lane_point.sr = 1.0 + std::pow(std::max(child_dtlane.lw, child_dtlane.rw), 2);
      if(child_lane.clossid == 0)
      {
        child_lane_point.is_in_intersection = false;
      }
      else
      {
        child_lane_point.is_in_intersection = true;
      }

      // const auto& current_lane = lanes_map[current_tree.parent_id];
      // const auto& current_node = nodes_map[current_lane.bnid];
      // const auto& current_point = points_map[current_node.pid];
      // const double 
      // const double current_node = current_lane.
      // double cumulated_s = 0;
      if(current_tree.cache_past_trees.size() == 0)
      {
        std::cerr << "something wrong" << std::endl;
        return false;
      }
      const auto& previous_lane_point = current_tree.cache_past_trees.back();
      const double distance = 
                  std::sqrt((std::pow(child_lane_point.tx - previous_lane_point.tx,2)+
                  std::pow(child_lane_point.ty - previous_lane_point.ty,2)));
      const double cumulated_s = distance + current_tree.cumulated_s;

      child_lane_point.cumulated_s = cumulated_s;
      
      queue.push(createTree(child_lane, 
                            next_depth, 
                            child_lane_point, 
                            current_tree.cache_past_trees
                            ));
      
      
      // inside intersection
      // const auto& current_lane = lanes_map[current_tree.parent_id];
      // if(current_lane.clossid != 0)
      // {
      //   if(current_lane.flid2 != 0)
      //   {
      //     const auto& sibling_lane = lanes_map[current_lane.flid2];
      //     queue.push(createTree(sibling_lane, next_depth));
      //   }
      //   if(current_lane.flid3 != 0)
      //   {
      //     const auto& sibling_lane = lanes_map[current_lane.flid3];
      //     queue.push(createTree(sibling_lane, next_depth));
      //   }
      //   if(current_lane.flid4 != 0)
      //   {
      //     const auto& sibling_lane = lanes_map[current_lane.flid4];
      //     queue.push(createTree(sibling_lane, next_depth));
      //   }
      // }
      
    } 
    //sanity check
    if(lane_point.goal_points.size() > 3)
    {
      std::cerr << "this lane has more than 3 goals " << std::endl;
    }
    // if(lane_point.goal_points.size() != 1)
    // {
    //   std::cerr << lane_point.goal_points.size() << std::endl;
    // }  
    lane_points_.emplace_back(lane_point);
  }
  return true;
}

bool VectorMap::getNearestLaneIDWithinRadius(
      const vector_map_msgs::Lane& current_lane,
      const double search_radius, 
      const std::unordered_map<int, vector_map_msgs::Lane>& lanes_map,
      const std::unordered_map<int, vector_map_msgs::Node>& nodes_map,
      const std::unordered_map<int, vector_map_msgs::Point>& points_map,
      const std::vector<int>& initial_other_lanes_id,
      std::vector<int>& other_lanes_id)
{
  const auto & current_node = nodes_map.at(current_lane.bnid);
  const auto& current_point = points_map.at(current_node.pid);
  const double cx = current_point.ly;
  const double cy = current_point.bx;
  const int target_intersection_id = current_lane.clossid;
  std::vector<int> candidate_lanes;
  for(const auto& it_lane: lanes_map)
  {
    const auto& lane = it_lane.second;
    if(lane.clossid == target_intersection_id)
    {
      const auto& target_node = nodes_map.at(lane.bnid);
      const auto& target_point = points_map.at(target_node.pid);
      const double tx = target_point.ly;
      const double ty = target_point.bx;
      const double distance = std::sqrt(std::pow((cx - tx), 2) + std::pow((cy-ty),2));
      if(distance < search_radius)
      {
        candidate_lanes.push_back(lane.lnid);
      }
    }
  }
  
  vector_map_msgs::Lane back_lane;
  for(const auto& id: candidate_lanes)
  {
    bool keep_searching = true;
    int back_lane_id = id;
    std::vector<int> not_ego_lanes = candidate_lanes;
    not_ego_lanes.erase(std::remove(not_ego_lanes.begin(), not_ego_lanes.end(), back_lane_id),
         not_ego_lanes.end());
    // std::cerr << "back lane id " << back_lane_id << std::endl;
    while(keep_searching)
    {
      back_lane = lanes_map.at(back_lane_id);
      if(std::binary_search(not_ego_lanes.begin(), not_ego_lanes.end(), back_lane_id))
      {
        keep_searching = false;
      }
      else
      {
        // if(back_lane.clossid == 0 ||
        //    std::binary_search(initial_other_lanes_id.begin(), initial_other_lanes_id.end(), back_lane_id))
        if(back_lane.clossid == 0)
        {
          // std::cerr << "target back lane id " << back_lane_id << std::endl;
          // for(const auto& i: initial_other_lanes_id)
          // {
          //   std::cerr << "compare id " << i  << std::endl;
          // }
          other_lanes_id.push_back(id);
          keep_searching = false;
        }
        else
        {
          // not considering backwards braching lane
          back_lane_id = back_lane.blid;
          if(back_lane_id == 0)
          {
            keep_searching = false;
          }
        }
      // std::cerr << "keep searc " << keep_searching << std::endl;
      }
      // std::cerr << "processed back lane cross " << back_lane.clossid << std::endl;
      // std::cerr << "processed back lane id " << back_lane_id << std::endl;
    }
  }
  return true;
}

// vector_map_msgs::Lane  VectorMap::getNearestLaneWithExtendedLane(
//   const vector_map_msgs::Lane& current_lane, 
//   const vector_map_msgs::Lane& another_initial_lane,
//   const std::unordered_map<int, vector_map_msgs::Lane>& lanes_map,
//   const std::unordered_map<int, vector_map_msgs::Node>& nodes_map,
//   const std::unordered_map<int, vector_map_msgs::Point>& points_map)
// {
//   const auto & current_node = nodes_map.at(current_lane.bnid);
//   const auto& current_point = points_map.at(current_node.pid);
//   const double cx = current_point.ly;
//   const double cy = current_point.bx;
//   const int target_intersection_id = current_lane.clossid;
//   bool keep_searching = true;
//   while(keep_searching)
//   {
//     const int next_lane_id = another_initial_lane.flid;
//     const auto& next_lane = lanes_map.at(next_lane_id);
//     if(lane.clossid == target_intersection_id)
//     {
      
//     }
//   }
  
//   // for(const auto& lane: lanes->data)
//   // {
//   //   if(lane.clossid == target_intersection_id)
//   //   {
      
//   //   }
//   // }
//   return current_lane;
// }

RootedTree VectorMap::createTree(const vector_map_msgs::Lane& lane, 
                                 const int depth,
                                 const LanePoint& lane_point,
                                 const std::vector<LanePoint>& past_lane_points)

{
  RootedTree tree;
  tree.parent_id = lane.lnid;
  tree.parent_depth = depth;
  tree.left_child_id = lane.flid;
  tree.cache_past_trees = past_lane_points;
  tree.cache_past_trees.push_back(lane_point);
  tree.cumulated_s = lane_point.cumulated_s;
  return tree;
}

vector_map_msgs::Lane VectorMap::getInitialLane(const vector_map_msgs::Lane& current_lane,
                                  const std::unordered_map<int, vector_map_msgs::Lane>& lanes_map)
{
  const double current_intersection_id = current_lane.clossid;
  vector_map_msgs::Lane initial_lane;
  bool keep_searching = true;
  int back_lane_id = current_lane.blid;
  while(keep_searching)
  {
    if(back_lane_id == 0)
    {
      keep_searching = false;
      continue;
    }
    // const auto& back_lane_id = current_lane.blid;
    // std::cerr << "back lane id " << back_lane_id << std::endl;
    const auto& back_lane = lanes_map.at(back_lane_id);
    // std::cerr << "backlane cross id " << back_lane.clossid << std::endl;
    // std::cerr << "backlane cross blid " << back_lane.blid << std::endl;
    if(back_lane.clossid == 0)
    {
      initial_lane = back_lane;
      keep_searching = false;
    }
    back_lane_id = back_lane.blid;
  }
  return initial_lane;
}

vector_map_msgs::Lane VectorMap::getNearestLaneWithLaneID(const vector_map_msgs::Lane& current_lane,
                                                          const boost::shared_ptr<const vector_map_msgs::LaneArray>& lanes,
                                                          const std::unordered_map<int, vector_map_msgs::Node>& nodes_map,
                                                          const std::unordered_map<int, vector_map_msgs::Point>& points_map,
                                                          const int target_lane_id)
{
  const auto & current_node = nodes_map.at(current_lane.bnid);
  const auto& current_point = points_map.at(current_node.pid);
  const double cx = current_point.ly;
  const double cy = current_point.bx;
  const int target_road_segment_id = current_lane.roadsecid;
  vector_map_msgs::Lane nearest_lane;
  double min_dist = 1e+10;
  for(const auto& lane: lanes->data)
  {
    const int road_segment_id = lane.roadsecid;
    const int lane_id = lane.lno;
    if(road_segment_id == target_road_segment_id &&
       lane_id == target_lane_id)
    {
      const auto& target_node = nodes_map.at(lane.bnid);
      const auto& target_point = points_map.at(target_node.pid);
      const double tx = target_point.ly;
      const double ty = target_point.bx;
      double distance = std::sqrt(std::pow((tx - cx),2) + std::pow((ty - cy), 2));
      if(distance < min_dist)
      {
        nearest_lane = lane;
        min_dist = distance;
      }
    }
  }
  return nearest_lane;
}