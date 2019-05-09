#include "shape_estimation/vector_map.hpp"

#include <unordered_map>
#include "ros/ros.h"
#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/DTLaneArray.h"

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

    std::unordered_map<int, vector_map_msgs::Point> points_map;
    for(const auto& point : points->data)
    {
        points_map[point.pid] = point;
    }

    lane_points_.reserve(dtlanes->data.size());
    for(const auto& dtlane : dtlanes->data)
    {
        const auto& point = points_map[dtlane.pid];
        LanePoint lane_point;
        lane_point.tx = point.ly;
        lane_point.ty = point.bx;
        lane_point.rz = dtlane.dir;
        lane_point.sr = 1.0 + std::pow(std::max(dtlane.lw, dtlane.rw), 2);
        lane_points_.emplace_back(lane_point);
    }

    return true;
}

bool VectorMap::getyaw(double x, double y, double& yaw) const
{
    double dist = 1e+10;
    bool   flag = false;
    for(const auto& lane_point : lane_points_)
    {
        double dx = x - lane_point.tx;
        double dy = y - lane_point.ty;
        double dr = (dx * dx) + (dy * dy);
        if((dr < dist) && (dr < lane_point.sr))
        {
            yaw  = lane_point.rz;
            dist = dr;
            flag = true;
        }
    }
    return flag;
}