#pragma once
#include <vector>

struct LanePoint
{
    double tx;  // translation of X axis
    double ty;  // translation of Y axis
    double rz;  // rotation of Z axis (yaw)
    double sr;  // squared radius
};

class VectorMap
{
    public:

        bool load();
        bool getyaw(double x, double y, double& yaw) const;

    private:

        std::vector<LanePoint> lane_points_;
};
