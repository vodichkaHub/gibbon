#pragma once

#include <vector>
#include "ros/ros.h"

#include "nav_msgs/Path.h"
#include "std_msgs/Float64MultiArray.h"


namespace gibbon::vis
{
    using namespace nav_msgs;
    using namespace std_msgs;
    using namespace std;

    inline const Path trajectory2Path(const Float64MultiArray traj)
    {
        Path result;
        result.header.frame_id = "world";
        result.header.stamp = ros::Time::now();

        result.poses.reserve(traj.data.size());
        // traj.data;
        // result.poses;
    }
} // namespace gibbon::vis
