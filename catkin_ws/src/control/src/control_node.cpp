#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "../include/params_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gibbon_control");
    ros::NodeHandle nh("gibbon_control");
    ros::NodeHandle nhp("~");
    gibbon::ParamsManager params(boost::make_shared<ros::NodeHandle>(nhp));

    ros::Publisher position_target(nh.advertise<std_msgs::Float64>(params.getControlTopic(), 1, true));

    ros::Rate loop_r(10);
    while (ros::ok())
    {
        std_msgs::Float64 g;
        g.data = std::sin(ros::Time::now().toSec());
        position_target.publish(g);

        ros::spinOnce();
        loop_r.sleep();
    }
    return 0;
}
