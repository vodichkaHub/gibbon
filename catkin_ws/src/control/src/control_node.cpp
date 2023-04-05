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

    ros::Publisher pos_target_drive(nh.advertise<std_msgs::Float64>(params.getControlTopicDrive(), 1, true));
    ros::Publisher pos_target_l1_dl(nh.advertise<std_msgs::Float64>(params.getControlTopicl1dl(), 1, true));
    ros::Publisher pos_target_l1_dr(nh.advertise<std_msgs::Float64>(params.getControlTopicl1dr(), 1, true));
    ros::Publisher pos_target_l2_dl(nh.advertise<std_msgs::Float64>(params.getControlTopicl2dl(), 1, true));
    ros::Publisher pos_target_l2_dr(nh.advertise<std_msgs::Float64>(params.getControlTopicl2dr(), 1, true));

    ros::Rate loop_r(10);
    std_msgs::Float64 command_sin;
    std_msgs::Float64 command_zero;
    command_zero.data = 0.;
    while (ros::ok())
    {
        command_sin.data = std::sin(ros::Time::now().toSec());
        pos_target_drive.publish(command_sin);
        pos_target_l1_dl.publish(command_zero);
        pos_target_l1_dr.publish(command_zero);
        pos_target_l2_dl.publish(command_zero);
        pos_target_l2_dr.publish(command_zero);

        ros::spinOnce();
        loop_r.sleep();
    }
    return 0;
}
