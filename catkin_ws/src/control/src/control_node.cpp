#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include "../include/control/ParamsManager.h"
#include "../include/control/Model.h"
#include "../include/control/Control.h"

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
    ros::Publisher test1(nh.advertise<std_msgs::Float64MultiArray>("test1", 1, true));

    const double hz = 1 / 1e-03;

    auto model(make_shared<gibbon::Model>());
    gibbon::Control proccess(model, hz);

    ros::Rate loop_r(hz);
    std_msgs::Float64 command_sin;
    std_msgs::Float64 command_zero;
    command_zero.data = 0.;

    gm::_state_t prev_state{0.1, 0, 0, 0};

    while (ros::ok())
    {
        std_msgs::Float64MultiArray t1;

        const auto int_step(proccess.step_ode(prev_state));
        prev_state = int_step;
        if (int_step.at(2) > 20. || int_step.at(3) > 20. || std::isnan(int_step.at(2)) || std::isnan(int_step.at(3)))
            throw std::runtime_error("Velosity has blown");

        t1.data.push_back(int_step.at(0));
        t1.data.push_back(int_step.at(1));
        t1.data.push_back(int_step.at(2));
        t1.data.push_back(int_step.at(3));
        test1.publish(t1);

        // command_sin.data = std::sin(ros::Time::now().toSec());
        // pos_target_drive.publish(command_sin);
        // pos_target_l1_dl.publish(command_zero);
        // pos_target_l1_dr.publish(command_zero);
        // pos_target_l2_dl.publish(command_zero);
        // pos_target_l2_dr.publish(command_zero);

        ros::spinOnce();
        loop_r.sleep();
    }
    return 0;
}
