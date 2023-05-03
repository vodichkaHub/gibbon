#include <iostream>

#include "std_msgs/Header.h"
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
    const double hz = 1 / 1e-03;
    ros::Rate loop_r(hz);

    while (ros::Time::now().is_zero())
    {
        loop_r.sleep();
        ros::spinOnce();
    }

    ros::Publisher pos_target_drive(nh.advertise<std_msgs::Float64>(params.getControlTopicDrive(), 1, true));
    ros::Publisher pos_target_l1_dl(nh.advertise<std_msgs::Float64>(params.getControlTopicl1dl(), 1, true));
    ros::Publisher pos_target_l1_dr(nh.advertise<std_msgs::Float64>(params.getControlTopicl1dr(), 1, true));
    ros::Publisher pos_target_l2_dl(nh.advertise<std_msgs::Float64>(params.getControlTopicl2dl(), 1, true));
    ros::Publisher pos_target_l2_dr(nh.advertise<std_msgs::Float64>(params.getControlTopicl2dr(), 1, true));
    ros::Publisher test1(nh.advertise<std_msgs::Float64MultiArray>("test1", 1, true));

    auto model(make_shared<gibbon::Model>());
    gibbon::Control proccess(model, hz);

    ros::Subscriber joint_state_sub = nh.subscribe(params.getSensorJointState(), 1, &gibbon::Control::jointStateCallback, &proccess);
    ros::Subscriber link_state_sub = nh.subscribe(params.getSensorLinkState(), 1, &gibbon::Control::linkStateCallback, &proccess);
    ros::Subscriber start_sub = nh.subscribe(params.getStartTopic(), 1, &gibbon::Control::startCallback, &proccess);

    std_msgs::Float64 command_drive;
    std_msgs::Float64 command_zero;
    command_zero.data = 0.;

    while (ros::ok())
    {
        if (!proccess.fb_ready())
        {
            loop_r.sleep();
            ros::spinOnce();
            continue;
        }

        if (!proccess.start())
        {
            loop_r.sleep();
            ros::spinOnce();
            pos_target_drive.publish(command_zero);
            continue;
        }

        const int sw(proccess.switchArm());
        if (sw == 1)
        {
            pos_target_l1_dl.publish(proccess.getGPOpenCommand());
            pos_target_l1_dr.publish(proccess.getGPOpenCommand());
            pos_target_l2_dl.publish(proccess.getGPCloseCommand());
            pos_target_l2_dr.publish(proccess.getGPCloseCommand());
        }
        else if (sw == 2)
        {
            pos_target_l1_dl.publish(proccess.getGPCloseCommand());
            pos_target_l1_dr.publish(proccess.getGPCloseCommand());
            pos_target_l2_dl.publish(proccess.getGPOpenCommand());
            pos_target_l2_dr.publish(proccess.getGPOpenCommand());
        }
        else if (proccess.gp_l1_closed() && proccess.gp_l2_closed())
        {
            pos_target_l2_dl.publish(proccess.getGPOpenCommand());
            pos_target_l2_dr.publish(proccess.getGPOpenCommand());
        }

        proccess.calcU();
        command_drive.data = proccess.getEffort();
        pos_target_drive.publish(command_drive);

        ros::spinOnce();
        loop_r.sleep();
    }
    return 0;
}
