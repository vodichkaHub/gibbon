#pragma once

#include <string>
#include <memory>
#include <boost/shared_ptr.hpp>

#include "ros/node_handle.h"

namespace gibbon
{
    class ParamsManager
    {
    public:
        struct _params_t
        {
            std::string topic_pos_target_drive = "default";
            std::string topic_pos_target_l1_dl = "default";
            std::string topic_pos_target_l1_dr = "default";
            std::string topic_pos_target_l2_dl = "default";
            std::string topic_pos_target_l2_dr = "default";
            std::string topic_joint_state = "default";
            std::string topic_link_state = "default";
            std::string topic_start_cmd = "default";
        };
        using _params_ptr_t = std::shared_ptr<_params_t>;

    public:
        ParamsManager(const ros::NodeHandlePtr &nhp)
            : _nhp(nhp), _data(std::make_shared<_params_t>())
        {
            read();
        };

        ~ParamsManager(){};

        inline const std::string getControlTopicDrive() const { return _data->topic_pos_target_drive; };
        inline const std::string getControlTopicl1dl() const { return _data->topic_pos_target_l1_dl; };
        inline const std::string getControlTopicl1dr() const { return _data->topic_pos_target_l1_dr; };
        inline const std::string getControlTopicl2dl() const { return _data->topic_pos_target_l2_dl; };
        inline const std::string getControlTopicl2dr() const { return _data->topic_pos_target_l2_dr; };
        inline const std::string getSensorJointState() const { return _data->topic_joint_state; };
        inline const std::string getSensorLinkState() const { return _data->topic_link_state; };
        inline const std::string getStartTopic() const { return _data->topic_start_cmd; };

        inline void read()
        {
            _nhp->getParam("topic_position_target_drive", _data->topic_pos_target_drive);
            _nhp->getParam("topic_position_target_l1_dl", _data->topic_pos_target_l1_dl);
            _nhp->getParam("topic_position_target_l1_dr", _data->topic_pos_target_l1_dr);
            _nhp->getParam("topic_position_target_l2_dl", _data->topic_pos_target_l2_dl);
            _nhp->getParam("topic_position_target_l2_dr", _data->topic_pos_target_l2_dr);
            _nhp->getParam("topic_joint_state", _data->topic_joint_state);
            _nhp->getParam("topic_link_state", _data->topic_link_state);
            _nhp->getParam("topic_start_cmd", _data->topic_start_cmd);
        };

    private:
        ros::NodeHandlePtr _nhp;
        _params_ptr_t _data;
    };
}
