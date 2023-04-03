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
            std::string topic_pos_target = "control";
        };
        using _params_ptr_t = std::shared_ptr<_params_t>;

    public:
        ParamsManager(const ros::NodeHandlePtr &nhp)
            : _nhp(nhp), _data(std::make_shared<_params_t>())
        {
            read();
        };

        ~ParamsManager(){};

        inline const std::string getControlTopic() const { return _data->topic_pos_target; };

        inline void read()
        {
            _nhp->getParam("topic_position_target", _data->topic_pos_target);
        };

    private:
        ros::NodeHandlePtr _nhp;
        _params_ptr_t _data;
    };
}
