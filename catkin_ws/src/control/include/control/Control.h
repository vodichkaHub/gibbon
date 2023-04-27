#pragma once

#include <deque>

#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include "Model.h"
#include <memory>

using namespace std;
using gm = gibbon::Model;

namespace gibbon
{
    using Vector5 = Eigen::Matrix<double, 5, 1>;
    struct push_back_state_and_time
    {
        vector<gm::_state_t> &m_states;
        vector<double> &m_times;

        push_back_state_and_time(vector<gm::_state_t> &states, vector<double> &times)
            : m_states(states), m_times(times) {}

        void operator()(const gm::_state_t &x, double t)
        {
            m_states.push_back(x);
            m_times.push_back(t);
        }
    };

    class Control
    {
    private:
        shared_ptr<Model> _m;
        double _hz = 100.;
        double _prev_time = -1.;
        double _Kp, _Kd;
        double _u = 0.;

        Vector5 _joint_states;
        Vector2 _q;
        Vector2 _dq;

        tf2_ros::Buffer _buf;
        tf2_ros::TransformListener _listener;

        const int findIndex(const vector<string> &ln, const string &s) const;

    public:
        Control(const shared_ptr<Model> &m, const double hz);

        ~Control();

        const pair<ros::Time, gm::_state_t> step_ode(gm::_state_t x0 = {std::nan("")});

        const vector<gm::_state_t> integrate_ode(gm::_state_t x0 = {std::nan("")});

        void calcU();

        inline const double &getEffort() const { return min(500., _u); };

        void jointStateCallback(sensor_msgs::JointStateConstPtr state);

        void linkStateCallback(gazebo_msgs::LinkStatesConstPtr state);

        inline const double calcReferenceQ2() const { return 0.52 * atan(_dq.x()); };
    };

} // namespace gibbon
