#pragma once

#include <deque>
#include <memory>

#include "Eigen/Geometry"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"

#include <tf2_ros/transform_listener.h>

#include "Model.h"

#include "mpc_solver/mpc_solve.h"

using namespace std;
using gm = gibbon::Model;

namespace gibbon
{
    using Quaterniond = Eigen::Quaternion<double>;
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
        bool _start = false;

        int _f_count = 0;
        vector<double> _mem_u;

        bool _fb1 = false;
        bool _fb2 = false;

        Vector4 _gp_states;
        Vector2 _q = {-1, -1};
        Vector2 _dq = {-1, -1};

        array<Vector3, 4> _ladder_p;
        array<Vector3, 2> _gibbon_p;
        array<Quaterniond, 2> _gibbon_q;
        array<Vector3, 2> _gibbon_lin;
        array<Vector3, 2> _gibbon_ang;

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

        void startCallback(std_msgs::BoolConstPtr state);

        inline const double calcReferenceQ2() const { return 0.52 * atan(_dq.x()); };

        const int switchArm() const;

        inline const bool gp_l1_closed() const { return fabs(_gp_states.x()) < 0.08 && fabs(_gp_states.y()) < 0.08; };
        inline const bool gp_l2_closed() const { return fabs(_gp_states.z()) < 0.08 && fabs(_gp_states.w()) < 0.08; };
        static const std_msgs::Float64 getGPCloseCommand();
        static const std_msgs::Float64 getGPOpenCommand();

        inline const bool fb_ready() const { return _fb1 && _fb2; };
        inline const bool start() const { return _start; };

        inline auto getMemU() { return _mem_u; };

        inline Vector4 getState() const {return {_q.x(), _q.y(), _dq.x(), _dq.y()};};
    };

} // namespace gibbon
