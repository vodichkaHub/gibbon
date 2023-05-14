#include "../include/control/Control.h"

#include "ros/ros.h"
#include <boost/numeric/odeint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace boost::numeric::odeint;

namespace gibbon
{
    Control::Control(const shared_ptr<Model> &m, const double hz)
        : _m(m), _hz(hz), _Kp(220), _Kd(30), _listener(_buf)
    {
    }

    Control::~Control()
    {
    }

    const std_msgs::Float64 Control::getGPCloseCommand()
    {
        std_msgs::Float64 c;
        c.data = 0.;
        return c;
    }

    const std_msgs::Float64 Control::getGPOpenCommand()
    {
        std_msgs::Float64 c;
        c.data = 3.;
        return c;
    }

    const int Control::findIndex(const vector<string> &ln, const string &s) const
    {
        int i = -1;
        for (const auto &n : ln)
        {
            if (n == s)
                break;
            else
                i++;
        }
        return i;
    }

    const pair<ros::Time, gm::_state_t> Control::step_ode(gm::_state_t x0)
    {
        if (std::isnan(x0.at(0)))
            x0 = {0., 0., 0., 0.};

        gm::_state_t res_state;
        vector<double> res_time;

        const auto cur_time(ros::Time::now());

        runge_kutta4<gm::_state_t> stepper;
        const double dt = 1. / _hz;
        stepper.do_step(*_m, x0, cur_time.toSec(), dt);

        // _prev_time = ros::Time::now().toSec();
        return make_pair(cur_time, x0);
        // x0 = {modPI(x0.at(0)), modPI(x0.at(1)), x0.at(2), x0.at(3)};
    }

    const vector<gm::_state_t> Control::integrate_ode(gm::_state_t x0)
    {
        if (std::isnan(x0.at(0)))
            x0 = {0., 0., 0., 0.};

        vector<gm::_state_t> res_state;
        vector<double> res_time;

        const double cur_time(ros::Time::now().toSec() + 1.);

        runge_kutta4<gm::_state_t> stepper;
        const double dt = 1. / _hz;
        for (double t = 0.0; t < 10.0; t += dt)
        {
            auto save(push_back_state_and_time(res_state, res_time));
            stepper.do_step(*_m, x0, t, dt);
            // x0 = {modPI(x0.at(0)), modPI(x0.at(1)), x0.at(2), x0.at(3)};
            save(x0, t);
        }

        return res_state;
    };

    void Control::calcU()
    {
        const double q1(_q.x());
        const double q2(_q.y());
        const double dq1(_dq.x());
        const double dq2(_dq.y());

        mpc_solver::mpc_solve srv;
        srv.request.header.stamp = ros::Time::now();
        srv.request.N_steps = 350;
        srv.request.x0 = {q1, q2, dq1, dq2};
        srv.request.xf = {-0.8634, 1.32896, 0, 0};
        // srv.request.xf = {-0.907369, 1.32896, 0, 0};
        srv.request.tf = 3.;
        // _mpc_solver->call(req, resp);
        ros::service::call("mpc_solve", srv);
        ROS_ERROR("MPS RES %i ", srv.response.trajectory.size());
        if (!srv.response.trajectory.empty() && _mem_u.empty())
            _mem_u = srv.response.trajectory.back().data;
        for (auto p : _mem_u)
            cerr << endl
                 << p << endl;
        const double q2_ref(calcReferenceQ2());

        const auto m_M(_m->M(_q));
        const auto m_C(_m->C(Vector4(q1, q2, dq1, dq2)));
        const auto m_G(_m->G(_q));

        const double q2_err(q2_ref - q2);
        const double dq2_err(dq2);
        const double M_bar(m_M(1, 1) - m_M(1, 0) / m_M(0, 0) * m_M(0, 1));
        const double s1((q2_err * _Kp - dq2_err * _Kd) * M_bar);

        const Vector2 C_bar_temp(m_C * _dq);
        const double C_bar(C_bar_temp.y() - m_M(1, 0) / m_M(0, 0) * C_bar_temp.x());

        const double G_bar(m_G.y() - m_M(1, 0) / m_M(0, 0) * m_G.x());

        _u = s1 + C_bar + G_bar;
    }

    void Control::linkStateCallback(gazebo_msgs::LinkStatesConstPtr state)
    {
        _fb1 = true;

        const int l1_tip_ind(findIndex(state->name, "gibbon::link_1_tip"));
        const int l2_tip_ind(findIndex(state->name, "gibbon::link_2_tip"));
        const int lad1_ind(findIndex(state->name, "ladder::ladder__link_0"));
        const int lad2_ind(findIndex(state->name, "ladder::ladder__link_1"));
        const int lad3_ind(findIndex(state->name, "ladder::ladder__link_2"));
        const int lad4_ind(findIndex(state->name, "ladder::ladder__link_3"));

        geometry_msgs::Point p;
        p = lad1_ind > 0 ? state->pose.at(lad1_ind).position : geometry_msgs::Point();
        _ladder_p.at(0) = Vector3(p.x, p.y, p.z);
        p = lad2_ind > 0 ? state->pose.at(lad2_ind).position : geometry_msgs::Point();
        _ladder_p.at(1) = Vector3(p.x, p.y, p.z);
        p = lad3_ind > 0 ? state->pose.at(lad3_ind).position : geometry_msgs::Point();
        _ladder_p.at(2) = Vector3(p.x, p.y, p.z);
        p = lad4_ind > 0 ? state->pose.at(lad4_ind).position : geometry_msgs::Point();
        _ladder_p.at(3) = Vector3(p.x, p.y, p.z);

        p = l1_tip_ind > 0 ? state->pose.at(l1_tip_ind).position : geometry_msgs::Point();
        _gibbon_p.at(0) = Vector3(p.x, p.y, p.z);
        p = l2_tip_ind > 0 ? state->pose.at(l2_tip_ind).position : geometry_msgs::Point();
        _gibbon_p.at(1) = Vector3(p.x, p.y, p.z);
        geometry_msgs::Quaternion q;
        q = l1_tip_ind > 0 ? state->pose.at(l1_tip_ind).orientation : geometry_msgs::Quaternion();
        _gibbon_q.at(0) = Quaterniond(q.w, q.x, q.y, q.z);
        q = l2_tip_ind > 0 ? state->pose.at(l2_tip_ind).orientation : geometry_msgs::Quaternion();
        _gibbon_q.at(1) = Quaterniond(q.w, q.x, q.y, q.z);

        geometry_msgs::Vector3 vl;
        vl = l1_tip_ind> 0 ? state->twist.at(l1_tip_ind).linear : geometry_msgs::Vector3();
        _gibbon_lin.at(0) = Vector3(vl.x, vl.y, vl.z);
        vl = l2_tip_ind> 0 ? state->twist.at(l2_tip_ind).linear : geometry_msgs::Vector3();
        _gibbon_lin.at(1) = Vector3(vl.x, vl.y, vl.z);

        geometry_msgs::Vector3 va; 
        va = l1_tip_ind > 0 ? state->twist.at(l1_tip_ind).angular : geometry_msgs::Vector3();
        _gibbon_ang.at(0) = Vector3(va.x, va.y, va.z);
        va = l2_tip_ind > 0 ? state->twist.at(l2_tip_ind).angular : geometry_msgs::Vector3();
        _gibbon_ang.at(1) = Vector3(va.x, va.y, va.z);

        _dq.x() = _gibbon_ang.at(0).y();
        _dq.y() = _gibbon_ang.at(1).y();
    }

    void Control::jointStateCallback(sensor_msgs::JointStateConstPtr state)
    {
        const int l1_l2_ind(findIndex(state->name, "link_1__link_2"));
        const int gp_l1_l_ind(findIndex(state->name, "link_1_left_gripper_joint"));
        const int gp_l1_r_ind(findIndex(state->name, "link_1_right_gripper_joint"));
        const int gp_l2_l_ind(findIndex(state->name, "link_2_left_gripper_joint"));
        const int gp_l2_r_ind(findIndex(state->name, "link_2_right_gripper_joint"));

        _gp_states << state->position.at(gp_l1_l_ind),
            state->position.at(gp_l1_r_ind),
            state->position.at(gp_l2_l_ind),
            state->position.at(gp_l2_r_ind);

        _q.y() = state->position.at(l1_l2_ind);

        if (_buf.canTransform("link_1_tip", "world", ros::Time(0)))
        {
            geometry_msgs::TransformStamped link_1_tip_tf;
            link_1_tip_tf = _buf.lookupTransform("link_1_tip", "world", ros::Time(0));
            tf2::Quaternion link_1_tip_q;
            tf2::convert(link_1_tip_tf.transform.rotation, link_1_tip_q);
            tf2::Matrix3x3 link_1_tip_m(link_1_tip_q);
            double r, p, y;
            link_1_tip_m.getRPY(r, p, y);
            _q.x() = p;
            _fb2 = true;
        }
    }

    void Control::startCallback(std_msgs::BoolConstPtr state)
    {
        _start = state->data;
    }

    const int Control::switchArm() const
    {
        if (gp_l1_closed())
        {
            if (distVec3(_gibbon_p.at(1), _ladder_p.at(3)) < 0.2 && _gibbon_p.at(1).x() < _ladder_p.at(3).x())
                return 1;
        }
        else
        {
            if (distVec3(_gibbon_p.at(0), _ladder_p.at(2)) < 0.2 && _gibbon_p.at(0).x() < _ladder_p.at(2).x())
                return 2;
        }
        return -1;
    }
} // namespace gibbon
