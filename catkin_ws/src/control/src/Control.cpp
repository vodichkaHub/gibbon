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

    const int Control::findIndex(const vector<string> &ln, const string &s) const
    {
        int i = 0;
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
        const int dq1_ind(findIndex(state->name, "gibbon::base_link"));
        const int dq2_ind(findIndex(state->name, "gibbon::link_2"));
        const auto vel(state->twist);
        _dq.x() = vel.at(dq1_ind).angular.y;
        _dq.y() = vel.at(dq2_ind).angular.y;
    }

    void Control::jointStateCallback(sensor_msgs::JointStateConstPtr state)
    {
        const double q1_pev();
        const auto pos(state->position);
        const auto vel(state->velocity);
        _joint_states << pos.at(0), pos.at(1), pos.at(2), pos.at(3), pos.at(4);
        const int q2_ind(findIndex(state->name, "link_1__link_2"));
        _q.y() = pos.at(q2_ind);

        if (_buf.canTransform("gibbon__base_link", "world", ros::Time(0)))
        {
            geometry_msgs::TransformStamped base_link_tf;
            base_link_tf = _buf.lookupTransform("gibbon__base_link", "world", ros::Time(0));
            tf2::Quaternion base_link_q;
            tf2::convert(base_link_tf.transform.rotation, base_link_q);
            tf2::Matrix3x3 base_link_m(base_link_q);
            double r, p, y;
            base_link_m.getRPY(r, p, y);
            _q.x() = p;
        }
    }
} // namespace gibbon
