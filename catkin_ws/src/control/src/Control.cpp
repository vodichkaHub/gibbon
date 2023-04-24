#include "../include/control/Control.h"

#include "ros/ros.h"
#include <boost/numeric/odeint.hpp>
using namespace boost::numeric::odeint;

namespace gibbon
{
    Control::Control(const shared_ptr<Model> &m, const double hz)
        : _m(m), _hz(hz)
    {
    }

    Control::~Control()
    {
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
} // namespace gibbon
