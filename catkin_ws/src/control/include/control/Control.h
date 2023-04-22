#pragma once

#include "Model.h"
#include <memory>

using namespace std;
using gm = gibbon::Model;

namespace gibbon
{
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

    public:
        Control(const shared_ptr<Model> &m, const double hz);

        ~Control();

        const gm::_state_t step_ode(gm::_state_t x0 = {std::nan("")});
        const vector<gm::_state_t> integrate_ode(gm::_state_t x0 = {std::nan("")});
    };

} // namespace gibbon
