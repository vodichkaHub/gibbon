#pragma once

#include "iostream"
#include <Eigen/Dense>
#include "Math.h"
#include "boost/array.hpp"

using namespace std;
using namespace Eigen;

namespace gibbon
{
    using Vector2 = Eigen::Matrix<double, 2, 1>;
    using Vector4 = Eigen::Matrix<double, 4, 1>;
    using Matrix2 = Eigen::Matrix<double, 2, 2>;
    using Matrix4 = Eigen::Matrix<double, 4, 4>;

    class Model
    {
    public:
        using _state_t = boost::array<double, 4>;

    private:
        double g, l1, lc1, l2, lc2, m1, m2, I1, I2, damp;

        inline const Matrix2 M(const Vector2 &state) const
        {
            Matrix2 temp;
            temp << I1 + I2 + m2 * l1 * l1 + 2. * m2 * l1 * lc2 * cos(state.y()),
                I2 + m2 * l1 * lc2 * cos(state.y()),
                I2 + m2 * l1 * lc2 * cos(state.y()),
                I2;
            // Matrix2 temp;
            // temp << I1 + I2 + m2 * l1 * l1 + 2. * m2 * l1 * lc2 * cos(state.y()) + m2 * lc2 * lc2 + m1 * lc1 * lc1,
            //     I2 + m2 * l1 * lc2 * cos(state.y()) + m2 * lc2 * lc2,
            //     I2 + m2 * l1 * lc2 * cos(state.y()) + m2 * lc2 * lc2,
            //     I2 + m2 * lc2 * lc2;
            return temp;
        };

        inline const Matrix2 C(const Vector4 &state) const
        {
            Matrix2 temp;
            temp << -2. * m2 * l1 * lc2 * sin(state.y()) * state.w(),
                -1. * m2 * l1 * lc2 * sin(state.y()) * state.w(),
                m2 * l1 * lc2 * sin(state.y()) * state.z(),
                0.;
            return temp;
        };

        inline const Vector2 G(const Vector2 &state) const
        {
            // m1 * g * lc1 * cos(state.x()) + m2 * g * (l1 * cos(state.x()) + lc2 * cos(state.x() + state.y())),
            // m2 * g * lc2 * cos(state.x() + state.y()));
            return Vector2(
                m1 * g * lc1 * sin(state.x()) + m2 * g * (l1 * sin(state.x()) + lc2 * sin(state.x() + state.y())),
                m2 * g * lc2 * sin(state.x() + state.y()));
        };

    public:
        Model()
        {
            g = 9.8;
            l1 = 1.;
            lc1 = l1 / 2.;
            l2 = 1.;
            lc2 = l2 / 2.;
            m1 = 1.;
            m2 = 1.;
            I1 = 1. / 3. * m1 * l1 * l1;
            I2 = 1. / 3. * m2 * l2 * l2;
            damp = 5.;
        };

        ~Model(){};

        // inline void ode(const _state_t &x, _state_t &dxdt, const double t)
        void operator()(const _state_t &x, _state_t &dxdt, const double /* t */)
        {
            // const Vector2 x_c(modPI(x.at(0)), modPI(x.at(1)));
            const Vector2 x_c(x.at(0), x.at(1));
            const Vector2 x_v(x.at(2), x.at(3));
            const Vector4 x_vec(x.at(0), x.at(1), x.at(2), x.at(3));

            // if (x.at(2) > 100. || x.at(3) > 100. || std::isnan(x.at(2)) || std::isnan(x.at(3)))
            //     throw std::runtime_error("Velosity has blown");

            const auto inv_M(M(x_c).inverse().eval());

            const auto x3x4(inv_M * (Vector2(0, 1) * 0. - C(x_vec) * x_v - G(x_c) - damp * x_v));
            dxdt[0] = x[2];
            dxdt[1] = x[3];
            dxdt[2] = x3x4.eval().x();
            dxdt[3] = x3x4.eval().y();

            // dxdt[0] = x[1];
            // dxdt[1] = x[2];
            // dxdt[2] = x[3];
            // dxdt[3] = -x[0] - 4*x[1] - 6*x[2] - 4*x[3];
        };
    };
}