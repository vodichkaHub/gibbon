#pragma once

#include <math.h>
#include <Eigen/Dense>

namespace gibbon
{
    using Vector2 = Eigen::Matrix<double, 2, 1>;
    using Vector3 = Eigen::Matrix<double, 3, 1>;
    using Vector4 = Eigen::Matrix<double, 4, 1>;
    using Vector5 = Eigen::Matrix<double, 5, 1>;
    using Matrix2 = Eigen::Matrix<double, 2, 2>;
    using Matrix4 = Eigen::Matrix<double, 4, 4>;

    inline double modPI(double x)
    {
        x = fmod(x, M_PI);
        if (x < 0)
            x += M_PI;
        return x;
    };

    inline double mod2PI(double x)
    {
        x = fmod(x, 2. * M_PI);
        if (x < 0)
            x += 2. * M_PI;
        return x;
    };

    inline const double distVec3(const Vector3 &l, const Vector3 &r)
    {
        const double dx(l.x() - r.x());
        const double dy(l.y() - r.y());
        const double dz(l.z() - r.z());
        return sqrt(dx * dx + dy * dy + dz * dz);
    }
} // namespace gibbon
