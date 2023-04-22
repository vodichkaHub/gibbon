#pragma once
#include <math.h>

namespace gibbon
{
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
} // namespace gibbon
