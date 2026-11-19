#pragma once

#include <algorithm>
#include <cmath>

#include "shared/SolarSystem.h"

namespace tenth
{
    // Air pressure as a share of sea level: 1 on the ground, thinning out with height, and nothing past the top of the
    // atmosphere. It halves every few hundred metres on the scaled Earth, so engines reach vacuum quickly.
    inline double pressureRatio(const BodyInfo& body, double altitude)
    {
        if (body.atmosphereHeight <= 0.0 || altitude >= body.atmosphereHeight)
        {
            return 0.0;
        }
        const double scaleHeight = body.atmosphereHeight / 12.0; // roughly Earth's ratio of the two
        return std::exp(-std::max(0.0, altitude) / scaleHeight);
    }
}
