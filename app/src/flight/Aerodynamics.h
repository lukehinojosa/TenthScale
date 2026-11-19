#pragma once

#include "flight/RocketDesign.h"
#include "shared/Vec3d.h"

namespace tenth
{
    constexpr double SEA_LEVEL_DENSITY = 1.225; // kg/m^3
    constexpr double PARACHUTE_DIAMETER = 12.0; // metres, fully open
    constexpr double PARACHUTE_RISE = 7.0; // how far the canopy flies out from the top of its part
    constexpr double PARACHUTE_OPEN_TIME = 2.5; // seconds from packed to fully open, once there's air to fill it

    // The air's push on one part, in the design's axes, for the part moving at velocity (also in the design's axes)
    // through still air of this density. Round parts feel their side outline and only the faces left uncovered;
    // fins and legs are flat plates pushed only by the flow across them.
    Vec3d partDrag(const RocketDesign& design, PartIndex index, const Vec3d& velocity, double density);

    // The pull of a parachute open by this share, 0 to 1, moving at velocity through air of this density
    Vec3d parachuteDrag(double opening, const Vec3d& velocity, double density);
}
