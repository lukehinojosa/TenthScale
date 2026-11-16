#pragma once

#include <vector>

#include "flight/RocketDesign.h"
#include "shared/Vec3d.h"

namespace tenth
{
    // Converts specific impulse in seconds to exhaust speed; fixed by definition, not the local gravity
    constexpr double STANDARD_GRAVITY = 9.80665;

    // One burn: the engines that fire together, and the decoupler that drops them once they run dry
    struct StageStats
    {
        int depth = 0; // how many decouplers sit between these parts and the root
        std::vector<PartIndex> engines;
        PartIndex decoupler = NO_PART; // NO_PART for the last stage, which stays attached to the pod

        double startMass = 0.0; // kg, with this stage's tanks full and lower stages already dropped
        double endMass = 0.0; // kg, once this stage's fuel is spent
        double fuelMass = 0.0;
        double thrustVacuum = 0.0;
        double thrustSeaLevel = 0.0;
        double ispVacuum = 0.0;
        double ispSeaLevel = 0.0;
        double deltaVVacuum = 0.0;
        double deltaVSeaLevel = 0.0;
        double twrSeaLevel = 0.0; // thrust against weight on the launch pad
    };

    struct DesignStats
    {
        std::vector<StageStats> stages; // in firing order, first burn first
        double totalDeltaVVacuum = 0.0;
        Vec3d centerOfMass;
        Vec3d centerOfPressure;
        bool stable = true; // pressure center behind the mass center, so the air keeps the nose forward
    };

    // How many decouplers lie on each part's path to the root, counting itself. Decouplers split the rocket into
    // sections, and engines only draw fuel from tanks in their own section.
    std::vector<int> sectionDepths(const RocketDesign& design);

    // Where the air's push on the rocket's side balances, estimated from each part's side area. Fins have a lot of
    // side area for their size, so low fins pull it down behind the center of mass.
    Vec3d centerOfPressure(const RocketDesign& design);

    // Stages fire from the deepest section up. surfaceGravity sets the weight for the launch pad's TWR.
    DesignStats computeStats(const RocketDesign& design, double surfaceGravity);
}
