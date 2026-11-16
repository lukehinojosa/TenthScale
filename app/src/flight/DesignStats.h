#pragma once

#include <vector>

#include "flight/RocketDesign.h"
#include "shared/Vec3d.h"

namespace tenth
{
    // Converts specific impulse in seconds to exhaust speed; fixed by definition, not the local gravity
    constexpr double STANDARD_GRAVITY = 9.80665;

    // One press of the staging key: its decouplers fire, its engines light, and they burn the fuel they can reach
    struct StageStats
    {
        std::vector<PartIndex> engines; // lit in this stage and still attached
        PartIndex decoupler = NO_PART; // the first decoupler this stage fires, if any

        double startMass = 0.0; // kg, after this stage's decouplers have dropped what's below them
        double endMass = 0.0; // kg, once the fuel these engines reach is spent
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
        std::vector<StageStats> stages; // one per stage of the design, in firing order
        double totalDeltaVVacuum = 0.0;
        double liftoffTwr = 0.0; // of the first stage that lights an engine
        bool hasEngines = false;
        Vec3d centerOfMass;
        Vec3d centerOfPressure;
        bool stable = true; // pressure center behind the mass center, so the air keeps the nose forward
    };

    // How many decouplers lie on each part's path to the root, counting itself
    std::vector<int> sectionDepths(const RocketDesign& design);

    // The part that starts each part's section: the root, or the decoupler above it. Engines only draw fuel from
    // tanks in their own section.
    std::vector<PartIndex> sectionStarts(const RocketDesign& design);

    // Where the air's push on the rocket's side balances, estimated from each part's side area. Fins have a lot of
    // side area for their size, so low fins pull it down behind the center of mass.
    Vec3d centerOfPressure(const RocketDesign& design);

    // Goes through the stages in order. Lit engines keep burning in later stages until their fuel runs out or they're
    // dropped. surfaceGravity sets the weight for the launch pad's TWR.
    DesignStats computeStats(const RocketDesign& design, double surfaceGravity);
}
