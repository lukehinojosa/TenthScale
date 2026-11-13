#include "flight/DesignStats.h"

#include <algorithm>
#include <cmath>

namespace tenth
{
    std::vector<int> sectionDepths(const RocketDesign& design)
    {
        // Parents come before children, so each part's depth builds on its parent's
        std::vector<int> depths(design.partCount(), 0);
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            const PlacedPart& placed = design.part(i);
            const int parentDepth = placed.parent == NO_PART ? 0 : depths[placed.parent];
            depths[i] = parentDepth + (placed.pDefinition->separates ? 1 : 0);
        }
        return depths;
    }

    Vec3d centerOfPressure(const RocketDesign& design)
    {
        Vec3d weighted;
        double totalArea = 0.0;
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            const PlacedPart& placed = design.part(i);
            const PartShape& shape = placed.shape;
            const double area =
                shape.kind == ShapeKind::Box ? shape.height * shape.span : shape.height * (shape.topDiameter + shape.bottomDiameter) * 0.5;
            weighted += placed.position * area;
            totalArea += area;
        }
        return totalArea > 0.0 ? weighted / totalArea : Vec3d{};
    }

    namespace
    {
        // Engines firing together act like one engine whose efficiency weighs each by its share of the fuel flow
        double combinedIsp(double totalThrust, double flowSum)
        {
            return flowSum > 0.0 ? totalThrust / flowSum : 0.0;
        }

        double rocketEquation(double isp, double startMass, double endMass)
        {
            if (isp <= 0.0 || endMass <= 0.0 || startMass <= endMass)
            {
                return 0.0;
            }
            return isp * STANDARD_GRAVITY * std::log(startMass / endMass);
        }
    }

    DesignStats computeStats(const RocketDesign& design, double surfaceGravity)
    {
        DesignStats stats;
        if (design.empty())
        {
            return stats;
        }
        stats.centerOfMass = design.centerOfMass();
        stats.centerOfPressure = centerOfPressure(design);
        stats.stable = stats.centerOfPressure.y <= stats.centerOfMass.y;

        const std::vector<int> depths = sectionDepths(design);
        const int deepest = *std::max_element(depths.begin(), depths.end());
        for (int depth = deepest; depth >= 0; depth--)
        {
            StageStats stage;
            stage.depth = depth;
            double flowVacuum = 0.0;
            double flowSeaLevel = 0.0;
            for (PartIndex i = 0; i < design.partCount(); i++)
            {
                const PartDefinition& definition = *design.part(i).pDefinition;
                if (depths[i] <= depth)
                {
                    stage.startMass += definition.wetMass();
                }
                if (depths[i] != depth)
                {
                    continue;
                }
                stage.fuelMass += definition.fuelMass;
                if (definition.separates && stage.decoupler == NO_PART)
                {
                    stage.decoupler = i;
                }
                if (definition.isEngine())
                {
                    const double thrustSeaLevel = definition.thrustVacuum * definition.ispSeaLevel / definition.ispVacuum;
                    stage.engines.push_back(i);
                    stage.thrustVacuum += definition.thrustVacuum;
                    stage.thrustSeaLevel += thrustSeaLevel;
                    flowVacuum += definition.thrustVacuum / definition.ispVacuum;
                    flowSeaLevel += thrustSeaLevel / definition.ispSeaLevel;
                }
            }

            // A section without engines just rides along until its decoupler fires
            if (stage.engines.empty())
            {
                continue;
            }
            stage.endMass = stage.startMass - stage.fuelMass;
            stage.ispVacuum = combinedIsp(stage.thrustVacuum, flowVacuum);
            stage.ispSeaLevel = combinedIsp(stage.thrustSeaLevel, flowSeaLevel);
            stage.deltaVVacuum = rocketEquation(stage.ispVacuum, stage.startMass, stage.endMass);
            stage.deltaVSeaLevel = rocketEquation(stage.ispSeaLevel, stage.startMass, stage.endMass);
            stage.twrSeaLevel = stage.thrustSeaLevel / (stage.startMass * surfaceGravity);
            stats.totalDeltaVVacuum += stage.deltaVVacuum;
            stats.stages.push_back(stage);
        }
        return stats;
    }
}
