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

    std::vector<PartIndex> sectionStarts(const RocketDesign& design)
    {
        std::vector<PartIndex> starts(design.partCount(), 0);
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            const PlacedPart& placed = design.part(i);
            starts[i] = placed.parent == NO_PART || placed.pDefinition->separates ? i : starts[placed.parent];
        }
        return starts;
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

        // Drops the decoupler and everything attached through it; children come after parents, so one pass does it
        void detach(const RocketDesign& design, PartIndex decoupler, std::vector<bool>& attached)
        {
            std::vector<bool> dropped(design.partCount(), false);
            for (PartIndex i = decoupler; i < design.partCount(); i++)
            {
                const PartIndex parent = design.part(i).parent;
                dropped[i] = i == decoupler || (parent != NO_PART && dropped[parent]);
                if (dropped[i])
                {
                    attached[i] = false;
                }
            }
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

        const std::vector<PartIndex> starts = sectionStarts(design);
        std::vector<bool> attached(design.partCount(), true);
        std::vector<bool> lit(design.partCount(), false);
        std::vector<double> fuel(design.partCount(), 0.0);
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            fuel[i] = design.part(i).pDefinition->fuelMass;
        }

        for (int stageIndex = 0; stageIndex < design.stageCount(); stageIndex++)
        {
            StageStats stage;
            for (PartIndex i : design.partsInStage(stageIndex))
            {
                const PartDefinition& definition = *design.part(i).pDefinition;
                if (definition.separates && attached[i])
                {
                    if (stage.decoupler == NO_PART)
                    {
                        stage.decoupler = i;
                    }
                    detach(design, i, attached);
                }
                if (definition.isEngine())
                {
                    lit[i] = true;
                    stats.hasEngines = true;
                }
            }

            // Lit engines burn whatever fuel is left in their sections
            std::vector<PartIndex> burningSections;
            double flowVacuum = 0.0;
            double flowSeaLevel = 0.0;
            for (PartIndex i = 0; i < design.partCount(); i++)
            {
                const PartDefinition& definition = *design.part(i).pDefinition;
                if (!lit[i] || !attached[i])
                {
                    continue;
                }
                bool hasFuel = false;
                for (PartIndex j = 0; j < design.partCount(); j++)
                {
                    hasFuel = hasFuel || (attached[j] && starts[j] == starts[i] && fuel[j] > 0.0);
                }
                if (!hasFuel)
                {
                    continue;
                }
                const double thrustSeaLevel = definition.thrustVacuum * definition.ispSeaLevel / definition.ispVacuum;
                stage.engines.push_back(i);
                stage.thrustVacuum += definition.thrustVacuum;
                stage.thrustSeaLevel += thrustSeaLevel;
                flowVacuum += definition.thrustVacuum / definition.ispVacuum;
                flowSeaLevel += thrustSeaLevel / definition.ispSeaLevel;
                if (std::find(burningSections.begin(), burningSections.end(), starts[i]) == burningSections.end())
                {
                    burningSections.push_back(starts[i]);
                }
            }

            for (PartIndex i = 0; i < design.partCount(); i++)
            {
                if (!attached[i])
                {
                    continue;
                }
                stage.startMass += design.part(i).dryMass() + fuel[i];
                if (std::find(burningSections.begin(), burningSections.end(), starts[i]) != burningSections.end())
                {
                    stage.fuelMass += fuel[i];
                    fuel[i] = 0.0;
                }
            }
            stage.endMass = stage.startMass - stage.fuelMass;
            stage.ispVacuum = combinedIsp(stage.thrustVacuum, flowVacuum);
            stage.ispSeaLevel = combinedIsp(stage.thrustSeaLevel, flowSeaLevel);
            stage.deltaVVacuum = rocketEquation(stage.ispVacuum, stage.startMass, stage.endMass);
            stage.deltaVSeaLevel = rocketEquation(stage.ispSeaLevel, stage.startMass, stage.endMass);
            stage.twrSeaLevel = stage.thrustSeaLevel / (stage.startMass * surfaceGravity);
            if (!stage.engines.empty() && stats.liftoffTwr <= 0.0)
            {
                stats.liftoffTwr = stage.twrSeaLevel;
            }
            stats.totalDeltaVVacuum += stage.deltaVVacuum;
            stats.stages.push_back(stage);
        }
        return stats;
    }
}
