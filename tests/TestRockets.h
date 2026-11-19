#pragma once

#include <numbers>

#include <flight/PartCatalog.h>
#include <flight/RocketDesign.h>

namespace tenth
{
    // A small two-stage rocket for tests: pod and nose cone on a lander with legs, a decoupler, and a finned booster
    inline RocketDesign twoStageRocket(const PartCatalog& catalog)
    {
        RocketDesign design;
        auto find = [&catalog](const char* id) -> const PartDefinition&
        {
            return *catalog.find(id);
        };
        auto single = [&find](const char* id)
        {
            RocketDesign part;
            part.setRoot(find(id));
            return part;
        };

        const PartIndex pod = design.setRoot(find("pod"));
        design.attachStack(pod, Node::Top, find("nose-cone"));
        const PartIndex upperTank = design.attachStack(pod, Node::Bottom, find("tank-large"));
        const PartIndex vacuum = design.attachStack(upperTank, Node::Bottom, find("engine-vacuum"));
        design.attachDesignRadial(vacuum, 0.0, 0.0, single("landing-leg"), 4);
        PartIndex below = design.attachStack(vacuum, Node::Bottom, find("decoupler"));
        for (int i = 0; i < 2; i++)
        {
            below = design.attachStack(below, Node::Bottom, find("tank-large"));
        }
        const PartIndex lifter = design.attachStack(below, Node::Bottom, find("engine-lifter"));
        const double finHeight = find("fin").shape.height;
        const double lifterHeight = find("engine-lifter").shape.height;
        design.attachDesignRadial(lifter, std::numbers::pi / 4.0, (finHeight - lifterHeight) * 0.5, single("fin"), 4);
        design.autoStage();
        return design;
    }
}
