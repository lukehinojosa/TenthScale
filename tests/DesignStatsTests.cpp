#include <doctest/doctest.h>

#include <cmath>
#include <vector>

#include <flight/DesignStats.h>
#include <flight/PartCatalog.h>

using namespace tenth;

namespace
{
    const PartCatalog& catalog()
    {
        static const PartCatalog standard = PartCatalog::standard();
        return standard;
    }

    const PartDefinition& def(const char* id)
    {
        const PartDefinition* pDefinition = catalog().find(id);
        REQUIRE(pDefinition != nullptr);
        return *pDefinition;
    }

    // Pod, medium tank, vacuum engine | decoupler, large tank, lifter engine
    RocketDesign twoStage()
    {
        RocketDesign design;
        const PartIndex pod = design.setRoot(def("pod"));
        const PartIndex upperTank = design.attachStack(pod, Node::Bottom, def("tank-medium"));
        const PartIndex upperEngine = design.attachStack(upperTank, Node::Bottom, def("engine-vacuum"));
        const PartIndex decoupler = design.attachStack(upperEngine, Node::Bottom, def("decoupler"));
        const PartIndex lowerTank = design.attachStack(decoupler, Node::Bottom, def("tank-large"));
        design.attachStack(lowerTank, Node::Bottom, def("engine-lifter"));
        design.updateStaging();
        return design;
    }
}

TEST_CASE("Decouplers split the rocket into sections, counting themselves with the part below")
{
    const std::vector<int> depths = sectionDepths(twoStage());
    CHECK(depths == std::vector<int>{ 0, 0, 0, 1, 1, 1 });
    CHECK(sectionStarts(twoStage()) == std::vector<PartIndex>{ 0, 0, 0, 3, 3, 3 });
    CHECK(sectionDepths(RocketDesign()).empty());
}

TEST_CASE("Auto-staging fires the bottom engines, then the decoupler with the engines above, then the parachute")
{
    const RocketDesign design = twoStage();
    REQUIRE(design.stageCount() == 3);
    CHECK(design.partsInStage(0) == std::vector<PartIndex>{ 5 });
    CHECK(design.partsInStage(1) == std::vector<PartIndex>{ 2, 3 });
    CHECK(design.partsInStage(2) == std::vector<PartIndex>{ 0 });
    CHECK_FALSE(design.customStaging());

    // Tanks aren't staged
    CHECK(design.part(1).stage == NO_STAGE);
}

TEST_CASE("A two-stage rocket fires its bottom stage first, then drops it and fires the top")
{
    const RocketDesign design = twoStage();
    const double g = 9.82;
    const DesignStats stats = computeStats(design, g);
    REQUIRE(stats.stages.size() == 3);

    const StageStats& first = stats.stages[0];
    REQUIRE(first.engines.size() == 1);
    CHECK(first.engines[0] == 5);
    CHECK(first.decoupler == NO_PART);
    CHECK(first.startMass == doctest::Approx(design.totalMass()));
    CHECK(first.fuelMass == doctest::Approx(4000.0));
    CHECK(first.endMass == doctest::Approx(design.totalMass() - 4000.0));
    CHECK(first.deltaVVacuum == doctest::Approx(310.0 * STANDARD_GRAVITY * std::log(first.startMass / first.endMass)));
    CHECK(first.deltaVSeaLevel < first.deltaVVacuum);
    CHECK(first.twrSeaLevel == doctest::Approx(215000.0 * 280.0 / 310.0 / (design.totalMass() * g)));
    CHECK(stats.liftoffTwr == doctest::Approx(first.twrSeaLevel));

    // The decoupler drops the lower stage, so the upper one starts with just its own parts and a full tank
    const StageStats& second = stats.stages[1];
    CHECK(second.decoupler == 3);
    REQUIRE(second.engines.size() == 1);
    CHECK(second.engines[0] == 2);
    CHECK(second.startMass == doctest::Approx(800.0 + 2250.0 + 500.0));
    CHECK(second.endMass == doctest::Approx(800.0 + 250.0 + 500.0));
    CHECK(second.deltaVVacuum == doctest::Approx(345.0 * STANDARD_GRAVITY * std::log(3550.0 / 1550.0)));

    // The parachute's stage has no fuel left to burn
    CHECK(stats.stages[2].engines.empty());
    CHECK(stats.stages[2].deltaVVacuum == doctest::Approx(0.0));

    CHECK(stats.totalDeltaVVacuum == doctest::Approx(first.deltaVVacuum + second.deltaVVacuum));
}

TEST_CASE("Engines without fuel in their own section give no delta-v")
{
    // The tank is above the decoupler and the engine below it, so the engine can't reach the fuel
    RocketDesign design;
    const PartIndex pod = design.setRoot(def("pod"));
    const PartIndex tank = design.attachStack(pod, Node::Bottom, def("tank-small"));
    const PartIndex decoupler = design.attachStack(tank, Node::Bottom, def("decoupler"));
    design.attachStack(decoupler, Node::Bottom, def("engine-lifter"));
    design.updateStaging();

    const DesignStats stats = computeStats(design, 9.82);
    REQUIRE(stats.stages.size() == 3);
    CHECK(stats.stages[0].fuelMass == doctest::Approx(0.0));
    CHECK(stats.stages[0].deltaVVacuum == doctest::Approx(0.0));
    CHECK(stats.totalDeltaVVacuum == doctest::Approx(0.0));
    CHECK(stats.hasEngines);

    CHECK(computeStats(RocketDesign(), 9.82).stages.empty());
}

TEST_CASE("Engines firing together share one combined efficiency")
{
    // Two different engines in one stage: their combined efficiency lies between theirs
    RocketDesign design;
    const PartIndex pod = design.setRoot(def("pod"));
    const PartIndex tank = design.attachStack(pod, Node::Bottom, def("tank-large"));
    design.attachStack(tank, Node::Bottom, def("engine-lifter"));
    design.attachStack(pod, Node::Top, def("nose-cone"));
    RocketDesign vacuum;
    vacuum.setRoot(def("engine-vacuum"));
    RocketDesign withSecond = design;
    withSecond.attachDesign(2, Node::Bottom, vacuum);
    withSecond.updateStaging();

    const StageStats stage = computeStats(withSecond, 9.82).stages.at(0);
    REQUIRE(stage.engines.size() == 2);
    CHECK(stage.thrustVacuum == doctest::Approx(275000.0));
    CHECK(stage.ispVacuum > 310.0);
    CHECK(stage.ispVacuum < 345.0);
    CHECK(stage.ispVacuum == doctest::Approx(275000.0 / (215000.0 / 310.0 + 60000.0 / 345.0)));
}

TEST_CASE("Moving an engine to another stage changes when it burns, and keeps the player's staging")
{
    RocketDesign design = twoStage();

    // Lighting the upper engine with the lower one: both burn in stage 1, and the decoupler's stage burns nothing
    design.moveToStage({ 2 }, 0);
    CHECK(design.customStaging());
    const DesignStats stats = computeStats(design, 9.82);
    REQUIRE(stats.stages.size() == 3);
    CHECK(stats.stages[0].engines.size() == 2);
    CHECK(stats.stages[0].fuelMass == doctest::Approx(6000.0));
    CHECK(stats.stages[1].engines.empty());

    // Adding a part keeps the moved engine where it is
    design.attachStack(0, Node::Top, def("nose-cone"));
    design.updateStaging();
    CHECK(design.part(2).stage == 0);

    // Auto-staging puts it back
    design.autoStage();
    CHECK(design.part(2).stage == 1);
    CHECK_FALSE(design.customStaging());
}

TEST_CASE("Stages can be added anywhere, but only empty ones removed")
{
    RocketDesign design = twoStage();
    design.insertStage(1);
    REQUIRE(design.stageCount() == 4);
    CHECK(design.partsInStage(1).empty());
    CHECK(design.part(2).stage == 2);
    CHECK(design.part(0).stage == 3);

    CHECK_FALSE(design.removeStage(0));
    CHECK(design.removeStage(1));
    CHECK(design.stageCount() == 3);
    CHECK(design.part(2).stage == 1);

    // Staging an empty stage leaves a gap that burns nothing
    design.insertStage(0);
    const DesignStats stats = computeStats(design, 9.82);
    REQUIRE(stats.stages.size() == 4);
    CHECK(stats.stages[0].engines.empty());
    CHECK(stats.stages[1].engines.size() == 1);
    CHECK(stats.liftoffTwr == doctest::Approx(stats.stages[1].twrSeaLevel));
}

TEST_CASE("Fins low on the rocket pull the pressure center below the mass center")
{
    RocketDesign design = twoStage();
    const DesignStats bare = computeStats(design, 9.82);

    RocketDesign fin;
    fin.setRoot(def("fin"));
    design.attachDesignRadial(5, 0.0, 0.0, fin, 4);
    const DesignStats finned = computeStats(design, 9.82);

    CHECK(finned.centerOfPressure.y < bare.centerOfPressure.y);
    CHECK(finned.centerOfPressure.y < finned.centerOfMass.y);
    CHECK(finned.stable);
    CHECK(finned.centerOfPressure.x == doctest::Approx(0.0));
    CHECK(finned.centerOfPressure.z == doctest::Approx(0.0));
}
