#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include <flight/Atmosphere.h>
#include <flight/DesignFile.h>
#include <flight/DesignStats.h>
#include <flight/PartCatalog.h>
#include <flight/Rocket.h>
#include <flight/Vehicle.h>
#include <solar/CircularSolarSystem.h>

using namespace tenth;

namespace
{
    const PartCatalog& catalog()
    {
        static const PartCatalog standard = PartCatalog::standard();
        return standard;
    }

    std::vector<PartIndex> enginesOf(const Vehicle& vehicle)
    {
        std::vector<PartIndex> engines;
        for (PartIndex i = 0; i < vehicle.design.partCount(); i++)
        {
            if (vehicle.design.part(i).pDefinition->isEngine())
            {
                engines.push_back(i);
            }
        }
        return engines;
    }

    struct LaunchTest
    {
        CircularSolarSystem system;
        BodyId earth = system.findBody("Earth");
        Rocket rocket;
        double time = 0.0;

        LaunchTest()
        {
            const Vehicle vehicle = makeVehicle(defaultRocket(catalog()));
            rocket.setVehicle(vehicle);
            rocket.place(earth, { system.body(earth).radius + vehicle.groundClearance, 0.0, 0.0 }, {});
        }

        void run(double seconds)
        {
            const int steps = (int)(seconds * 50.0);
            for (int i = 0; i < steps; i++)
            {
                rocket.step(1.0 / 50.0, system, time);
                time += 1.0 / 50.0;
            }
        }

        double climbRate() const { return rocket.velocity().dot(rocket.position().normalized()); }
    };
}

TEST_CASE("Air pressure is full on the ground, thins with height, and stops at the top of the atmosphere")
{
    CircularSolarSystem system;
    const BodyInfo& earth = system.body(system.findBody("Earth"));
    CHECK(pressureRatio(earth, 0.0) == doctest::Approx(1.0));
    CHECK(pressureRatio(earth, 1000.0) < pressureRatio(earth, 500.0));
    CHECK(pressureRatio(earth, earth.atmosphereHeight + 1.0) == 0.0);
    CHECK(pressureRatio(system.body(system.findBody("Moon")), 0.0) == 0.0);
}

TEST_CASE("An engine pushes harder and burns longer in vacuum than at sea level")
{
    const PartDefinition& engine = *catalog().find("engine-lifter");
    const EngineOutput vacuum = engineOutput(engine, 0.0);
    const EngineOutput seaLevel = engineOutput(engine, 1.0);
    CHECK(vacuum.thrust == doctest::Approx(engine.thrustVacuum));
    CHECK(vacuum.isp == doctest::Approx(engine.ispVacuum));
    CHECK(seaLevel.isp == doctest::Approx(engine.ispSeaLevel));
    CHECK(seaLevel.thrust == doctest::Approx(engine.thrustVacuum * engine.ispSeaLevel / engine.ispVacuum));

    // The same fuel flow either way: only how hard it pushes changes
    CHECK(seaLevel.thrust / seaLevel.isp == doctest::Approx(vacuum.thrust / vacuum.isp));
}

TEST_CASE("Staging lights only that stage's engines, and they draw only from their own section's tanks")
{
    Vehicle vehicle = makeVehicle(defaultRocket(catalog()));
    const std::vector<PartIndex> engines = enginesOf(vehicle);
    REQUIRE(engines.size() == 2);
    const PartIndex lifter = vehicle.design.part(engines[0]).stage == 0 ? engines[0] : engines[1];
    const PartIndex upper = lifter == engines[0] ? engines[1] : engines[0];

    fireNextStage(vehicle);
    CHECK(vehicle.nextStage == 1);
    CHECK(vehicle.lit[lifter]);
    CHECK_FALSE(vehicle.lit[upper]);

    const double upperFuel = sectionFuel(vehicle, vehicle.sections[upper]);
    const double lowerFuel = sectionFuel(vehicle, vehicle.sections[lifter]);
    const double massBefore = vehicle.mass;
    const double centerBefore = vehicle.centerOfMass.y;
    burnFuel(vehicle, 1.0, 10.0);
    updateMass(vehicle);

    const EngineOutput output = engineOutput(*vehicle.design.part(lifter).pDefinition, 1.0);
    const double expected = output.thrust / (output.isp * STANDARD_GRAVITY) * 10.0;
    CHECK(sectionFuel(vehicle, vehicle.sections[upper]) == doctest::Approx(upperFuel));
    CHECK(sectionFuel(vehicle, vehicle.sections[lifter]) == doctest::Approx(lowerFuel - expected));
    CHECK(vehicle.mass == doctest::Approx(massBefore - expected));

    // Fuel leaves the bottom of the rocket, so the balance point rises
    CHECK(vehicle.centerOfMass.y > centerBefore);

    // Both lower tanks empty at the same pace
    std::vector<double> shares;
    for (PartIndex i = 0; i < vehicle.design.partCount(); i++)
    {
        if (vehicle.sections[i] == vehicle.sections[lifter] && vehicle.design.part(i).pDefinition->fuelMass > 0.0)
        {
            shares.push_back(vehicle.fuel[i] / vehicle.design.part(i).pDefinition->fuelMass);
        }
    }
    REQUIRE(shares.size() == 2);
    CHECK(shares[0] == doctest::Approx(shares[1]));
}

TEST_CASE("At full throttle the default rocket lifts off and climbs straight up")
{
    LaunchTest launch;
    const double massBefore = launch.rocket.vehicle().mass;
    launch.rocket.fireNextStage();
    launch.run(10.0);
    CHECK_FALSE(launch.rocket.onGround());
    CHECK(launch.rocket.altitude(launch.system) > 20.0);
    CHECK(launch.climbRate() > 5.0);
    CHECK(launch.rocket.vehicle().mass < massBefore);
    CHECK(launch.rocket.toWorld({ 0.0, 1.0, 0.0 }).dot(launch.rocket.position().normalized()) == doctest::Approx(1.0));
    CHECK(launch.rocket.spin().length() < 1e-6);
}

TEST_CASE("With the throttle cut, a lit rocket stays on the pad and burns nothing")
{
    LaunchTest launch;
    launch.rocket.setThrottle(0.0);
    launch.rocket.fireNextStage();
    const double massBefore = launch.rocket.vehicle().mass;
    launch.run(3.0);
    CHECK(launch.rocket.onGround());
    CHECK(launch.rocket.thrust() == 0.0);
    CHECK(launch.rocket.vehicle().mass == doctest::Approx(massBefore));
}

TEST_CASE("Once its section runs dry an engine stops pushing")
{
    LaunchTest launch;
    launch.rocket.fireNextStage();
    launch.run(300.0);
    const Vehicle& vehicle = launch.rocket.vehicle();
    for (PartIndex engine : enginesOf(vehicle))
    {
        CHECK_FALSE(isBurning(vehicle, engine));
    }
    CHECK(launch.rocket.thrust() == 0.0);
}

TEST_CASE("Delta-v left shrinks as the burning stage uses its fuel")
{
    Vehicle vehicle = makeVehicle(defaultRocket(catalog()));
    const double gravity = 9.81;
    const DesignStats full = computeStats(vehicle.design, gravity);
    fireNextStage(vehicle);
    const DesignStats lit = computeStats(vehicle, gravity);
    CHECK(lit.totalDeltaVVacuum == doctest::Approx(full.totalDeltaVVacuum));

    burnFuel(vehicle, 1.0, 20.0);
    updateMass(vehicle);
    const DesignStats partway = computeStats(vehicle, gravity);
    CHECK(partway.stages[0].deltaVVacuum < full.stages[0].deltaVVacuum);
    CHECK(partway.stages[1].deltaVVacuum == doctest::Approx(full.stages[1].deltaVVacuum));
    CHECK(partway.totalDeltaVVacuum < full.totalDeltaVVacuum);
}
