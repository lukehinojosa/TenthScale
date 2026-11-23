#include <doctest/doctest.h>

#include <cmath>

#include <flight/DesignFile.h>
#include <flight/DesignStats.h>
#include <flight/PartCatalog.h>
#include <flight/Rocket.h>
#include <flight/TimeWarp.h>
#include <solar/CircularSolarSystem.h>

#include "TestRockets.h"

using namespace tenth;

namespace
{
    // A clock that never moves, so every warp step fits in the budget
    double frozen()
    {
        return 0.0;
    }
}

TEST_CASE("Time warp starts in real time and steps up and down through its rates, stopping at each end")
{
    TimeWarp warp;
    CHECK(warp.rate() == 1);
    warp.slower();
    CHECK(warp.rate() == 1);
    for (int i = 0; i < 20; i++)
    {
        warp.faster();
    }
    CHECK(warp.rate() == TimeWarp::RATES.back());
    CHECK(warp.rate() >= 1000);
    warp.slower();
    CHECK(warp.rate() == TimeWarp::RATES[TimeWarp::RATES.size() - 2]);
    warp.stop();
    CHECK(warp.rate() == 1);
}

TEST_CASE("Each tick runs as many physics steps as the warp rate, and none while paused")
{
    TimeWarp warp;
    int steps = 0;
    auto step = [&]
    {
        steps++;
    };
    CHECK(warp.run(step, frozen, 0.01) == 1);
    warp.setLevel(4);
    CHECK(warp.run(step, frozen, 0.01) == TimeWarp::RATES[4]);
    CHECK(steps == 1 + TimeWarp::RATES[4]);
    warp.togglePause();
    CHECK(warp.run(step, frozen, 0.01) == 0);
    CHECK(warp.achieved() == 0.0);
    warp.togglePause();
    CHECK(warp.run(step, frozen, 0.01) == TimeWarp::RATES[4]);
}

TEST_CASE("When the steps take longer than the budget, the tick stops early and the readout shows the slower rate")
{
    TimeWarp warp;
    warp.stop();
    for (int i = 0; i < 20; i++)
    {
        warp.faster();
    }
    double clock = 0.0;
    auto slowStep = [&]
    {
        clock += 0.001; // each step takes a millisecond
    };
    auto now = [&]
    {
        return clock;
    };
    for (int tick = 0; tick < 100; tick++)
    {
        CHECK(warp.run(slowStep, now, 0.0105) == 11);
    }
    CHECK(warp.achieved() == doctest::Approx(11.0).epsilon(0.01));
    CHECK(warp.rate() == TimeWarp::RATES.back());
}

TEST_CASE("Warping a coasting orbit lands in the same place as stepping it one tick at a time")
{
    const PartCatalog catalog = PartCatalog::standard();
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    const double radius = system.body(earth).radius + 50.0e3;
    Rocket normal;
    normal.setVehicle(makeVehicle(defaultRocket(catalog)));
    normal.place(earth, { radius, 0.0, 0.0 }, { 0.0, 0.0, -std::sqrt(system.body(earth).mu / radius) });
    Rocket warped = normal;

    const double dt = 1.0 / 50.0;
    double normalTime = 0.0;
    for (int i = 0; i < 100 * 50; i++)
    {
        normal.step(dt, system, normalTime);
        normalTime += dt;
    }
    TimeWarp warp;
    warp.setLevel(5);
    double warpedTime = 0.0;
    for (int tick = 0; tick < 50; tick++)
    {
        warp.run(
            [&]
            {
                warped.step(dt, system, warpedTime);
                warpedTime += dt;
            },
            frozen, 1.0);
    }
    CHECK(warpedTime == doctest::Approx(normalTime));
    CHECK((warped.position() - normal.position()).length() < 1e-6);
}

TEST_CASE("Seconds of fuel count every engine drawing on a section, and stretch as the throttle comes down")
{
    const PartCatalog catalog = PartCatalog::standard();
    Vehicle vehicle = makeVehicle(twoStageRocket(catalog));
    CHECK(std::isinf(secondsOfFuel(vehicle, 1.0)));
    fireNextStage(vehicle);
    PartIndex lifter = NO_PART;
    for (PartIndex i = 0; i < vehicle.design.partCount(); i++)
    {
        if (vehicle.lit[i])
        {
            lifter = i;
        }
    }
    REQUIRE(lifter != NO_PART);
    const EngineOutput output = engineOutput(*vehicle.design.part(lifter).pDefinition, 1.0);
    const double flow = output.thrust / (output.isp * STANDARD_GRAVITY);
    const double full = secondsOfFuel(vehicle, 1.0);
    CHECK(full == doctest::Approx(sectionFuel(vehicle, vehicle.sections[lifter]) / flow));
    vehicle.throttle = 0.5;
    CHECK(secondsOfFuel(vehicle, 1.0) == doctest::Approx(full * 2.0));
    vehicle.throttle = 0.0;
    CHECK(std::isinf(secondsOfFuel(vehicle, 1.0)));
}

TEST_CASE("A hold drops warp to real time and keeps it there until lifted, and ends a tick early")
{
    TimeWarp warp;
    warp.setLevel(5);
    warp.setHeld(true);
    CHECK(warp.rate() == 1);
    warp.faster();
    warp.setLevel(4);
    CHECK(warp.rate() == 1);
    warp.setHeld(false);
    CHECK(warp.rate() == 1);
    warp.faster();
    CHECK(warp.rate() == TimeWarp::RATES[1]);

    warp.setLevel(5);
    int steps = 0;
    const int ran = warp.run(
        [&]
        {
            steps++;
            warp.setHeld(steps >= 30);
        },
        frozen, 1.0);
    CHECK(ran == 30);
    CHECK(warp.rate() == 1);
}

TEST_CASE("Warping through a burn drops to real time ten seconds before the tank runs dry, and lets go once it has")
{
    const PartCatalog catalog = PartCatalog::standard();
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    const double radius = system.body(earth).radius + 100.0e3;
    Rocket rocket;
    rocket.setVehicle(makeVehicle(twoStageRocket(catalog)));
    rocket.place(earth, { radius, 0.0, 0.0 }, { 0.0, 0.0, -std::sqrt(system.body(earth).mu / radius) });
    rocket.fireNextStage();

    const double dt = 1.0 / 50.0;
    double time = 0.0;
    TimeWarp warp;
    warp.setLevel(5);
    auto step = [&]
    {
        rocket.step(dt, system, time);
        time += dt;
        warp.setHeld(secondsOfFuel(rocket.vehicle(), rocket.pressure()) < 10.0);
    };
    while (!warp.held() && time < 600.0)
    {
        warp.run(step, frozen, 1.0);
    }
    REQUIRE(warp.held());
    CHECK(warp.rate() == 1);
    const double left = secondsOfFuel(rocket.vehicle(), rocket.pressure());
    CHECK(left < 10.0);
    CHECK(left > 10.0 - dt * 2.0);

    // At real time it burns down the last ten seconds, then the hold lifts with the tank empty
    int ticks = 0;
    while (warp.held() && ticks < 50 * 20)
    {
        warp.run(step, frozen, 1.0);
        ticks++;
    }
    CHECK_FALSE(warp.held());
    CHECK(ticks == doctest::Approx(left * 50.0).epsilon(0.02));
    CHECK(std::isinf(secondsOfFuel(rocket.vehicle(), rocket.pressure())));
}
