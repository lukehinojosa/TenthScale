#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include <flight/PartCatalog.h>
#include <flight/Rocket.h>
#include <flight/Vehicle.h>
#include <solar/CircularSolarSystem.h>

#include "TestRockets.h"

using namespace tenth;

namespace
{
    const PartCatalog& catalog()
    {
        static const PartCatalog standard = PartCatalog::standard();
        return standard;
    }

    // The two-stage rocket in a circular orbit above the air, standing straight up from the planet
    struct OrbitTest
    {
        CircularSolarSystem system;
        BodyId earth = system.findBody("Earth");
        Rocket rocket;
        double time = 0.0;

        OrbitTest()
        {
            rocket.setVehicle(makeVehicle(twoStageRocket(catalog())));
            const double radius = system.body(earth).radius + 100.0e3;
            rocket.place(earth, { radius, 0.0, 0.0 }, { 0.0, 0.0, -std::sqrt(system.body(earth).mu / radius) });
        }

        void run(double seconds)
        {
            const int steps = (int)std::lround(seconds * 50.0);
            for (int i = 0; i < steps; i++)
            {
                rocket.step(1.0 / 50.0, system, time);
                time += 1.0 / 50.0;
            }
        }

        Vec3d nose() const
        {
            return rocket.toWorld({ 0.0, 1.0, 0.0 });
        }

        // Degrees between the nose and where the mode wants it
        double offTarget(SasMode mode) const
        {
            Vec3d target;
            REQUIRE(rocket.holdDirection(mode, target));
            return std::acos(std::clamp(nose().dot(target), -1.0, 1.0)) * 180.0 / 3.14159265358979;
        }

        // The worst turn rate seen while running, in radians per second
        double runWatchingSpin(double seconds)
        {
            double worst = 0.0;
            for (int i = 0; i < (int)std::lround(seconds * 50.0); i++)
            {
                run(1.0 / 50.0);
                worst = std::max(worst, rocket.spin().length());
            }
            return worst;
        }
    };

    double degreesBetween(const Vec3d& a, const Vec3d& b)
    {
        return std::acos(std::clamp(a.dot(b), -1.0, 1.0)) * 180.0 / 3.14159265358979;
    }
}

TEST_CASE("The orbit's directions: prograde along the motion, normal square to the orbit, radial out away from the planet")
{
    OrbitTest test;
    Vec3d direction;
    REQUIRE(test.rocket.holdDirection(SasMode::Prograde, direction));
    CHECK(degreesBetween(direction, test.rocket.velocity().normalized()) < 1e-6);
    REQUIRE(test.rocket.holdDirection(SasMode::RadialOut, direction));
    CHECK(degreesBetween(direction, test.rocket.position().normalized()) < 1e-6);
    REQUIRE(test.rocket.holdDirection(SasMode::Normal, direction));
    CHECK(std::abs(direction.dot(test.rocket.velocity())) < 1e-6);
    CHECK(std::abs(direction.dot(test.rocket.position())) < 1e-3);
    Vec3d opposite;
    REQUIRE(test.rocket.holdDirection(SasMode::AntiNormal, opposite));
    CHECK(degreesBetween(direction, -opposite) < 1e-6);
}

TEST_CASE("Standing still, every mode still has a direction: prograde up, and normal north as for an eastward climb")
{
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    const double radius = system.body(earth).radius + 100.0e3;
    Rocket rocket;
    rocket.setVehicle(makeVehicle(twoStageRocket(catalog())));
    rocket.place(earth, { radius, 0.0, 0.0 }, {});
    Vec3d direction;
    REQUIRE(rocket.holdDirection(SasMode::Prograde, direction));
    CHECK(direction.x == doctest::Approx(1.0));
    REQUIRE(rocket.holdDirection(SasMode::Normal, direction));
    CHECK(direction.y == doctest::Approx(1.0));
    REQUIRE(rocket.holdDirection(SasMode::RadialOut, direction));
    CHECK(direction.z == doctest::Approx(1.0));

    // Ten degrees off vertical, radial out lies square to the path: pointing back, ten degrees above the horizon
    rocket.place(earth, { radius, 0.0, 0.0 }, { 200.0 * std::cos(0.1745), 0.0, -200.0 * std::sin(0.1745) });
    REQUIRE(rocket.holdDirection(SasMode::RadialOut, direction));
    CHECK(direction.x == doctest::Approx(std::sin(0.1745)));
    CHECK(direction.z == doctest::Approx(std::cos(0.1745)));
}

TEST_CASE("A lit engine adds to the wheels' pitch and yaw, but only the wheels roll")
{
    Vehicle vehicle = makeVehicle(twoStageRocket(catalog()));
    const Vec3d coasting = turnAuthority(vehicle, 0.0);
    CHECK(coasting.x == doctest::Approx(wheelTorque(vehicle)));
    fireNextStage(vehicle);
    const Vec3d burning = turnAuthority(vehicle, 0.0);
    CHECK(burning.x > coasting.x * 5.0);
    CHECK(burning.z == doctest::Approx(burning.x));
    CHECK(burning.y == doctest::Approx(wheelTorque(vehicle)));
}

TEST_CASE("Stability stops a tumble and then holds the nose still")
{
    OrbitTest test;
    test.rocket.setSpin(test.rocket.toWorld({ 0.15, 0.5, -0.1 }));
    test.rocket.setSas(SasMode::Stability);
    test.run(40.0);
    CHECK(test.rocket.spin().length() < 0.002);
    const Vec3d held = test.nose();
    test.run(20.0);
    CHECK(degreesBetween(test.nose(), held) < 0.5);
}

TEST_CASE("Each orbit mode swings the nose round onto its direction and settles there")
{
    for (SasMode mode : { SasMode::Prograde, SasMode::Retrograde, SasMode::Normal, SasMode::AntiNormal, SasMode::RadialIn })
    {
        CAPTURE((int)mode);
        OrbitTest test;
        test.rocket.setSas(mode);
        test.run(60.0);
        CHECK(test.offTarget(mode) < 1.0);
        CHECK(test.runWatchingSpin(20.0) < 0.01);
        CHECK(test.offTarget(mode) < 1.0);
    }
}

TEST_CASE("Under power the swivelling engine helps hold the nose without setting it wobbling")
{
    OrbitTest test;
    test.rocket.fireNextStage();
    test.rocket.setSpin(test.rocket.toWorld({ 0.2, 0.0, 0.1 }));
    test.rocket.setSas(SasMode::Stability);
    test.run(10.0);
    const Vec3d held = test.nose();
    CHECK(test.runWatchingSpin(10.0) < 0.005);
    CHECK(degreesBetween(test.nose(), held) < 0.5);
}

TEST_CASE("Under power, prograde swings the nose round quickly and holds it there")
{
    OrbitTest test;
    test.rocket.fireNextStage();
    test.rocket.setSas(SasMode::Prograde);
    test.run(15.0);
    CHECK(test.offTarget(SasMode::Prograde) < 1.0);
    CHECK(test.runWatchingSpin(5.0) < 0.01);
}

TEST_CASE("The pilot can steer away from a Stability hold, and it holds the new heading once let go")
{
    OrbitTest test;
    test.rocket.setSas(SasMode::Stability);
    const Vec3d start = test.nose();
    test.rocket.setSteering({ 1.0, 0.0, 0.0 });
    test.run(3.0);
    test.rocket.setSteering({});
    test.run(30.0);
    const Vec3d settled = test.nose();
    CHECK(degreesBetween(settled, start) > 5.0);
    test.run(20.0);
    CHECK(degreesBetween(test.nose(), settled) < 0.5);
}

TEST_CASE("Turning the SAS off leaves only the pilot steering")
{
    OrbitTest test;
    test.rocket.setSas(SasMode::Prograde);
    test.run(1.0);
    test.rocket.setSas(SasMode::Off);
    test.run(0.1);
    CHECK(test.rocket.vehicle().steering.lengthSquared() == 0.0);
}

TEST_CASE("A dropped stage comes away with its SAS off")
{
    OrbitTest test;
    test.rocket.setSas(SasMode::Stability);
    test.rocket.fireNextStage();
    test.rocket.fireNextStage();
    const std::vector<Rocket> dropped = test.rocket.takeDropped();
    REQUIRE_FALSE(dropped.empty());
    for (const Rocket& piece : dropped)
    {
        CHECK(piece.sas() == SasMode::Off);
    }
    CHECK(test.rocket.sas() == SasMode::Stability);
}

namespace
{
    // Taps pitch for a moment while a mode holds, then returns how far the nose swung past the target on the way
    // back and how long it took to settle within a degree, in degrees and seconds
    void tapAndRecover(OrbitTest& test, SasMode mode, double tap, double& overshoot, double& settled)
    {
        test.run(10.0);
        test.rocket.setSteering({ 1.0, 0.0, 0.0 });
        test.run(tap);
        test.rocket.setSteering({});
        overshoot = 0.0;
        settled = -1.0;
        for (int i = 1; i <= 50 * 15; i++)
        {
            test.run(1.0 / 50.0);
            Vec3d target;
            REQUIRE(test.rocket.holdDirection(mode, target));
            const Vec3d local = test.rocket.toLocal(target);
            overshoot = std::max(overshoot, std::atan2(local.z, local.y) * 180.0 / 3.14159265358979);
            if (test.offTarget(mode) > 1.0)
            {
                settled = -1.0;
            }
            else if (settled < 0.0)
            {
                settled = i / 50.0;
            }
        }
    }
}

TEST_CASE("Knocked off its direction by a tap of pitch, the SAS brings the nose back without swinging past")
{
    for (bool burning : { false, true })
    {
        CAPTURE(burning);
        OrbitTest test;
        if (burning)
        {
            test.rocket.fireNextStage();
        }
        test.rocket.setSas(SasMode::RadialOut);
        double overshoot = 0.0;
        double settled = 0.0;
        tapAndRecover(test, SasMode::RadialOut, burning ? 0.5 : 1.0, overshoot, settled);
        CHECK(overshoot < 0.5);
        CHECK(settled > 0.0);
        CHECK(settled < 5.0);
    }
}

TEST_CASE("Climbing straight up, a tap of pitch eases normal and radial round instead of throwing them")
{
    for (SasMode mode : { SasMode::Normal, SasMode::RadialOut, SasMode::RadialIn })
    {
        CAPTURE((int)mode);
        OrbitTest test;
        test.rocket.place(test.earth, test.rocket.position(), test.rocket.position().normalized() * 200.0);
        test.rocket.fireNextStage();
        test.rocket.setSas(mode);
        test.run(5.0);
        Vec3d last;
        REQUIRE(test.rocket.holdDirection(mode, last));
        double worst = 0.0;
        for (int i = 0; i < 50 * 10; i++)
        {
            test.rocket.setSteering(i < 25 ? Vec3d{ 1.0, 0.0, 0.0 } : Vec3d{});
            test.run(1.0 / 50.0);
            Vec3d now;
            REQUIRE(test.rocket.holdDirection(mode, now));
            worst = std::max(worst, degreesBetween(now, last));
            last = now;
        }
        CHECK(test.rocket.sas() == mode);
        CHECK(worst < 1.0);
    }
}
