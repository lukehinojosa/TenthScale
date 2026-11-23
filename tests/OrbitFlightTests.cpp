#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <numbers>

#include <flight/DesignFile.h>
#include <flight/DesignStats.h>
#include <flight/OrbitInfo.h>
#include <flight/PartCatalog.h>
#include <flight/Rocket.h>
#include <solar/CircularSolarSystem.h>

using namespace tenth;

namespace
{
    constexpr double STEP = 1.0 / 50.0;
    constexpr double TARGET_APOAPSIS = 40.0e3; // metres above the surface
    constexpr double TARGET_PERIAPSIS = 15.0e3;
    constexpr double TIME_LIMIT = 900.0;
    constexpr double MOON_TRIP = 2700.0; // m/s from low orbit to the Moon's surface and home again, roughly

    const PartCatalog& catalog()
    {
        static const PartCatalog standard = PartCatalog::standard();
        return standard;
    }

    enum class Phase
    {
        Vertical,
        Kick,
        Climb,
        Coast,
        Circularize,
        Done,
    };

    // Flies the default rocket to orbit the way a player would: straight up off the pad, a tap of D to lean it east
    // once it's moving, then SAS on prograde until the apoapsis is high enough. It coasts up and, near the top, points
    // level and burns until the periapsis clears the air, staging whenever the burning stage runs dry.
    struct AscentPilot
    {
        CircularSolarSystem system;
        BodyId earth = system.findBody("Earth");
        Rocket rocket;
        double time = 0.0;
        Phase phase = Phase::Vertical;

        AscentPilot()
        {
            const Vehicle vehicle = makeVehicle(defaultRocket(catalog()));
            rocket.setVehicle(vehicle);
            rocket.place(earth, { body().radius + vehicle.groundClearance, 0.0, 0.0 }, {});
            rocket.fireNextStage();
            rocket.setSas(SasMode::Stability);
        }

        const BodyInfo& body() const
        {
            return system.body(earth);
        }

        OrbitInfo orbit() const
        {
            return orbitFrom(rocket.position(), rocket.velocity(), body().mu);
        }

        double leftToBurn() const
        {
            return computeStats(rocket.vehicle(), system.surfaceGravity(earth)).totalDeltaVVacuum;
        }

        // Seconds the burning stage would take to reach circular speed at the apoapsis, so the burn can straddle it
        double circularizeTime() const
        {
            const OrbitInfo now = orbit();
            const double reach = now.apoapsis;
            const double axis = (now.apoapsis + now.periapsis) * 0.5;
            const double needed = std::sqrt(body().mu / reach) - std::sqrt(body().mu * (2.0 / reach - 1.0 / axis));
            for (const StageStats& stage : computeStats(rocket.vehicle(), system.surfaceGravity(earth)).stages)
            {
                if (stage.thrustVacuum > 0.0)
                {
                    return needed * rocket.vehicle().mass / stage.thrustVacuum;
                }
            }
            return 1.0e9;
        }

        // Steers the nose level, along the way the rocket is moving over the ground
        Vec3d steerLevel() const
        {
            const Vec3d up = rocket.position().normalized();
            const Vec3d across = (rocket.velocity() - up * rocket.velocity().dot(up)).normalized();
            const Vec3d local = rocket.toLocal(across);
            const double side = std::sqrt(local.x * local.x + local.z * local.z);
            const double angle = std::atan2(side, local.y);
            const Vec3d error = side > 1e-9 ? Vec3d{ local.z, 0.0, -local.x } * (angle / side) : Vec3d{};
            const Vec3d turning = rocket.toLocal(rocket.spin());
            return { std::clamp(error.x * 3.0 - turning.x * 6.0, -1.0, 1.0), std::clamp(-turning.y * 6.0, -1.0, 1.0),
                     std::clamp(error.z * 3.0 - turning.z * 6.0, -1.0, 1.0) };
        }

        void decide(double kickSpeed, double kickPitch)
        {
            const Vec3d up = rocket.position().normalized();
            const double pitch = std::asin(std::clamp(rocket.toWorld({ 0.0, 1.0, 0.0 }).dot(up), -1.0, 1.0)) * 180.0 / std::numbers::pi;
            const OrbitInfo now = orbit();
            Vec3d steering;
            if (phase == Phase::Vertical && rocket.velocity().length() > kickSpeed)
            {
                phase = Phase::Kick;
            }
            if (phase == Phase::Kick)
            {
                steering = { 0.0, 0.0, -1.0 };
                if (pitch < kickPitch)
                {
                    phase = Phase::Climb;
                    rocket.setSas(SasMode::Prograde);
                }
            }
            if (phase == Phase::Climb && now.apoapsis - body().radius > TARGET_APOAPSIS)
            {
                phase = Phase::Coast;
                rocket.setThrottle(0.0);
            }
            if (phase == Phase::Coast && now.timeToApoapsis < circularizeTime() * 0.5)
            {
                phase = Phase::Circularize;
                rocket.setSas(SasMode::Off);
                rocket.setThrottle(1.0);
            }
            if (phase == Phase::Circularize)
            {
                steering = steerLevel();
                if (now.periapsis - body().radius > TARGET_PERIAPSIS)
                {
                    phase = Phase::Done;
                    rocket.setThrottle(0.0);
                    steering = {};
                }
            }
            rocket.setSteering(steering);
        }

        // Out of fuel in every lit engine, not merely throttled down
        bool burnedOut() const
        {
            const Vehicle& vehicle = rocket.vehicle();
            for (PartIndex i = 0; i < vehicle.design.partCount(); i++)
            {
                if (isBurning(vehicle, i))
                {
                    return false;
                }
            }
            return true;
        }

        // Returns whether it got to orbit before running out of time or hitting the ground
        bool fly(double kickSpeed, double kickPitch)
        {
            while (phase != Phase::Done && time < TIME_LIMIT)
            {
                decide(kickSpeed, kickPitch);
                if (rocket.throttle() > 0.0 && burnedOut() && rocket.vehicle().nextStage < rocket.vehicle().design.stageCount())
                {
                    rocket.fireNextStage();
                    rocket.takeDropped();
                }
                rocket.step(STEP, system, time);
                time += STEP;
                if (time > 10.0 && rocket.altitude(system) <= 0.0)
                {
                    return false;
                }
            }
            return phase == Phase::Done;
        }

        // The lowest it gets over one more lap with the engines off
        double lowestOverLap()
        {
            double lowest = rocket.altitude(system);
            const double end = time + orbit().period;
            while (time < end)
            {
                rocket.step(STEP, system, time);
                time += STEP;
                lowest = std::min(lowest, rocket.altitude(system));
            }
            return lowest;
        }
    };
}

TEST_CASE("The default rocket leaves the pad with half again its weight in thrust")
{
    const RocketDesign design = defaultRocket(catalog());
    CHECK(computeStats(design, 9.81).liftoffTwr > 1.4);
}

TEST_CASE("Flown by hand with a gravity turn, the default rocket reaches an orbit clear of the air with enough left for the Moon")
{
    // Leaning a few degrees east at around 100 m/s, the usual advice, lands in the middle of what works
    const double kicks[][2] = { { 90.0, 83.0 }, { 100.0, 84.0 }, { 110.0, 82.0 } };
    for (const auto& kick : kicks)
    {
        CAPTURE(kick[0]);
        CAPTURE(kick[1]);
        AscentPilot pilot;
        REQUIRE(pilot.fly(kick[0], kick[1]));
        const OrbitInfo orbit = pilot.orbit();
        CHECK(orbit.periapsis - pilot.body().radius > pilot.body().atmosphereHeight);
        CHECK(orbit.apoapsis - pilot.body().radius < 150.0e3);
        CHECK(pilot.leftToBurn() > MOON_TRIP);
        CHECK(pilot.lowestOverLap() > pilot.body().atmosphereHeight);
    }
}
