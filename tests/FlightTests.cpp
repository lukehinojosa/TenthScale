#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <numbers>

#include <flight/Rocket.h>
#include <solar/CircularSolarSystem.h>

using namespace tenth;

TEST_CASE("A rocket in a circular low Earth orbit stays at its altitude for a full lap")
{
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    const BodyInfo& info = system.body(earth);
    const double radius = info.radius + 20.0e3;
    const double speed = std::sqrt(info.mu / radius);
    const double period = 2.0 * std::numbers::pi * radius / speed;

    Rocket rocket;
    rocket.place(earth, { radius, 0.0, 0.0 }, { 0.0, 0.0, -speed });
    const double dt = 1.0 / 50.0;
    double time = 0.0;
    double lowest = radius;
    double highest = radius;
    while (time < period)
    {
        rocket.step(dt, system, time);
        time += dt;
        const double distance = rocket.position().length();
        lowest = std::min(lowest, distance);
        highest = std::max(highest, distance);
    }

    CHECK(rocket.reference() == earth);
    CHECK(highest - lowest < 100.0);
    CHECK((rocket.position() - Vec3d{ radius, 0.0, 0.0 }).length() < 2.0e3);
}

TEST_CASE("A rocket left at rest above the Earth falls straight down")
{
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    const double radius = system.body(earth).radius + 1.0e3;

    Rocket rocket;
    rocket.place(earth, { 0.0, radius, 0.0 }, {});
    for (int i = 0; i < 50; i++)
    {
        rocket.step(1.0 / 50.0, system, i / 50.0);
    }

    CHECK(rocket.position().y < radius);
    CHECK(rocket.velocity().y == doctest::Approx(-9.8).epsilon(0.02));
    CHECK(rocket.position().x == doctest::Approx(0.0));
}

TEST_CASE("Entering the Moon's sphere of influence switches reference without the true motion jumping")
{
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    const BodyId moon = system.findBody("Moon");
    const double time = 7.0e5;

    // Placed just outside the Moon's sphere of influence, coasting toward the Moon
    const Vec3d moonFromEarth = system.positionAt(moon, time) - system.positionAt(earth, time);
    const double moonSoi = system.body(moon).sphereOfInfluence;
    const Vec3d start = moonFromEarth - moonFromEarth.normalized() * (moonSoi + 10.0);
    const Vec3d velocity = system.velocityAt(moon, time) - system.velocityAt(earth, time) + moonFromEarth.normalized() * 500.0;

    Rocket rocket;
    rocket.place(earth, start, velocity);
    const Vec3d absoluteBefore = rocket.absolutePosition(system, time);
    const Vec3d absoluteVelBefore = rocket.absoluteVelocity(system, time);

    const double dt = 1.0 / 50.0;
    rocket.step(dt, system, time);
    CHECK(rocket.reference() == earth);
    for (int i = 1; i < 50; i++)
    {
        rocket.step(dt, system, time + i * dt);
    }

    CHECK(rocket.reference() == moon);
    CHECK(rocket.position().length() < moonSoi);
    const Vec3d absoluteAfter = rocket.absolutePosition(system, time + 1.0);
    CHECK((absoluteAfter - absoluteBefore - absoluteVelBefore).length() < 5.0);
    CHECK((rocket.absoluteVelocity(system, time + 1.0) - absoluteVelBefore).length() < 1.0);
}
