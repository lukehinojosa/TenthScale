#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

#include <solar/CircularSolarSystem.h>

using namespace tenth;

TEST_CASE("The placeholder Solar System has the Sun, Earth, and the Moon in a parent chain")
{
    CircularSolarSystem system;
    const BodyId sun = system.findBody("Sun");
    const BodyId earth = system.findBody("Earth");
    const BodyId moon = system.findBody("Moon");
    REQUIRE(sun != NO_BODY);
    REQUIRE(earth != NO_BODY);
    REQUIRE(moon != NO_BODY);
    CHECK(system.findBody("Pluto") == NO_BODY);

    CHECK(system.body(sun).parent == NO_BODY);
    CHECK(system.body(earth).parent == sun);
    CHECK(system.body(moon).parent == earth);
}

TEST_CASE("Bodies are a tenth of their real size but keep their real surface gravity")
{
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    const BodyId moon = system.findBody("Moon");
    CHECK(system.body(earth).radius == doctest::Approx(637.1e3));
    CHECK(system.surfaceGravity(earth) == doctest::Approx(9.82).epsilon(0.01));
    CHECK(system.surfaceGravity(moon) == doctest::Approx(1.62).epsilon(0.01));

    // The proposal's headline number: low orbit takes about 2.4 km/s instead of 7.8
    const BodyInfo& info = system.body(earth);
    const double lowOrbitSpeed = std::sqrt(info.mu / (info.radius + 10.0e3));
    CHECK(lowOrbitSpeed == doctest::Approx(2.48e3).epsilon(0.02));
}

TEST_CASE("Orbits keep their distance and come back around after one period")
{
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    const BodyId moon = system.findBody("Moon");
    const double moonPeriod = 2.0 * std::numbers::pi * std::sqrt(std::pow(3.844e7, 3) / system.body(earth).mu);

    for (double time : { 0.0, 1.0e5, 2.5e6, moonPeriod })
    {
        CHECK(system.positionAt(earth, time).length() == doctest::Approx(1.495978707e10));
        const Vec3d moonFromEarth = system.positionAt(moon, time) - system.positionAt(earth, time);
        CHECK(moonFromEarth.length() == doctest::Approx(3.844e7));
        CHECK(moonFromEarth.y == doctest::Approx(0.0));
    }

    const Vec3d start = system.positionAt(moon, 0.0) - system.positionAt(earth, 0.0);
    const Vec3d lap = system.positionAt(moon, moonPeriod) - system.positionAt(earth, moonPeriod);
    CHECK((lap - start).length() < 1.0);
}

TEST_CASE("Velocities match how fast the positions change")
{
    CircularSolarSystem system;
    const double h = 1.0;
    for (BodyId id = 0; id < system.bodyCount(); id++)
    {
        for (double time : { 0.0, 3.0e5, 4.0e6 })
        {
            const Vec3d estimate = (system.positionAt(id, time + h) - system.positionAt(id, time - h)) / (2.0 * h);
            CHECK((estimate - system.velocityAt(id, time)).length() < 1.0e-3);
        }
    }
}

TEST_CASE("The dominant body is the smallest sphere of influence around the position")
{
    CircularSolarSystem system;
    const BodyId sun = system.findBody("Sun");
    const BodyId earth = system.findBody("Earth");
    const BodyId moon = system.findBody("Moon");
    const double time = 1.2e6;
    const Vec3d earthPos = system.positionAt(earth, time);
    const Vec3d moonPos = system.positionAt(moon, time);
    const double earthSoi = system.body(earth).sphereOfInfluence;

    CHECK(earthSoi == doctest::Approx(9.25e7).epsilon(0.01));
    CHECK(system.body(moon).sphereOfInfluence == doctest::Approx(6.6e6).epsilon(0.02));

    CHECK(system.dominantBody(earthPos + Vec3d{ 0.0, system.body(earth).radius, 0.0 }, time) == earth);
    CHECK(system.dominantBody(moonPos + Vec3d{ 0.0, 1.0e5, 0.0 }, time) == moon);
    CHECK(system.dominantBody(earthPos + Vec3d{ 0.0, earthSoi * 0.99, 0.0 }, time) == earth);
    CHECK(system.dominantBody(earthPos + Vec3d{ 0.0, earthSoi * 1.01, 0.0 }, time) == sun);
    CHECK(system.dominantBody(Vec3d{ 5.0e9, 0.0, 0.0 }, time) == sun);
}
