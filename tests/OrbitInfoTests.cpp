#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

#include <flight/OrbitInfo.h>

using namespace tenth;

namespace
{
    constexpr double MU = 3.986e13; // Earth's, at a tenth of the distances
    constexpr double RADIUS = 637.1e3;
}

TEST_CASE("A circular orbit has its apoapsis and periapsis at its own height")
{
    const double r = RADIUS + 20.0e3;
    const double speed = std::sqrt(MU / r);
    const OrbitInfo orbit = orbitFrom({ r, 0.0, 0.0 }, { 0.0, 0.0, -speed }, MU);
    CHECK(orbit.eccentricity < 1e-9);
    CHECK(orbit.apoapsis == doctest::Approx(r).epsilon(1e-9));
    CHECK(orbit.periapsis == doctest::Approx(r).epsilon(1e-9));
    CHECK(orbit.period == doctest::Approx(2.0 * std::numbers::pi * r / speed));
    CHECK(orbit.inclination == doctest::Approx(0.0));
    CHECK_FALSE(orbit.escaping);
}

TEST_CASE("Moving faster than circular at a point makes that point the periapsis, half a lap from the apoapsis")
{
    const double r = RADIUS + 20.0e3;
    const double speed = std::sqrt(MU / r) * 1.1;
    const OrbitInfo orbit = orbitFrom({ r, 0.0, 0.0 }, { 0.0, speed, 0.0 }, MU);
    CHECK(orbit.periapsis == doctest::Approx(r));
    CHECK(orbit.apoapsis > r * 1.4);
    CHECK(orbit.timeToApoapsis == doctest::Approx(orbit.period * 0.5));
    CHECK(orbit.inclination == doctest::Approx(std::numbers::pi * 0.5));
}

TEST_CASE("The times to apoapsis and periapsis count down as the orbit is followed")
{
    // Follow an elliptical orbit for a while from its periapsis by stepping it under gravity
    const double r = RADIUS + 20.0e3;
    Vec3d position = { r, 0.0, 0.0 };
    Vec3d velocity = { 0.0, 0.0, -std::sqrt(MU / r) * 1.15 };
    const OrbitInfo start = orbitFrom(position, velocity, MU);
    const double dt = 0.01;
    const double elapsed = 100.0;
    for (int i = 0; i < (int)(elapsed / dt); i++)
    {
        velocity -= position * (MU / std::pow(position.length(), 3.0) * dt);
        position += velocity * dt;
    }
    const OrbitInfo later = orbitFrom(position, velocity, MU);
    CHECK(later.apoapsis == doctest::Approx(start.apoapsis).epsilon(1e-3));
    CHECK(later.periapsis == doctest::Approx(start.periapsis).epsilon(1e-3));
    CHECK(later.timeToApoapsis == doctest::Approx(start.timeToApoapsis - elapsed).epsilon(1e-3));
    CHECK(later.timeToPeriapsis == doctest::Approx(start.period - elapsed).epsilon(1e-3));
}

TEST_CASE("Standing still on the ground falls straight in, with the apoapsis right here")
{
    const OrbitInfo orbit = orbitFrom({ 0.0, RADIUS, 0.0 }, {}, MU);
    CHECK(orbit.apoapsis == doctest::Approx(RADIUS));
    CHECK(orbit.periapsis == doctest::Approx(0.0));
    CHECK(orbit.timeToApoapsis == doctest::Approx(0.0));
}

TEST_CASE("Faster than escape speed never comes back, and a periapsis behind it stays behind")
{
    const double r = RADIUS + 20.0e3;
    const double escape = std::sqrt(2.0 * MU / r);
    const OrbitInfo leaving = orbitFrom({ r, 0.0, 0.0 }, { escape * 0.5, 0.0, -escape }, MU);
    CHECK(leaving.escaping);
    CHECK(leaving.periapsis < r);
    CHECK(leaving.timeToPeriapsis == 0.0);

    const OrbitInfo arriving = orbitFrom({ r, 0.0, 0.0 }, { -escape * 0.5, 0.0, -escape }, MU);
    CHECK(arriving.escaping);
    CHECK(arriving.timeToPeriapsis > 0.0);
}
