#include <doctest/doctest.h>

#include <cmath>
#include <limits>
#include <numbers>

#include <flight/OrbitInfo.h>
#include <flight/OrbitPath.h>

using namespace tenth;

namespace
{
    constexpr double MU = 3.986e13; // Earth's, at a tenth of the distances
    constexpr double RADIUS = 637.1e3;
    constexpr double SPHERE = 9.25e7; // Earth's sphere of influence at a tenth of the distance
    constexpr double NO_EDGE = std::numeric_limits<double>::infinity();

    // Follows gravity alone in small steps until the distance from the centre leaves [low, high], and says when
    double timeToLeave(Vec3d position, Vec3d velocity, double low, double high, double dt)
    {
        auto pull = [](const Vec3d& p)
        {
            const double r = p.length();
            return p * (-MU / (r * r * r));
        };
        double time = 0.0;
        while (time < 1.0e7)
        {
            const Vec3d a1 = pull(position);
            const Vec3d a2 = pull(position + velocity * (dt * 0.5));
            const Vec3d a3 = pull(position + (velocity + a1 * (dt * 0.5)) * (dt * 0.5));
            const Vec3d a4 = pull(position + (velocity + a2 * (dt * 0.5)) * dt);
            const Vec3d nextPosition = position + (velocity + (a1 + a2 + a3) * (dt / 6.0)) * dt;
            velocity = velocity + (a1 + a2 * 2.0 + a3 * 2.0 + a4) * (dt / 6.0);
            position = nextPosition;
            time += dt;
            const double r = position.length();
            if (r < low || r > high)
            {
                return time;
            }
        }
        return time;
    }
}

TEST_CASE("A circular orbit's path is one whole lap at its own height, starting where the rocket is")
{
    const double r = RADIUS + 20.0e3;
    const double speed = std::sqrt(MU / r);
    const Vec3d position = { r, 0.0, 0.0 };
    const PredictedPath path = predictPath(position, { 0.0, 0.0, -speed }, MU, RADIUS, SPHERE, 120);
    REQUIRE(path.points.size() == 121);
    CHECK(path.end == PathEnd::Lap);
    CHECK((path.points.front() - position).length() < 1e-6 * r);
    CHECK((path.points.back() - position).length() < 1e-6 * r);
    for (const Vec3d& point : path.points)
    {
        CHECK(point.length() == doctest::Approx(r));
    }
    CHECK(path.endTime == doctest::Approx(2.0 * std::numbers::pi * r / speed));
    CHECK_FALSE(path.reachesApoapsis);
    CHECK_FALSE(path.reachesPeriapsis);
}

TEST_CASE("An elliptical orbit marks its apoapsis and periapsis where and when the orbit readout puts them")
{
    const double r = RADIUS + 30.0e3;
    const Vec3d position = { 0.0, 0.0, r };
    const Vec3d velocity = { std::sqrt(MU / r) * 1.08, 200.0, 300.0 };
    const PredictedPath path = predictPath(position, velocity, MU, RADIUS, SPHERE, 360);
    const OrbitInfo orbit = orbitFrom(position, velocity, MU);
    REQUIRE(path.reachesApoapsis);
    REQUIRE(path.reachesPeriapsis);
    CHECK(path.end == PathEnd::Lap);
    CHECK(path.conic.radiusAt(path.apoapsisAngle) == doctest::Approx(orbit.apoapsis));
    CHECK(path.conic.radiusAt(path.periapsisAngle) == doctest::Approx(orbit.periapsis));
    CHECK(path.conic.timeUntil(path.apoapsisAngle) == doctest::Approx(orbit.timeToApoapsis));
    CHECK(path.conic.timeUntil(path.periapsisAngle) == doctest::Approx(orbit.timeToPeriapsis));
    CHECK(path.endTime == doctest::Approx(orbit.period));
}

TEST_CASE("A hop that doesn't reach orbit ends where and when it hits the ground")
{
    const Vec3d position = { RADIUS + 15.0, 0.0, 0.0 };
    const Vec3d velocity = { 700.0, 0.0, -900.0 };
    const PredictedPath path = predictPath(position, velocity, MU, RADIUS, SPHERE, 200);
    REQUIRE_FALSE(path.points.empty());
    CHECK(path.end == PathEnd::Impact);
    CHECK(path.points.back().length() == doctest::Approx(RADIUS));
    CHECK(path.reachesApoapsis);
    CHECK_FALSE(path.reachesPeriapsis);
    CHECK(path.endTime == doctest::Approx(timeToLeave(position, velocity, RADIUS, NO_EDGE, 0.01)).epsilon(1e-3));
}

TEST_CASE("Past escape speed the path ends where and when it leaves the sphere of influence")
{
    const double r = RADIUS + 50.0e3;
    const Vec3d position = { r, 0.0, 0.0 };
    const Vec3d velocity = { 0.0, 0.0, -std::sqrt(2.0 * MU / r) * 1.05 };
    const PredictedPath path = predictPath(position, velocity, MU, RADIUS, SPHERE, 200);
    REQUIRE_FALSE(path.points.empty());
    CHECK(path.end == PathEnd::Escape);
    CHECK(path.points.back().length() == doctest::Approx(SPHERE));
    CHECK(path.endTime == doctest::Approx(timeToLeave(position, velocity, 0.0, SPHERE, 1.0)).epsilon(1e-3));
    CHECK_FALSE(path.reachesApoapsis);
}

TEST_CASE("An ellipse reaching past the sphere of influence escapes before its apoapsis")
{
    const double r = RADIUS + 50.0e3;
    const Vec3d position = { r, 0.0, 0.0 };
    const Vec3d velocity = { 0.0, 0.0, -std::sqrt(2.0 * MU / r) * 0.999 };
    const PredictedPath path = predictPath(position, velocity, MU, RADIUS, SPHERE, 200);
    REQUIRE(path.conic.closed());
    CHECK(path.end == PathEnd::Escape);
    CHECK(path.points.back().length() == doctest::Approx(SPHERE));
    CHECK_FALSE(path.reachesApoapsis);
}

TEST_CASE("Something going straight up or standing still has no curve to draw")
{
    CHECK(predictPath({ RADIUS + 10.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, MU, RADIUS, SPHERE, 100).points.empty());
    CHECK(predictPath({ RADIUS + 10.0, 0.0, 0.0 }, { 300.0, 0.0, 0.0 }, MU, RADIUS, SPHERE, 100).points.empty());
}
