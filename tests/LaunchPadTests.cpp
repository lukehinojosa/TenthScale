#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

#include <flight/DesignFile.h>
#include <flight/LaunchPad.h>
#include <flight/PartCatalog.h>
#include <flight/Rocket.h>
#include <solar/CircularSolarSystem.h>

using namespace tenth;

namespace
{
    constexpr double DEGREES = std::numbers::pi / 180.0;

    LaunchPad testPad(BodyId body, const SurfaceFrame& frame)
    {
        LaunchPad pad;
        pad.body = body;
        pad.frame = frame;
        pad.halfWidth = 50.0;
        pad.halfLength = 60.0;
        pad.height = 15.0;
        pad.skirt = 45.0;
        return pad;
    }

    // Drops the default rocket from a little above a spot and lets it settle, returning its altitude
    double settleAt(double x, double z, bool& onGround)
    {
        const PartCatalog catalog = PartCatalog::standard();
        CircularSolarSystem system;
        const BodyId earth = system.findBody("Earth");
        const double radius = system.body(earth).radius;
        const Vec3d site = SurfaceFrame::direction(28.6 * DEGREES, -80.6 * DEGREES);
        const SurfaceFrame frame = SurfaceFrame::below(site, radius);
        Rocket rocket;
        rocket.setVehicle(makeVehicle(defaultRocket(catalog)));
        rocket.setPad(testPad(earth, frame));
        const Vec3d spot = frame.origin + frame.toBody({ x, 0.0, z });
        rocket.place(earth, spot.normalized() * (radius + 25.0 + rocket.vehicle().groundClearance), {});
        double time = 0.0;
        for (int i = 0; i < 50 * 10; i++)
        {
            rocket.step(1.0 / 50.0, system, time);
            time += 1.0 / 50.0;
        }
        onGround = rocket.onGround();
        return rocket.altitude(system);
    }
}

TEST_CASE("Latitude and longitude put the prime meridian on +X, north up +Y, and east toward -Z")
{
    const Vec3d meridian = SurfaceFrame::direction(0.0, 0.0);
    CHECK(meridian.x == doctest::Approx(1.0));
    const Vec3d east = SurfaceFrame::direction(0.0, 90.0 * DEGREES);
    CHECK(east.z == doctest::Approx(-1.0));
    const Vec3d north = SurfaceFrame::direction(90.0 * DEGREES, 0.0);
    CHECK(north.y == doctest::Approx(1.0));

    // The frame standing on a spot faces east the way its longitude grows
    const SurfaceFrame frame = SurfaceFrame::below(SurfaceFrame::direction(28.6 * DEGREES, -80.6 * DEGREES), 1.0);
    const Vec3d further = SurfaceFrame::direction(28.6 * DEGREES, -80.5 * DEGREES);
    CHECK(frame.toLocal(further).x > 0.0);
    CHECK(frame.toBody(frame.rotate({ 0.3, -0.2, 0.9 })).z == doctest::Approx(0.9));
}

TEST_CASE("The pad stands at full height over its top, slopes down its sides, and is gone past their foot")
{
    const LaunchPad pad = testPad(0, SurfaceFrame::below({ 1.0, 0.0, 0.0 }, 1.0));
    CHECK(pad.heightAt(0.0, 0.0) == doctest::Approx(15.0));
    CHECK(pad.heightAt(49.0, -59.0) == doctest::Approx(15.0));
    CHECK(pad.heightAt(50.0 + 22.5, 0.0) == doctest::Approx(7.5));
    CHECK(pad.heightAt(0.0, -60.0 - 45.0) == doctest::Approx(0.0));
    CHECK(pad.heightAt(200.0, 0.0) == 0.0);
    CHECK(pad.normalAt(0.0, 0.0).y == doctest::Approx(1.0));
    CHECK(pad.normalAt(60.0, 0.0).x > 0.0);
    CHECK(pad.normalAt(0.0, -70.0).z < 0.0);
}

TEST_CASE("A rocket dropped onto the pad comes to rest on its top, and one dropped beside it rests on the ground")
{
    bool onGround = false;
    CHECK(settleAt(0.0, 0.0, onGround) == doctest::Approx(15.0).epsilon(0.01));
    CHECK(onGround);
    CHECK(settleAt(150.0, 0.0, onGround) == doctest::Approx(0.0).scale(1.0).epsilon(0.05));
    CHECK(onGround);
}
