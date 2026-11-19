#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <numbers>

#include <flight/DesignFile.h>
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

    const PartDefinition& def(const char* id)
    {
        const PartDefinition* pDefinition = catalog().find(id);
        REQUIRE(pDefinition != nullptr);
        return *pDefinition;
    }
}

TEST_CASE("A lone tank spins like a solid cylinder about its own centre")
{
    RocketDesign design;
    design.setRoot(def("tank-small"));
    const Vehicle vehicle = makeVehicle(design);

    const double mass = 1125.0;
    const double radius = 0.625;
    CHECK(vehicle.mass == doctest::Approx(mass));
    CHECK(vehicle.centerOfMass == Vec3d{});
    CHECK(vehicle.groundClearance == doctest::Approx(0.5));
    CHECK(vehicle.inertia.yy == doctest::Approx(mass * radius * radius / 2.0));
    CHECK(vehicle.inertia.xx == doctest::Approx(mass * (3.0 * radius * radius + 1.0) / 12.0));
    CHECK(vehicle.inertia.zz == doctest::Approx(vehicle.inertia.xx));
}

TEST_CASE("Stacked parts add their distance from the center of mass to the pitch inertia, not the roll")
{
    RocketDesign one;
    one.setRoot(def("tank-small"));
    RocketDesign two = one;
    two.attachStack(0, Node::Bottom, def("tank-small"));

    const Inertia single = makeVehicle(one).inertia;
    const Inertia pair = makeVehicle(two).inertia;

    // Each tank's centre sits half a metre from the middle
    CHECK(pair.xx == doctest::Approx(2.0 * single.xx + 2.0 * 1125.0 * 0.25));
    CHECK(pair.yy == doctest::Approx(2.0 * single.yy));
    CHECK(pair.xy == doctest::Approx(0.0));
}

TEST_CASE("A fin's inertia turns with the side it's on")
{
    RocketDesign tank;
    tank.setRoot(def("tank-small"));
    const PartIndex front = tank.attachRadial(0, 0.0, 0.0, def("fin"));
    const PartIndex side = tank.attachRadial(0, std::numbers::pi / 2.0, 0.0, def("fin"));
    const Inertia facingZ = partInertia(tank.part(front));
    const Inertia facingX = partInertia(tank.part(side));
    CHECK(facingX.xx == doctest::Approx(facingZ.zz));
    CHECK(facingX.zz == doctest::Approx(facingZ.xx));
    CHECK(facingZ.xz == doctest::Approx(0.0));

    const PartIndex diagonal = tank.attachRadial(0, std::numbers::pi / 4.0, 0.0, def("fin"));
    CHECK(partInertia(tank.part(diagonal)).xz != doctest::Approx(0.0));
}

TEST_CASE("The default rocket keeps its mass, balance, and staging when handed to flight")
{
    const RocketDesign design = defaultRocket(catalog());
    const Vehicle vehicle = makeVehicle(design);
    CHECK(vehicle.mass == doctest::Approx(design.totalMass()));
    CHECK(vehicle.centerOfMass.y == doctest::Approx(design.centerOfMass().y));
    CHECK(vehicle.design.stageCount() == design.stageCount());
    CHECK(vehicle.nextStage == 0);

    // Symmetric fins and legs keep it balanced on its axis: equal pitch both ways, no cross terms
    CHECK(vehicle.centerOfMass.x == doctest::Approx(0.0));
    CHECK(vehicle.inertia.xx == doctest::Approx(vehicle.inertia.zz));
    CHECK(vehicle.inertia.xz == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(vehicle.inertia.xx > vehicle.inertia.yy);
}

TEST_CASE("The outline reaches the rocket's lowest point and its widest fins")
{
    const RocketDesign design = defaultRocket(catalog());
    const Vehicle vehicle = makeVehicle(design);
    double lowest = 0.0;
    double widest = 0.0;
    for (const Vec3d& point : vehicle.outline)
    {
        lowest = std::min(lowest, point.y);
        widest = std::max(widest, std::hypot(point.x, point.z));
    }
    CHECK(lowest == doctest::Approx(-vehicle.groundClearance));
    CHECK(widest > def("tank-large").shape.topDiameter * 0.5 + 0.2); // the fins stick out past the tanks
}

namespace
{
    struct PadTest
    {
        CircularSolarSystem system;
        BodyId earth = system.findBody("Earth");
        Vehicle vehicle = makeVehicle(defaultRocket(catalog()));
        double standing = system.body(earth).radius + vehicle.groundClearance;
        Rocket rocket;

        PadTest()
        {
            rocket.setVehicle(vehicle);
            rocket.place(earth, { standing, 0.0, 0.0 }, {});
        }

        void run(double seconds)
        {
            const int steps = (int)(seconds * 50.0);
            for (int i = 0; i < steps; i++)
            {
                rocket.step(1.0 / 50.0, system, i / 50.0);
            }
        }

        // Radians between the rocket's long axis and straight up
        double lean() const
        {
            const double cosine = rocket.toWorld({ 0.0, 1.0, 0.0 }).dot(rocket.position().normalized());
            return std::acos(std::clamp(cosine, -1.0, 1.0));
        }
    };
}

TEST_CASE("A rocket is placed standing straight up from the planet")
{
    PadTest pad;
    CHECK(pad.rocket.toWorld({ 0.0, 1.0, 0.0 }).x == doctest::Approx(1.0));
    CHECK(pad.rocket.toWorld({ 1.0, 0.0, 0.0 }).dot(pad.rocket.toWorld({ 0.0, 0.0, 1.0 })) == doctest::Approx(0.0));
    CHECK(pad.lean() == doctest::Approx(0.0));
}

TEST_CASE("A rocket on the pad stands still on the ground instead of falling in or tipping")
{
    PadTest pad;
    pad.run(10.0);
    CHECK(pad.rocket.onGround());
    CHECK(std::abs(pad.rocket.position().length() - pad.standing) < 0.02);
    CHECK(pad.rocket.velocity().length() < 0.05);
    CHECK(pad.lean() < 0.002);
    CHECK(std::abs(pad.rocket.altitude(pad.system)) < 0.02);
}

TEST_CASE("A rocket leaned past its fins falls over onto its side")
{
    PadTest pad;
    pad.rocket.turn({ 0.0, 0.0, 1.0 }, 0.5);
    pad.run(10.0);
    CHECK(pad.lean() > 1.2);
    CHECK(pad.rocket.position().length() < pad.standing - 3.0);
    CHECK(std::abs(pad.rocket.altitude(pad.system)) < 0.05);
}

TEST_CASE("A rocket leaned a little rocks back onto its fins")
{
    PadTest pad;
    pad.rocket.turn({ 0.0, 0.0, 1.0 }, 0.03);
    pad.run(10.0);
    CHECK(pad.lean() < 0.01);
}

TEST_CASE("A spinning rocket in space keeps spinning, since nothing pushes on it")
{
    PadTest pad;
    pad.rocket.place(pad.earth, { pad.system.body(pad.earth).radius + 50.0e3, 0.0, 0.0 }, { 0.0, 0.0, -2000.0 });
    const Vec3d roll = pad.rocket.toWorld({ 0.0, 0.5, 0.0 });
    pad.rocket.setSpin(roll);
    pad.run(5.0);
    CHECK((pad.rocket.spin() - roll).length() < 1e-9);
    CHECK_FALSE(pad.rocket.onGround());
}
