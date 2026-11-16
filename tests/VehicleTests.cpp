#include <doctest/doctest.h>

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

TEST_CASE("A rocket on the pad stands on the ground instead of falling in")
{
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    const Vehicle vehicle = makeVehicle(defaultRocket(catalog()));
    const double standing = system.body(earth).radius + vehicle.groundClearance;

    Rocket rocket;
    rocket.setVehicle(vehicle);
    rocket.place(earth, { standing, 0.0, 0.0 }, {});
    for (int i = 0; i < 100; i++)
    {
        rocket.step(1.0 / 50.0, system, i / 50.0);
    }
    CHECK(rocket.onGround());
    CHECK(rocket.position().length() == doctest::Approx(standing));
    CHECK(rocket.velocity().length() == doctest::Approx(0.0));
}
