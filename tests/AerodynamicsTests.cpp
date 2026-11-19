#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

#include <flight/Aerodynamics.h>
#include <flight/Atmosphere.h>
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

    const PartDefinition& def(const char* id)
    {
        const PartDefinition* pDefinition = catalog().find(id);
        REQUIRE(pDefinition != nullptr);
        return *pDefinition;
    }

    // How hard the air turns the rocket toward its direction of travel: positive when the turn lines the nose up with
    // the motion, which is what keeps a rocket flying straight
    double weathervaning(const RocketDesign& design)
    {
        CircularSolarSystem system;
        const BodyId earth = system.findBody("Earth");
        const double radius = system.body(earth).radius + 500.0;
        const Vehicle vehicle = makeVehicle(design);
        Rocket rocket;
        rocket.setVehicle(vehicle);

        // Flying up and a little sideways, so the air comes at it from slightly off the nose
        const double slip = 0.15;
        rocket.place(earth, { radius, 0.0, 0.0 }, { 200.0 * std::cos(slip), 200.0 * std::sin(slip), 0.0 });
        rocket.step(1.0 / 50.0, system, 0.0);
        const Vec3d axis = rocket.toWorld({ 0.0, 1.0, 0.0 });
        const Vec3d turning = rocket.spin().cross(axis);
        return turning.y;
    }
}

TEST_CASE("Air is thickest at sea level and gone above the atmosphere")
{
    CircularSolarSystem system;
    const BodyInfo& earth = system.body(system.findBody("Earth"));
    CHECK(airDensity(earth, 0.0) == doctest::Approx(SEA_LEVEL_DENSITY));
    CHECK(airDensity(earth, 2000.0) == doctest::Approx(SEA_LEVEL_DENSITY * pressureRatio(earth, 2000.0)));
    CHECK(airDensity(earth, earth.atmosphereHeight + 1.0) == 0.0);
}

TEST_CASE("A tank moving sideways feels its side outline; moving end-on, only its uncovered face")
{
    RocketDesign design;
    design.setRoot(def("tank-large"));
    const PartShape& shape = design.part(0).shape;
    const double speed = 100.0;
    const double pressure = 0.5 * SEA_LEVEL_DENSITY * speed * speed;

    const Vec3d sideways = partDrag(design, 0, { speed, 0.0, 0.0 }, SEA_LEVEL_DENSITY);
    CHECK(sideways.x == doctest::Approx(-pressure * shape.height * shape.topDiameter * 1.0));
    CHECK(sideways.y == doctest::Approx(0.0));

    const Vec3d endOn = partDrag(design, 0, { 0.0, speed, 0.0 }, SEA_LEVEL_DENSITY);
    const double face = std::numbers::pi * shape.topDiameter * shape.topDiameter / 4.0;
    CHECK(endOn.y == doctest::Approx(-pressure * face * 0.9));

    // With another tank on top, the top face is hidden from the flow
    design.attachStack(0, Node::Top, def("tank-large"));
    CHECK(partDrag(design, 0, { 0.0, speed, 0.0 }, SEA_LEVEL_DENSITY).y == doctest::Approx(0.0));
    CHECK(partDrag(design, 0, { 0.0, speed, 0.0 }, 0.0).length() == 0.0);
}

TEST_CASE("A nose cone parts the air far better than a flat tank top")
{
    RocketDesign cone;
    cone.setRoot(def("tank-large"));
    const PartIndex nose = cone.attachStack(0, Node::Top, def("nose-cone"));
    RocketDesign flat;
    flat.setRoot(def("tank-large"));
    const Vec3d up = { 0.0, 150.0, 0.0 };
    CHECK(std::abs(partDrag(cone, nose, up, 1.0).y) < 0.5 * std::abs(partDrag(flat, 0, up, 1.0).y));
}

TEST_CASE("A fin is pushed by flow across its face, hardly at all by flow along it")
{
    RocketDesign design;
    design.setRoot(def("tank-large"));
    const PartIndex fin = design.attachRadial(0, 0.0, 0.0, def("fin"));
    const Vec3d edgeOn = partDrag(design, fin, { 0.0, 100.0, 0.0 }, 1.0);
    const Vec3d faceOn = partDrag(design, fin, { 100.0, 0.0, 0.0 }, 1.0);
    CHECK(faceOn.length() > 10.0 * edgeOn.length());
    CHECK(faceOn.x < 0.0);
}

TEST_CASE("Fins low on the rocket turn its nose into the wind")
{
    const RocketDesign withFins = twoStageRocket(catalog());
    RocketDesign withoutFins = withFins;
    for (PartIndex i = withoutFins.partCount() - 1; i >= 0; i--)
    {
        if (withoutFins.part(i).pDefinition->id == "fin")
        {
            withoutFins.remove(i);
        }
    }
    CHECK(weathervaning(withFins) > 0.0);
    CHECK(weathervaning(withFins) > weathervaning(withoutFins));
}

TEST_CASE("A falling tank settles at a steady speed instead of speeding up forever")
{
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    RocketDesign design;
    design.setRoot(def("tank-small"));
    Rocket rocket;
    rocket.setVehicle(makeVehicle(design));
    rocket.place(earth, { system.body(earth).radius + 3000.0, 0.0, 0.0 }, {});
    double fastest = 0.0;
    for (int i = 0; i < 50 * 40 && !rocket.onGround(); i++)
    {
        rocket.step(1.0 / 50.0, system, i / 50.0);
        fastest = std::max(fastest, rocket.velocity().length());
    }
    CHECK(rocket.drag() > 0.0);
    CHECK(fastest < 200.0);
}

TEST_CASE("Under its parachute the pod comes down gently")
{
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    RocketDesign design;
    design.setRoot(def("pod"));
    design.autoStage();
    Rocket rocket;
    rocket.setVehicle(makeVehicle(design));
    // Coming down fast and drifting sideways; the pad sits on the +X axis, so down is -X
    rocket.place(earth, { system.body(earth).radius + 2000.0, 0.0, 0.0 }, { -120.0, 80.0, 0.0 });
    rocket.fireNextStage();

    double landing = 0.0;
    for (int i = 0; i < 50 * 600 && !rocket.onGround(); i++)
    {
        landing = rocket.velocity().length();
        rocket.step(1.0 / 50.0, system, i / 50.0);
    }
    REQUIRE(rocket.onGround());
    CHECK(landing < 10.0);
    CHECK(rocket.toWorld({ 0.0, 1.0, 0.0 }).dot(rocket.position().normalized()) > 0.99);
    CHECK(rocket.vehicle().canopy[0] == doctest::Approx(1.0));

    // Once down, it folds away and stops pulling
    for (int i = 0; i < 50 * 5; i++)
    {
        rocket.step(1.0 / 50.0, system, i / 50.0);
    }
    CHECK(rocket.vehicle().canopy[0] == 0.0);
}

TEST_CASE("A parachute released in space waits for air to fill it")
{
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    RocketDesign design;
    design.setRoot(def("pod"));
    design.autoStage();
    Rocket rocket;
    rocket.setVehicle(makeVehicle(design));
    rocket.place(earth, { system.body(earth).radius + 50.0e3, 0.0, 0.0 }, { 0.0, 0.0, -2000.0 });
    rocket.fireNextStage();
    rocket.step(1.0, system, 0.0);
    CHECK(rocket.vehicle().deployed[0]);
    CHECK(rocket.vehicle().canopy[0] == 0.0);
}

