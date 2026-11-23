#include <doctest/doctest.h>

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

    // Coasting high above the air, so only the wheels turn it
    struct SpaceTest
    {
        CircularSolarSystem system;
        BodyId earth = system.findBody("Earth");
        Rocket rocket;
        double time = 0.0;

        explicit SpaceTest(const RocketDesign& design)
        {
            rocket.setVehicle(makeVehicle(design));
            rocket.place(earth, { system.body(earth).radius + 100.0e3, 0.0, 0.0 }, {});
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

        // Its spin about one of its own axes, in radians per second
        double spinAbout(const Vec3d& axis) const
        {
            return rocket.toLocal(rocket.spin()).dot(axis);
        }
    };

    RocketDesign podAndTank()
    {
        RocketDesign design;
        const PartIndex pod = design.setRoot(*catalog().find("pod"));
        design.attachStack(pod, Node::Bottom, *catalog().find("tank-large"));
        return design;
    }
}

TEST_CASE("Only the pod carries reaction wheels")
{
    const Vehicle vehicle = makeVehicle(twoStageRocket(catalog()));
    CHECK(wheelTorque(vehicle) == doctest::Approx(catalog().find("pod")->wheelTorque));
    CHECK(catalog().find("pod")->wheelTorque > 0.0);
    CHECK(catalog().find("tank-large")->wheelTorque == 0.0);
}

TEST_CASE("Pitching spins the rocket about its own x axis at the wheels' torque over its inertia")
{
    SpaceTest test(twoStageRocket(catalog()));
    test.rocket.setSteering({ 1.0, 0.0, 0.0 });
    test.run(2.0);
    const Vehicle& vehicle = test.rocket.vehicle();
    const double expected = wheelTorque(vehicle) / vehicle.inertia.xx * 2.0;
    CHECK(test.spinAbout({ 1.0, 0.0, 0.0 }) == doctest::Approx(expected).epsilon(0.01));
    CHECK(std::abs(test.spinAbout({ 0.0, 1.0, 0.0 })) < expected * 0.01);
    CHECK(std::abs(test.spinAbout({ 0.0, 0.0, 1.0 })) < expected * 0.01);
}

TEST_CASE("Each steering axis turns the rocket its own way, and backwards when reversed")
{
    const Vec3d axes[] = { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } };
    for (const Vec3d& axis : axes)
    {
        SpaceTest forward(podAndTank());
        forward.rocket.setSteering(axis);
        forward.run(0.5);
        SpaceTest backward(podAndTank());
        backward.rocket.setSteering(-axis);
        backward.run(0.5);
        CHECK(forward.spinAbout(axis) > 0.0);
        CHECK(backward.spinAbout(axis) == doctest::Approx(-forward.spinAbout(axis)));
    }
}

TEST_CASE("The same wheels turn a heavier rocket slower, and roll it quicker than they pitch it")
{
    SpaceTest light(podAndTank());
    SpaceTest heavy(twoStageRocket(catalog()));
    light.rocket.setSteering({ 1.0, 0.0, 0.0 });
    heavy.rocket.setSteering({ 1.0, 0.0, 0.0 });
    light.run(1.0);
    heavy.run(1.0);
    CHECK(heavy.spinAbout({ 1.0, 0.0, 0.0 }) < light.spinAbout({ 1.0, 0.0, 0.0 }) * 0.5);

    SpaceTest rolling(twoStageRocket(catalog()));
    rolling.rocket.setSteering({ 0.0, 1.0, 0.0 });
    rolling.run(1.0);
    CHECK(rolling.spinAbout({ 0.0, 1.0, 0.0 }) > heavy.spinAbout({ 1.0, 0.0, 0.0 }) * 10.0);
}

TEST_CASE("Letting go of the keys stops the wheels pushing, and the rocket keeps the spin it had")
{
    SpaceTest test(twoStageRocket(catalog()));
    test.rocket.setSteering({ 0.0, 0.0, 1.0 });
    test.run(1.0);
    const double spun = test.spinAbout({ 0.0, 0.0, 1.0 });
    test.rocket.setSteering({});
    test.run(1.0);
    CHECK(test.spinAbout({ 0.0, 0.0, 1.0 }) == doctest::Approx(spun));
}

TEST_CASE("Steering is held to full deflection")
{
    Rocket rocket;
    rocket.setSteering({ 3.0, -2.0, 0.5 });
    CHECK(rocket.steering().x == 1.0);
    CHECK(rocket.steering().y == -1.0);
    CHECK(rocket.steering().z == 0.5);
}

TEST_CASE("A dropped stage has no steering of its own")
{
    SpaceTest test(twoStageRocket(catalog()));
    test.rocket.fireNextStage();
    test.rocket.setSteering({ 1.0, 1.0, 1.0 });
    test.rocket.fireNextStage();
    for (const Rocket& piece : test.rocket.takeDropped())
    {
        CHECK(piece.steering().lengthSquared() == 0.0);
    }
}
