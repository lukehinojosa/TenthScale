#include <doctest/doctest.h>

#include <cmath>

#include <flight/DesignFile.h>
#include <flight/DesignStats.h>
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

    PartIndex findPart(const RocketDesign& design, const char* id)
    {
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            if (design.part(i).pDefinition->id == id)
            {
                return i;
            }
        }
        return NO_PART;
    }

    double totalFuel(const Vehicle& vehicle)
    {
        double total = 0.0;
        for (double fuel : vehicle.fuel)
        {
            total += fuel;
        }
        return total;
    }

    struct Flight
    {
        CircularSolarSystem system;
        BodyId earth = system.findBody("Earth");
        Rocket rocket;
        double time = 0.0;

        Flight()
        {
            const Vehicle vehicle = makeVehicle(twoStageRocket(catalog()));
            rocket.setVehicle(vehicle);
            rocket.place(earth, { system.body(earth).radius + vehicle.groundClearance, 0.0, 0.0 }, {});
        }

        void run(double seconds)
        {
            const int steps = (int)(seconds * 50.0);
            for (int i = 0; i < steps; i++)
            {
                rocket.step(1.0 / 50.0, system, time);
                time += 1.0 / 50.0;
            }
        }
    };
}

TEST_CASE("Splitting at the decoupler shares out the parts and fuel, and uncovers the upper engine's bell")
{
    Vehicle vehicle = makeVehicle(twoStageRocket(catalog()));
    const int parts = vehicle.design.partCount();
    const double mass = vehicle.mass;
    const double fuel = totalFuel(vehicle);
    const PartIndex vacuum = findPart(vehicle.design, "engine-vacuum");
    REQUIRE(vehicle.design.isShrouded(vacuum));

    Vec3d rootOffset;
    const Vehicle lower = detachBranch(vehicle, findPart(vehicle.design, "decoupler"), rootOffset);
    CHECK(lower.design.part(0).pDefinition->id == "decoupler");
    CHECK(lower.design.partCount() + vehicle.design.partCount() == parts);
    CHECK(lower.mass + vehicle.mass == doctest::Approx(mass));
    CHECK(totalFuel(lower) + totalFuel(vehicle) == doctest::Approx(fuel));
    CHECK(rootOffset.y < 0.0);
    CHECK_FALSE(vehicle.design.isShrouded(findPart(vehicle.design, "engine-vacuum")));
    CHECK(findPart(vehicle.design, "engine-lifter") == NO_PART);
}

TEST_CASE("Dropping a stage in flight keeps the pieces' shared balance point and momentum, and pushes them apart")
{
    Flight flight;
    flight.rocket.fireNextStage();
    flight.run(5.0);
    const double massBefore = flight.rocket.vehicle().mass;
    const Vec3d positionBefore = flight.rocket.position();
    const Vec3d velocityBefore = flight.rocket.velocity();

    flight.rocket.fireNextStage();
    std::vector<Rocket> dropped = flight.rocket.takeDropped();
    REQUIRE(dropped.size() == 1);
    const Rocket& lower = dropped.front();
    const double kept = flight.rocket.vehicle().mass;
    const double gone = lower.vehicle().mass;
    CHECK(kept + gone == doctest::Approx(massBefore));

    const Vec3d centre = (flight.rocket.position() * kept + lower.position() * gone) / (kept + gone);
    CHECK((centre - positionBefore).length() < 1e-6);
    const Vec3d momentum = flight.rocket.velocity() * kept + lower.velocity() * gone;
    CHECK((momentum - velocityBefore * massBefore).length() < 1e-6 * massBefore);

    // The upper part moves up the rocket's axis, away from the dropped stage
    const Vec3d up = flight.rocket.toWorld({ 0.0, 1.0, 0.0 });
    CHECK((flight.rocket.velocity() - lower.velocity()).dot(up) == doctest::Approx(Rocket::DECOUPLER_SPEED));
    CHECK((flight.rocket.position() - lower.position()).dot(up) > 0.0);
    CHECK(flight.rocket.takeDropped().empty());

    // The charge went off at the decoupler, between the two pieces, and lined up with them, so neither turns
    const std::vector<Separation> separations = flight.rocket.takeSeparations();
    REQUIRE(separations.size() == 1);
    const double aboveLower = (separations.front().site - lower.position()).dot(up);
    const double belowUpper = (flight.rocket.position() - separations.front().site).dot(up);
    CHECK(aboveLower > 0.0);
    CHECK(belowUpper > 0.0);
    CHECK(separations.front().size == 1.0);
    CHECK(flight.rocket.spin().length() < 1e-9);
    CHECK(lower.spin().length() < 1e-9);
    CHECK(flight.rocket.takeSeparations().empty());
}

TEST_CASE("A dropped stage's engines shut off as it leaves")
{
    Flight flight;
    flight.rocket.fireNextStage();
    flight.run(5.0);
    flight.rocket.fireNextStage();
    Rocket lower = flight.rocket.takeDropped().front();
    for (bool lit : lower.vehicle().lit)
    {
        CHECK_FALSE(lit);
    }
    lower.step(1.0 / 50.0, flight.system, flight.time);
    CHECK(lower.thrust() == 0.0);
}

TEST_CASE("A split reports how far the rocket's center jumped")
{
    Flight flight;
    flight.rocket.fireNextStage();
    flight.run(5.0);
    const Vec3d before = flight.rocket.position();
    flight.rocket.fireNextStage();
    const std::vector<Separation> separations = flight.rocket.takeSeparations();
    REQUIRE(separations.size() == 1);
    CHECK((flight.rocket.position() - before - separations.front().shift).length() < 1e-9);
    CHECK(separations.front().shift.length() > 1.0);
}

TEST_CASE("After staging, the delta-v left is the upper stage's")
{
    Flight flight;
    flight.rocket.fireNextStage();
    flight.run(200.0);
    flight.rocket.fireNextStage();
    const DesignStats left = computeStats(flight.rocket.vehicle(), 9.81);
    const DesignStats full = computeStats(twoStageRocket(catalog()), 9.81);
    CHECK(left.totalDeltaVVacuum == doctest::Approx(full.stages[1].deltaVVacuum).epsilon(0.01));
}

TEST_CASE("The nose cone pops off at the top of the climb, and only once")
{
    Flight flight;
    flight.rocket.fireNextStage();
    bool popped = false;
    double climbAtPop = 0.0;
    for (int second = 0; second < 600 && !popped; second++)
    {
        flight.run(1.0);
        const std::vector<Rocket> dropped = flight.rocket.takeDropped();
        if (!dropped.empty())
        {
            popped = true;
            CHECK(dropped.size() == 1);
            CHECK(dropped.front().vehicle().design.part(0).pDefinition->popsOffAtPeak);
            climbAtPop = flight.rocket.velocity().dot(flight.rocket.position().normalized());
        }
    }
    REQUIRE(popped);
    CHECK(std::abs(climbAtPop) < 15.0);
    CHECK(findPart(flight.rocket.vehicle().design, "nose-cone") == NO_PART);
    flight.run(30.0);
    CHECK(flight.rocket.takeDropped().empty());
}

TEST_CASE("Opening the parachute pops the nose cone first if it's still on")
{
    Flight flight;
    flight.rocket.fireNextStage();
    flight.rocket.fireNextStage();
    flight.rocket.takeDropped();
    flight.rocket.fireNextStage();
    const std::vector<Rocket> dropped = flight.rocket.takeDropped();
    REQUIRE(dropped.size() == 1);
    CHECK(dropped.front().vehicle().design.part(0).pDefinition->id == "nose-cone");
    const std::vector<Separation> separations = flight.rocket.takeSeparations();
    REQUIRE(separations.size() == 2);
    CHECK(separations.back().size < separations.front().size);

    // The parachute shoves the cone from where they meet: below the cone, above the rest of the rocket
    const Vec3d up = flight.rocket.toWorld({ 0.0, 1.0, 0.0 });
    const Vec3d site = separations.back().site;
    CHECK((dropped.front().position() - site).dot(up) > 0.0);
    CHECK((site - flight.rocket.position()).dot(up) > 0.0);
    CHECK((dropped.front().velocity() - flight.rocket.velocity()).dot(up) == doctest::Approx(Rocket::NOSE_CONE_SPEED));
    const PartIndex pod = findPart(flight.rocket.vehicle().design, "pod");
    CHECK(flight.rocket.vehicle().deployed[pod]);
}

TEST_CASE("A point inside a part finds the nearest way out")
{
    RocketDesign design;
    design.setRoot(*catalog().find("tank-large"));
    const PartShape& shape = design.part(0).shape;
    double depth = 0.0;
    Vec3d outward;
    CHECK(insidePart(design, 0, { 0.0, shape.height * 0.5 - 0.1, 0.0 }, depth, outward));
    CHECK(depth == doctest::Approx(0.1));
    CHECK(outward.y == doctest::Approx(1.0));
    CHECK(insidePart(design, 0, { shape.topDiameter * 0.5 - 0.05, 0.0, 0.0 }, depth, outward));
    CHECK(depth == doctest::Approx(0.05));
    CHECK(outward.x == doctest::Approx(1.0));
    CHECK_FALSE(insidePart(design, 0, { shape.topDiameter, 0.0, 0.0 }, depth, outward));
    CHECK_FALSE(insidePart(design, 0, { 0.0, shape.height, 0.0 }, depth, outward));

    // A fin on the +X side is thin along Z there and reaches out along X
    const PartIndex fin = design.attachRadial(0, std::acos(0.0), 0.0, *catalog().find("fin"));
    const Vec3d centre = design.part(fin).position;
    CHECK(insidePart(design, fin, centre + Vec3d{ 0.0, 0.0, 0.01 }, depth, outward));
    CHECK(std::abs(outward.z) == doctest::Approx(1.0));
}

namespace
{
    // Steps the rocket and a piece it dropped together, letting them knock into each other
    void runPair(Flight& flight, Rocket& piece, double seconds)
    {
        const int steps = (int)(seconds * 50.0);
        for (int i = 0; i < steps; i++)
        {
            flight.rocket.step(1.0 / 50.0, flight.system, flight.time);
            piece.step(1.0 / 50.0, flight.system, flight.time);
            collide(flight.rocket, piece, 1.0 / 50.0);
            flight.time += 1.0 / 50.0;
        }
    }
}

TEST_CASE("Pieces that have just come apart drift off without catching on each other")
{
    Flight flight;
    flight.rocket.fireNextStage();
    flight.run(5.0);
    flight.rocket.setThrottle(0.0);
    flight.rocket.fireNextStage();
    Rocket lower = flight.rocket.takeDropped().front();

    // Short of the top of the climb, where the nose cone popping off would nudge the rocket
    runPair(flight, lower, 1.0);
    const Vec3d up = flight.rocket.toWorld({ 0.0, 1.0, 0.0 });
    CHECK((flight.rocket.velocity() - lower.velocity()).dot(up) == doctest::Approx(Rocket::DECOUPLER_SPEED).epsilon(0.001));
}

TEST_CASE("A dropped stage thrown back at the rocket pushes it along instead of flying through it")
{
    Flight flight;
    flight.rocket.fireNextStage();
    flight.run(5.0);
    flight.rocket.fireNextStage();
    Rocket lower = flight.rocket.takeDropped().front();
    const Vec3d up = flight.rocket.toWorld({ 0.0, 1.0, 0.0 });
    const double gap = (flight.rocket.position() - lower.position()).dot(up);
    lower.place(flight.earth, lower.position(), flight.rocket.velocity() + up * 10.0);

    double closest = gap;
    for (int i = 0; i < 150; i++)
    {
        runPair(flight, lower, 1.0 / 50.0);
        closest = std::min(closest, (flight.rocket.position() - lower.position()).dot(up));
    }
    // Thrown at 10 m/s it gets a step's worth in before the contact catches it, but no further
    CHECK(closest > gap - 0.5);

    // It bounced off rather than carrying on through
    CHECK((lower.velocity() - flight.rocket.velocity()).dot(up) < 1.0);
}


TEST_CASE("The default rocket lifts off, then stages down to the pod alone under its parachute")
{
    CircularSolarSystem system;
    const BodyId earth = system.findBody("Earth");
    const Vehicle vehicle = makeVehicle(defaultRocket(catalog()));
    Rocket rocket;
    rocket.setVehicle(vehicle);
    rocket.place(earth, { system.body(earth).radius + vehicle.groundClearance, 0.0, 0.0 }, {});
    rocket.fireNextStage();
    for (int i = 0; i < 50 * 10; i++)
    {
        rocket.step(1.0 / 50.0, system, i / 50.0);
    }
    CHECK(rocket.altitude(system) > 20.0);
    CHECK(rocket.toWorld({ 0.0, 1.0, 0.0 }).dot(rocket.position().normalized()) == doctest::Approx(1.0));

    // Each later stage drops what's under it: three spent stages and the lander, then the parachute pops the cone
    int dropped = 0;
    for (int stage = 1; stage < vehicle.design.stageCount(); stage++)
    {
        rocket.fireNextStage();
        dropped += (int)rocket.takeDropped().size();
    }
    CHECK(dropped == 5);
    REQUIRE(rocket.vehicle().design.partCount() == 1);
    CHECK(rocket.vehicle().design.part(0).pDefinition->id == "pod");
    CHECK(rocket.vehicle().deployed[0]);
}
