#include <doctest/doctest.h>

#include <numbers>
#include <set>
#include <string>

#include <flight/PartCatalog.h>
#include <flight/RocketDesign.h>

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

    // Pod, small tank, and lifter engine stacked top to bottom
    RocketDesign simpleStack()
    {
        RocketDesign design;
        const PartIndex pod = design.setRoot(def("pod"));
        const PartIndex tank = design.attachStack(pod, Node::Bottom, def("tank-small"));
        design.attachStack(tank, Node::Bottom, def("engine-lifter"));
        return design;
    }
}

TEST_CASE("The standard catalog has unique ids and sensible parts")
{
    std::set<std::string> ids;
    for (const PartDefinition& part : catalog().parts())
    {
        CHECK(ids.insert(part.id).second);
        CHECK(part.dryMass > 0.0);
        CHECK(part.shape.height > 0.0);
        if (part.isEngine())
        {
            CHECK(part.ispVacuum >= part.ispSeaLevel);
        }
        if (part.mountsRadially)
        {
            CHECK_FALSE(part.hasTopNode);
            CHECK_FALSE(part.hasBottomNode);
        }
    }
    CHECK(catalog().find("pod") != nullptr);
    CHECK(catalog().find("no-such-part") == nullptr);
}

TEST_CASE("Stacked parts sit end to end below the root")
{
    const RocketDesign design = simpleStack();
    REQUIRE(design.partCount() == 3);
    CHECK(design.part(0).position.y == doctest::Approx(0.0));
    CHECK(design.part(1).position.y == doctest::Approx(-1.0)); // half the pod plus half the tank
    CHECK(design.part(2).position.y == doctest::Approx(-2.3)); // half the tank plus half the engine
    CHECK(design.part(2).parent == 1);
    CHECK(design.lowestPoint() == doctest::Approx(-3.1));
}

TEST_CASE("A node takes only one part, and parts need a matching node")
{
    RocketDesign design = simpleStack();

    CHECK_FALSE(design.isNodeFree(0, Node::Bottom));
    CHECK_FALSE(design.isNodeFree(1, Node::Top)); // its own link to the pod
    CHECK(design.isNodeFree(2, Node::Bottom));
    CHECK(design.attachStack(0, Node::Bottom, def("tank-small")) == NO_PART);

    // A nose cone has no top node, so it can't hang below anything, but it can sit on top of the pod
    CHECK_FALSE(design.canAttachStack(2, Node::Bottom, def("nose-cone")));
    const PartIndex nose = design.attachStack(0, Node::Top, def("nose-cone"));
    REQUIRE(nose != NO_PART);
    CHECK(design.part(nose).position.y == doctest::Approx(0.9));

    // Fins only go on the side
    CHECK_FALSE(design.canAttachStack(2, Node::Bottom, def("fin")));
    CHECK(design.attachStack(99, Node::Bottom, def("tank-small")) == NO_PART);
}

TEST_CASE("Radial parts sit against the parent's surface at the chosen angle and height")
{
    RocketDesign design = simpleStack();
    const PartIndex fin = design.attachRadial(1, 0.0, 0.25, def("fin"));
    REQUIRE(fin != NO_PART);
    const Vec3d position = design.part(fin).position;
    CHECK(position.z == doctest::Approx(0.625 + 0.25)); // tank radius plus half the fin's reach
    CHECK(position.x == doctest::Approx(0.0));
    CHECK(position.y == doctest::Approx(-1.0 + 0.25));

    // Heights past the parent's ends are pulled back onto it
    const PartIndex high = design.attachRadial(1, std::numbers::pi / 2.0, 10.0, def("fin"));
    CHECK(design.part(high).position.y == doctest::Approx(-0.5));
    CHECK(design.part(high).position.x == doctest::Approx(0.875));

    // On the pod's cone the surface is closer in near the top
    const PartIndex leg = design.attachRadial(0, 0.0, 0.5, def("landing-leg"));
    CHECK(design.part(leg).position.z == doctest::Approx(0.3 + 0.2));

    CHECK_FALSE(design.canAttachRadial(1, def("tank-small")));
    CHECK_FALSE(design.canAttachRadial(fin, def("fin")));
}

TEST_CASE("Removing a part takes everything attached through it and keeps the tree linked")
{
    RocketDesign design = simpleStack();
    design.attachRadial(2, 0.0, 0.0, def("fin"));
    design.attachRadial(2, std::numbers::pi, 0.0, def("fin"));
    const PartIndex chute = design.attachStack(0, Node::Top, def("parachute"));
    REQUIRE(design.partCount() == 6);

    CHECK(design.remove(1) == 4); // the tank, the engine below it, and both fins on the engine
    REQUIRE(design.partCount() == 2);
    CHECK(design.part(1).pDefinition->id == "parachute");
    CHECK(design.part(1).parent == 0);
    CHECK(design.isNodeFree(0, Node::Bottom));
    CHECK(chute == 5);

    CHECK(design.remove(0) == 2);
    CHECK(design.empty());
    CHECK(design.remove(0) == 0);
}

TEST_CASE("Mass and center of mass add up the parts with full tanks")
{
    RocketDesign design = simpleStack();
    CHECK(design.totalMass() == doctest::Approx(800.0 + 1125.0 + 1250.0));
    CHECK(design.dryMass() == doctest::Approx(800.0 + 125.0 + 1250.0));

    const double expectedY = (0.0 * 800.0 + -1.0 * 1125.0 + -2.3 * 1250.0) / design.totalMass();
    CHECK(design.centerOfMass().y == doctest::Approx(expectedY));
    CHECK(design.centerOfMass().x == doctest::Approx(0.0));

    // Fins in symmetry leave the center of mass on the axis; one alone pulls it to its side
    const PartIndex fin = design.attachRadial(2, 0.0, 0.0, def("fin"));
    CHECK(design.centerOfMass().z > 0.0);
    design.attachRadial(2, std::numbers::pi, 0.0, def("fin"));
    CHECK(design.centerOfMass().z == doctest::Approx(0.0));
    CHECK(fin == 3);

    CHECK(RocketDesign().centerOfMass() == Vec3d{});
}
