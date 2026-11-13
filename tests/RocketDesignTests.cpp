#include <doctest/doctest.h>

#include <cmath>
#include <numbers>
#include <set>
#include <string>
#include <vector>

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

    // The pod carries the parachute, so the nose cone is free to cap it
    CHECK(catalog().find("parachute") == nullptr);
    CHECK(def("pod").hasParachute);
    CHECK(def("nose-cone").popsOffAtPeak);
    RocketDesign capped;
    capped.setRoot(def("pod"));
    CHECK(capped.attachStack(0, Node::Top, def("nose-cone")) != NO_PART);
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
    CHECK(design.highestPoint() == doctest::Approx(0.5));
    CHECK(RocketDesign().highestPoint() == doctest::Approx(0.0));
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
    CHECK(design.part(nose).position.y == doctest::Approx(0.5 + 0.8 * 0.6 / 1.25 * 0.5)); // shrunk to the pod's narrow top

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
    const PartIndex chute = design.attachStack(0, Node::Top, def("nose-cone"));
    REQUIRE(design.partCount() == 6);

    CHECK(design.remove(1) == 4); // the tank, the engine below it, and both fins on the engine
    REQUIRE(design.partCount() == 2);
    CHECK(design.part(1).pDefinition->id == "nose-cone");
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

TEST_CASE("Extracting a branch gives a design rooted at that part, and attaching it back restores the rocket")
{
    RocketDesign design = simpleStack();
    design.attachRadial(2, 0.0, 0.0, def("fin"));
    const RocketDesign original = design;

    RocketDesign branch = design.extract(1);
    REQUIRE(branch.partCount() == 3); // tank, engine, and the fin on the engine
    REQUIRE(design.partCount() == 1);
    CHECK(branch.part(0).attachment == Attachment::Root);
    CHECK(branch.part(0).parent == NO_PART);
    CHECK(branch.part(0).position == Vec3d{});
    CHECK(branch.part(1).position.y == doctest::Approx(-1.3));
    CHECK(branch.part(2).parent == 1);
    CHECK(branch.part(2).position.z == doctest::Approx((0.9 + 1.25) * 0.25 + 0.25)); // the engine's radius halfway up plus half the fin

    const PartIndex root = design.attachDesign(0, Node::Bottom, branch);
    REQUIRE(root == 1);
    REQUIRE(design.partCount() == original.partCount());
    for (PartIndex i = 0; i < design.partCount(); i++)
    {
        CHECK(design.part(i).pDefinition == original.part(i).pDefinition);
        CHECK(design.part(i).parent == original.part(i).parent);
        CHECK(design.part(i).attachment == original.part(i).attachment);
        CHECK((design.part(i).position - original.part(i).position).length() == doctest::Approx(0.0));
    }
}

TEST_CASE("A subassembly attaches only by a node its root still has free")
{
    RocketDesign design;
    design.setRoot(def("pod"));

    // The tank's top was used by the pod, and its bottom by the engine, so this branch can only hang by its top
    RocketDesign rocket = simpleStack();
    const RocketDesign branch = rocket.extract(1);
    CHECK(branch.isNodeFree(0, Node::Top));
    CHECK_FALSE(branch.isNodeFree(0, Node::Bottom));
    CHECK(design.canAttachDesign(0, Node::Bottom, branch));
    CHECK_FALSE(design.canAttachDesign(0, Node::Top, branch));

    // Pods have a top node, so a branch fits on top of one too when its root's bottom is free
    RocketDesign chute;
    chute.setRoot(def("nose-cone"));
    CHECK(design.canAttachDesign(0, Node::Top, chute));
    CHECK_FALSE(design.canAttachDesign(0, Node::Top, RocketDesign()));

    // Extracting the root takes everything
    RocketDesign whole = simpleStack();
    const RocketDesign everything = whole.extract(0);
    CHECK(whole.empty());
    CHECK(everything.partCount() == 3);
    CHECK(everything.part(0).pDefinition->id == "pod");
}

TEST_CASE("Symmetry places copies evenly around the parent and groups them")
{
    RocketDesign design = simpleStack();
    RocketDesign fin;
    fin.setRoot(def("fin"));

    const std::vector<PartIndex> roots = design.attachDesignRadial(1, 0.0, 0.0, fin, 3);
    REQUIRE(roots.size() == 3);
    const double out = 0.625 + 0.25;
    for (int k = 0; k < 3; k++)
    {
        const double angle = 2.0 * std::numbers::pi * k / 3.0;
        const PlacedPart& placed = design.part(roots[k]);
        CHECK(placed.angle == doctest::Approx(angle));
        CHECK(placed.position.x == doctest::Approx(std::sin(angle) * out));
        CHECK(placed.position.z == doctest::Approx(std::cos(angle) * out));
        CHECK(placed.position.y == doctest::Approx(-1.0));
        CHECK(placed.parent == 1);
        CHECK(placed.symmetryGroup != 0);
        CHECK(placed.symmetryGroup == design.part(roots[0]).symmetryGroup);
    }
    CHECK(design.symmetryGroup(roots[1]) == roots);
    CHECK(design.centerOfMass().x == doctest::Approx(0.0));
    CHECK(design.centerOfMass().z == doctest::Approx(0.0));

    // A second set gets its own group, and a single part stays ungrouped
    const std::vector<PartIndex> legs = design.attachDesignRadial(2, 0.0, 0.0, RocketDesign(), 2);
    CHECK(legs.empty());
    RocketDesign leg;
    leg.setRoot(def("landing-leg"));
    const std::vector<PartIndex> pair = design.attachDesignRadial(2, 0.5, 0.0, leg, 2);
    REQUIRE(pair.size() == 2);
    CHECK(design.part(pair[0]).symmetryGroup != design.part(roots[0]).symmetryGroup);
    CHECK(design.part(pair[1]).angle == doctest::Approx(0.5 + std::numbers::pi));
    const std::vector<PartIndex> single = design.attachDesignRadial(2, 1.0, 0.0, leg, 1);
    CHECK(design.part(single[0]).symmetryGroup == 0);
    CHECK(design.symmetryGroup(single[0]) == single);

    CHECK(design.attachDesignRadial(1, 0.0, 0.0, fin, 0).empty());
}

TEST_CASE("Removing or picking up one symmetry copy takes the whole set")
{
    RocketDesign design = simpleStack();
    RocketDesign fin;
    fin.setRoot(def("fin"));
    const std::vector<PartIndex> fins = design.attachDesignRadial(2, 0.0, 0.0, fin, 4);
    design.attachStack(0, Node::Top, def("nose-cone"));
    REQUIRE(design.partCount() == 8);

    RocketDesign copy = design;
    CHECK(copy.removeWithSymmetry(fins[2]) == 4);
    CHECK(copy.partCount() == 4);
    CHECK(copy.part(3).pDefinition->id == "nose-cone");

    const RocketDesign held = design.extractWithSymmetry(fins[1]);
    REQUIRE(held.partCount() == 1);
    CHECK(held.part(0).symmetryGroup == 0);
    CHECK(held.part(0).pDefinition->id == "fin");
    CHECK(design.partCount() == 4);
    for (PartIndex i = 0; i < design.partCount(); i++)
    {
        CHECK(design.part(i).pDefinition->id != "fin");
    }

    // Removing an ungrouped part takes only its branch, as remove() does
    CHECK(design.removeWithSymmetry(3) == 1);
    CHECK(design.partCount() == 3);
}

TEST_CASE("A nose cone resizes to the diameter it sits on, keeping its proportions")
{
    // On the pod's narrow top it shrinks to the pod's 0.6 m
    RocketDesign design;
    const PartIndex pod = design.setRoot(def("pod"));
    const PartIndex onPod = design.attachStack(pod, Node::Top, def("nose-cone"));
    const PartShape& small = design.part(onPod).shape;
    CHECK(small.bottomDiameter == doctest::Approx(0.6));
    CHECK(small.topDiameter == doctest::Approx(0.0));
    CHECK(small.height == doctest::Approx(0.8 * 0.6 / 1.25));
    CHECK(design.highestPoint() == doctest::Approx(0.5 + small.height));

    // On a full-width tank it keeps its catalog size
    RocketDesign tankTop;
    const PartIndex tank = tankTop.setRoot(def("tank-small"));
    const PartIndex onTank = tankTop.attachStack(tank, Node::Top, def("nose-cone"));
    CHECK(tankTop.part(onTank).shape.bottomDiameter == doctest::Approx(1.25));
    CHECK(tankTop.part(onTank).shape.height == doctest::Approx(0.8));

    // Moving it from the tank to the pod refits it to the new spot
    const RocketDesign held = tankTop.extract(onTank);
    const PartIndex moved = design.remove(onPod) == 1 ? design.attachDesign(pod, Node::Top, held) : NO_PART;
    REQUIRE(moved != NO_PART);
    CHECK(design.part(moved).shape.bottomDiameter == doctest::Approx(0.6));

    // Other parts never resize
    const PartIndex below = design.attachStack(pod, Node::Bottom, def("tank-small"));
    CHECK(design.part(below).shape.topDiameter == doctest::Approx(1.25));
}
