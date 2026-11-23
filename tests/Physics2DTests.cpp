#include <doctest/doctest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <cyclone2d/cyclone2d.h>

using namespace cyclone2d;

namespace
{
    constexpr real DT = (real)0.01;
    constexpr real PI = (real)3.14159265358979;

    struct Box
    {
        RigidBody body;
        BoxCollider collider;
    };

    struct Ball
    {
        RigidBody body;
        CircleCollider collider;
    };

    std::unique_ptr<Box> makeBox(World& world, Vector2 position, Vector2 halfSize, real mass, real angle = 0)
    {
        auto pBox = std::make_unique<Box>();
        pBox->body.setPosition(position);
        pBox->body.setOrientation(angle);
        pBox->body.setBoxMass(mass, halfSize);
        pBox->collider.setHalfSize(halfSize);
        pBox->collider.pBody = &pBox->body;
        world.bodies.push_back(&pBox->body);
        world.colliders.push_back(&pBox->collider);
        return pBox;
    }

    std::unique_ptr<Ball> makeBall(World& world, Vector2 position, real radius, real mass)
    {
        auto pBall = std::make_unique<Ball>();
        pBall->body.setPosition(position);
        pBall->body.setCircleMass(mass, radius);
        pBall->collider.radius = radius;
        pBall->collider.pBody = &pBall->body;
        world.bodies.push_back(&pBall->body);
        world.colliders.push_back(&pBall->collider);
        return pBall;
    }

    void run(World& world, real seconds)
    {
        for (int i = 0; i < (int)(seconds / DT); i++)
        {
            world.runPhysics(DT);
        }
    }

    void runParticles(ParticleWorld& world, real seconds)
    {
        for (int i = 0; i < (int)(seconds / DT); i++)
        {
            world.runPhysics(DT);
        }
    }

    // How far an angle is from the nearest quarter turn, so a box lying on any side reads as flat
    real offSquare(real angle)
    {
        const real quarter = PI / 2;
        const real r = std::fmod(std::abs(angle), quarter);
        return std::min(r, quarter - r);
    }

    std::vector<Contact> contactsBetween(const Collider& a, const Collider& b)
    {
        std::vector<Contact> contacts(8);
        CollisionData data;
        data.contactArray = contacts.data();
        data.reset((int)contacts.size());
        CollisionTests::Generate(a, b, &data);
        contacts.resize((std::size_t)data.contactCount);
        return contacts;
    }
}

TEST_CASE("2D vectors cross, turn and transform as expected")
{
    const Vector2 x(1, 0);
    const Vector2 y(0, 1);
    CHECK(x.cross(y) == doctest::Approx(1));
    CHECK(y.cross(x) == doctest::Approx(-1));
    CHECK(x.perpendicular() == y);
    CHECK(cross(2, x) == Vector2(0, 2));

    const Rotation quarter(PI / 2);
    const Vector2 turned = quarter.rotate(Vector2(2, 0));
    CHECK(turned.x == doctest::Approx(0).epsilon(1e-5));
    CHECK(turned.y == doctest::Approx(2));
    CHECK(quarter.getAngle() == doctest::Approx(PI / 2));

    const Transform t(Vector2(5, 1), PI / 2);
    const Vector2 world = t.transform(Vector2(1, 0));
    CHECK(world.x == doctest::Approx(5));
    CHECK(world.y == doctest::Approx(2));
    const Vector2 back = t.transformInverse(world);
    CHECK(back.x == doctest::Approx(1));
    CHECK(back.y == doctest::Approx(0).epsilon(1e-5));
}

TEST_CASE("A polygon takes the convex hull of its points and knows its area and inertia")
{
    // Out of order, clockwise, with a point inside that should be dropped
    PolygonCollider polygon({ Vector2(1, 1), Vector2(1, -1), Vector2(0, 0), Vector2(-1, -1), Vector2(-1, 1) });
    REQUIRE(polygon.getVertexCount() == 4);
    CHECK(polygon.getArea() == doctest::Approx(4));
    CHECK(polygon.getCentroid().magnitude() == doctest::Approx(0));

    // Counter-clockwise, so every normal points out of the shape
    for (int i = 0; i < polygon.getVertexCount(); i++)
    {
        CHECK(polygon.getNormals()[(std::size_t)i].dot(polygon.getVertices()[(std::size_t)i]) > 0);
    }

    RigidBody body;
    body.setBoxMass(3, Vector2(1, 1));
    CHECK(polygon.computeInertia(3) == doctest::Approx(1 / body.getInverseInertia()));

    const PolygonCollider hexagon = PolygonCollider::regular(6, 1);
    CHECK(hexagon.getVertexCount() == 6);
    CHECK(hexagon.getBoundingRadius() == doctest::Approx(1));
}

TEST_CASE("A particle on a 2D cable hangs at the cable's length")
{
    Particle particle(Vector2(2, 0), Vector2(), Vector2::GRAVITY, (real)0.9);
    ParticleCableConstraint cable;
    cable.particle = &particle;
    cable.anchor = Vector2(0, 0);
    cable.maxLength = 3;

    ParticleWorld world;
    world.particles.push_back(&particle);
    world.contactGenerators.push_back(&cable);
    runParticles(world, 10);

    CHECK((particle.getPosition() - cable.anchor).magnitude() == doctest::Approx(3).epsilon(0.02));
    CHECK(particle.getPosition().y < -2.9f);
}

TEST_CASE("2D particles land on a sloped line and a rod keeps two apart")
{
    Particle a(Vector2(0, 3), Vector2(2, 0));
    Particle b(Vector2(0, 4), Vector2(-1, 1));
    std::vector<Particle*> particles = { &a, &b };
    ParticleRod rod;
    rod.particle[0] = &a;
    rod.particle[1] = &b;
    rod.length = 1;
    ParticleLineContacts ground;
    ground.pParticles = &particles;
    ground.normal = Vector2(-(real)0.2, 1).unit();

    ParticleWorld world;
    world.particles = particles;
    world.contactGenerators = { &rod, &ground };

    // In the air the rod holds exactly; a landing can leave it off for one frame, as in the book
    for (int i = 0; i < 50; i++)
    {
        world.runPhysics(DT);
        REQUIRE((a.getPosition() - b.getPosition()).magnitude() == doctest::Approx(1).epsilon(0.001));
    }
    runParticles(world, 3);
    CHECK((a.getPosition() - b.getPosition()).magnitude() == doctest::Approx(1).epsilon(0.01));
    CHECK(a.getPosition().dot(ground.normal) > -0.05f);
    CHECK(b.getPosition().dot(ground.normal) > -0.05f);
}

TEST_CASE("A 2D particle floats at the depth where buoyancy balances gravity")
{
    Particle particle(Vector2(0, 5), Vector2(), Vector2(), (real)0.5);
    particle.setMass(1);
    ParticleGravity gravity;
    ParticleBuoyancy buoyancy(1, (real)0.004, 0);

    ParticleWorld world;
    world.particles.push_back(&particle);
    world.forces.add(&particle, &gravity);
    world.forces.add(&particle, &buoyancy);
    runParticles(world, 30);

    // A quarter of the full lift holds it up, so it floats half a unit above the surface
    const real full = 1000 * (real)0.004 * (real)9.81;
    const real expected = 1 - 2 * (real)9.81 / full;
    CHECK(particle.getPosition().y == doctest::Approx(expected).epsilon(0.02));
}

TEST_CASE("Overlapping circles give one contact pushing them apart")
{
    CircleCollider a(1);
    CircleCollider b(1);
    a.offset.position = Vector2((real)1.5, 0);
    a.updateInternals();
    b.updateInternals();

    const std::vector<Contact> contacts = contactsBetween(a, b);
    REQUIRE(contacts.size() == 1);
    CHECK(contacts[0].normal.x == doctest::Approx(1));
    CHECK(contacts[0].penetration == doctest::Approx(0.5));
    CHECK(contacts[0].point.x == doctest::Approx(0.75));
}

TEST_CASE("A box resting on the ground or on another box touches at both corners")
{
    PlaneCollider ground;
    BoxCollider box(Vector2(1, (real)0.5));
    box.offset.position = Vector2(0, (real)0.45);
    box.updateInternals();
    ground.updateInternals();

    std::vector<Contact> contacts = contactsBetween(box, ground);
    REQUIRE(contacts.size() == 2);
    for (const Contact& contact : contacts)
    {
        CHECK(contact.normal.y == doctest::Approx(1));
        CHECK(contact.penetration == doctest::Approx(0.05));
    }

    // Off to one side, so the contacts sit where the boxes overlap, and the normal points from the second to the first
    BoxCollider top(Vector2((real)0.5, (real)0.5));
    top.offset.position = Vector2((real)0.8, (real)1.4);
    top.updateInternals();
    contacts = contactsBetween(top, box);
    REQUIRE(contacts.size() == 2);
    for (const Contact& contact : contacts)
    {
        CHECK(contact.normal.y == doctest::Approx(1));
        CHECK(contact.penetration == doctest::Approx(0.05));
        CHECK(contact.point.x >= (real)0.3 - (real)1e-4);
        CHECK(contact.point.x <= (real)1.0 + (real)1e-4);
    }

    contacts = contactsBetween(box, top);
    REQUIRE(contacts.size() == 2);
    CHECK(contacts[0].normal.y == doctest::Approx(-1));
}

TEST_CASE("A circle near a box's corner is pushed out along the line from the corner")
{
    BoxCollider box(Vector2(1, 1));
    CircleCollider circle((real)0.5);
    circle.offset.position = Vector2((real)1.3, (real)1.3);
    box.updateInternals();
    circle.updateInternals();

    const std::vector<Contact> contacts = contactsBetween(box, circle);
    REQUIRE(contacts.size() == 1);
    const real diagonal = 1 / std::sqrt((real)2);
    CHECK(contacts[0].normal.x == doctest::Approx(-diagonal));
    CHECK(contacts[0].normal.y == doctest::Approx(-diagonal));
    CHECK(contacts[0].penetration == doctest::Approx(0.5 - 0.3 * std::sqrt(2.0)).epsilon(0.001));

    circle.offset.position = Vector2((real)2.0, (real)2.0);
    circle.updateInternals();
    CHECK(contactsBetween(box, circle).empty());
}

TEST_CASE("A dropped box comes to rest flat on the ground and falls asleep")
{
    World world;
    PlaneCollider ground;
    world.colliders.push_back(&ground);
    auto pBox = makeBox(world, Vector2(0, 3), Vector2((real)0.5, (real)0.5), 1, (real)0.4);
    run(world, 6);

    CHECK(pBox->body.getPosition().y == doctest::Approx(0.5).epsilon(0.02));
    CHECK(offSquare(pBox->body.getOrientation()) < 0.02f);
    CHECK_FALSE(pBox->body.isAwake());
}

TEST_CASE("A stack of 2D boxes stays standing and sleeps")
{
    World world;
    PlaneCollider ground;
    world.colliders.push_back(&ground);
    std::vector<std::unique_ptr<Box>> boxes;
    for (int i = 0; i < 5; i++)
    {
        boxes.push_back(makeBox(world, Vector2(0, (real)0.5 + (real)i), Vector2((real)0.5, (real)0.5), 1));
    }
    run(world, 8);

    for (int i = 0; i < 5; i++)
    {
        const Vector2 position = boxes[(std::size_t)i]->body.getPosition();
        CHECK(std::abs(position.x) < 0.05f);
        CHECK(position.y == doctest::Approx((real)0.5 + (real)i).epsilon(0.03));
    }
    CHECK(world.getAwakeCount() == 0);
}

TEST_CASE("Friction holds a box on a gentle slope but not on a steep one")
{
    auto slideAfter = [](real slopeAngle)
    {
        World world;
        world.friction = (real)0.5;
        world.restitution = 0;
        const Vector2 normal(-std::sin(slopeAngle), std::cos(slopeAngle));
        PlaneCollider slope(normal, 0);
        world.colliders.push_back(&slope);
        auto pBox = makeBox(world, normal * (real)0.5, Vector2((real)0.5, (real)0.5), 1, slopeAngle);
        pBox->body.setCanSleep(false);
        const Vector2 start = pBox->body.getPosition();
        run(world, 2);
        return (pBox->body.getPosition() - start).magnitude();
    };

    CHECK(slideAfter((real)0.2) < 0.05f); // tan(0.2) is under the friction of 0.5
    CHECK(slideAfter((real)0.8) > 1.0f);
}

TEST_CASE("A bouncy ball comes back up and an off-centre hit sets a box spinning")
{
    World world;
    world.restitution = (real)0.8;
    world.friction = 0;
    PlaneCollider ground;
    world.colliders.push_back(&ground);
    auto pBall = makeBall(world, Vector2(0, 3), (real)0.5, 1);

    real highestAfterBounce = 0;
    bool bounced = false;
    for (int i = 0; i < 200; i++)
    {
        world.runPhysics(DT);
        if (pBall->body.getLinearVelocity().y > 0)
        {
            bounced = true;
        }
        if (bounced)
        {
            highestAfterBounce = std::max(highestAfterBounce, pBall->body.getPosition().y);
        }
    }
    CHECK(bounced);
    CHECK(highestAfterBounce > 1.5f);

    World spinWorld;
    spinWorld.restitution = 0;
    PlaneCollider floor;
    spinWorld.colliders.push_back(&floor);
    auto pBox = makeBox(spinWorld, Vector2(0, 2), Vector2(1, (real)0.25), 1, (real)0.3);
    real maxSpin = 0;
    for (int i = 0; i < 150; i++)
    {
        spinWorld.runPhysics(DT);
        maxSpin = std::max(maxSpin, std::abs(pBox->body.getAngularVelocity()));
    }
    CHECK(maxSpin > 1.0f);
}

TEST_CASE("A 2D joint holds a swinging body at its pivot")
{
    World world;
    auto pBox = makeBox(world, Vector2(2, 5), Vector2(1, (real)0.1), 1);
    Joint pin;
    pin.set(&pBox->body, Vector2(-1, 0), nullptr, Vector2(1, 5));
    world.joints.push_back(&pin);
    pBox->body.setCanSleep(false);

    real lowest = 5;
    for (int i = 0; i < 300; i++)
    {
        world.runPhysics(DT);
        REQUIRE((pin.getWorldPosition(0) - pin.getWorldPosition(1)).magnitude() < 0.05f);
        lowest = std::min(lowest, pBox->body.getPosition().y);
    }

    // The free end swung down through the bottom of its arc
    CHECK(lowest < 4.2f);
}

TEST_CASE("A 2D chain on World joints stays pinned together, then settles hanging straight down")
{
    World world;
    std::vector<std::unique_ptr<Box>> links;
    std::vector<std::unique_ptr<Joint>> joints;
    const Vector2 anchor(0, 10);
    const Vector2 halfSize((real)0.4, (real)0.08);
    const real reach = halfSize.x + (real)0.1;
    RigidBody* pPrevious = nullptr;
    Vector2 previousPoint = anchor;
    for (int i = 0; i < 4; i++)
    {
        // Laid out sideways, so the chain whips down when it starts
        auto pLink = makeBox(world, anchor + Vector2(reach * (real)(2 * i + 1), 0), halfSize, 1);
        pLink->body.setLinearDamping((real)0.5);
        pLink->body.setAngularDamping((real)0.5);
        auto pJoint = std::make_unique<Joint>();
        pJoint->set(&pLink->body, Vector2(-reach, 0), pPrevious, previousPoint);
        world.joints.push_back(pJoint.get());
        pPrevious = &pLink->body;
        previousPoint = Vector2(reach, 0);
        links.push_back(std::move(pLink));
        joints.push_back(std::move(pJoint));
    }

    // The book's single-contact joint let these drift a quarter of a metre apart
    for (int i = 0; i < 300; i++)
    {
        world.runPhysics(DT);
        for (const auto& pJoint : joints)
        {
            REQUIRE((pJoint->getWorldPosition(0) - pJoint->getWorldPosition(1)).magnitude() < 0.01f);
        }
    }
    run(world, 30);

    const Vector2 bottom = links.back()->body.getPosition();
    CHECK(std::abs(bottom.x) < 0.05f);
    CHECK(bottom.y == doctest::Approx(10 - 7 * reach).epsilon(0.01));
    CHECK(world.getAwakeCount() == 0);
}

TEST_CASE("A 2D spring pulls a body toward a fixed point and the world finds what's under the cursor")
{
    World world;
    auto pBall = makeBall(world, Vector2(3, 0), (real)0.5, 1);
    pBall->body.setAcceleration(Vector2());
    pBall->body.setCanSleep(false);
    RigidBodySpring spring(Vector2(), nullptr, Vector2(0, 0), 20, 1, 4);
    world.forces.add(&pBall->body, &spring);
    run(world, 10);
    CHECK(pBall->body.getPosition().magnitude() == doctest::Approx(1).epsilon(0.02));

    CHECK(world.findColliderAt(pBall->body.getPosition()) == &pBall->collider);
    CHECK(world.findColliderAt(Vector2(10, 10)) == nullptr);
}
