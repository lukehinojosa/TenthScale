#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <cyclone/cyclone.h>

using namespace cyclone;

namespace
{
    constexpr real DT = (real)0.01;

    struct Box
    {
        RigidBody body;
        BoxCollider collider;
    };

    struct Ball
    {
        RigidBody body;
        SphereCollider collider;
    };

    std::unique_ptr<Box> makeBox(World& world, Vector3 position, Vector3 halfSize, real mass)
    {
        auto pBox = std::make_unique<Box>();
        pBox->body.setPosition(position);
        pBox->body.setMass(mass);
        Matrix3 tensor;
        tensor.setBlockInertiaTensor(halfSize, mass);
        pBox->body.setInertiaTensor(tensor);
        pBox->collider.halfSize = halfSize;
        pBox->collider.pBody = &pBox->body;
        world.bodies.push_back(&pBox->body);
        world.colliders.push_back(&pBox->collider);
        return pBox;
    }

    std::unique_ptr<Ball> makeBall(World& world, Vector3 position, real radius, real mass)
    {
        auto pBall = std::make_unique<Ball>();
        pBall->body.setPosition(position);
        pBall->body.setMass(mass);
        const real inertia = (real)0.4 * mass * radius * radius;
        pBall->body.setInertiaTensor(Matrix3(inertia, 0, 0, 0, inertia, 0, 0, 0, inertia));
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

    void runParticles(ParticleWorld& world, real seconds, real dt = DT)
    {
        for (int i = 0; i < (int)(seconds / dt); i++)
        {
            world.runPhysics(dt);
        }
    }

    void placeStatic(Collider& collider)
    {
        collider.pBody = nullptr;
        collider.updateInternals();
    }
}

TEST_CASE("A particle on a cable hangs at the cable's length")
{
    Particle particle(Vector3(2, 0, 0), Vector3(), Vector3(0, -9.81f, 0), Vector3(0.9f, 0.9f, 0.9f));
    ParticleCableConstraint cable;
    cable.particle = &particle;
    cable.anchor = Vector3(0, 0, 0);
    cable.maxLength = 3;

    ParticleWorld world;
    world.particles.push_back(&particle);
    world.contactGenerators.push_back(&cable);
    runParticles(world, 10);

    CHECK((particle.getPosition() - cable.anchor).magnitude() == doctest::Approx(3).epsilon(0.02));
    CHECK(particle.getPosition().y < -2.9f);
}

TEST_CASE("A rod keeps two particles the same distance apart while they tumble")
{
    Particle a(Vector3(0, 5, 0), Vector3(3, 0, 0), Vector3(0, -9.81f, 0), Vector3(1, 1, 1));
    Particle b(Vector3(0, 6, 0), Vector3(-1, 2, 0), Vector3(0, -9.81f, 0), Vector3(1, 1, 1));
    ParticleRod rod;
    rod.particle[0] = &a;
    rod.particle[1] = &b;
    rod.length = 1;

    ParticleWorld world;
    world.particles = { &a, &b };
    world.contactGenerators.push_back(&rod);
    for (int i = 0; i < 100; i++)
    {
        world.runPhysics(DT);
        REQUIRE((a.getPosition() - b.getPosition()).magnitude() == doctest::Approx(1).epsilon(0.001));
    }
}

TEST_CASE("Particles land on the ground and stay on it")
{
    Particle particle(Vector3(0, 3, 0), Vector3(), Vector3(0, -9.81f, 0), Vector3(0.99f, 0.99f, 0.99f));
    std::vector<Particle*> particles = { &particle };
    ParticleGroundContacts ground;
    ground.pParticles = &particles;

    ParticleWorld world;
    world.particles = particles;
    world.contactGenerators.push_back(&ground);
    runParticles(world, 5);
    CHECK(particle.getPosition().y == doctest::Approx(0).epsilon(0.01));
    CHECK(std::abs(particle.getVelocity().y) < 0.2f);
}

TEST_CASE("The anchored spring hangs a particle where gravity balances the spring")
{
    Vector3 anchor(0, 10, 0);
    Particle particle(Vector3(0, 9, 0), Vector3(), Vector3(0, -9.81f, 0), Vector3(0.5f, 0.5f, 0.5f));
    particle.setMass(2);
    ParticleAnchoredSpring spring(&anchor, 1, 40);

    ParticleWorld world;
    world.particles.push_back(&particle);
    world.forces.add(&particle, &spring);
    runParticles(world, 20);
    // Gravity here is an acceleration, so the spring stretches until it holds the particle's full weight
    CHECK(particle.getPosition().y == doctest::Approx(10.0 - 1.0 - 9.81 * 2 / 40).epsilon(0.02));
}

TEST_CASE("The fake stiff spring stays stable at a large time step")
{
    Vector3 anchor(0, 0, 0);
    Particle particle(Vector3(1, 0, 0), Vector3(), Vector3(0, 0, 0), Vector3(1, 1, 1));
    ParticleFakeSpring spring(&anchor, 10000, 10);

    ParticleWorld world;
    world.particles.push_back(&particle);
    world.forces.add(&particle, &spring);
    // A real spring this stiff would blow up at 1/20 s steps
    runParticles(world, 5, (real)0.05);
    CHECK(std::isfinite(particle.getPosition().x));
    CHECK(particle.getPosition().magnitude() < 0.1f);
}

TEST_CASE("A box dropped on the ground settles flat, without sinking, and falls asleep")
{
    World world;
    PlaneCollider ground(Vector3(0, 1, 0), 0);
    world.colliders.push_back(&ground);
    auto pBox = makeBox(world, Vector3(0, 2, 0), Vector3(0.5f, 0.5f, 0.5f), 1);
    pBox->body.setOrientation(Quaternion(0.98f, 0.1f, 0.05f, 0.1f));

    run(world, 6);
    CHECK(pBox->body.getPosition().y == doctest::Approx(0.5).epsilon(0.03));
    CHECK_FALSE(pBox->body.isAwake());

    // Lying flat means its up axis lines up with the world's up axis (or another face does)
    const Vector3 up = pBox->body.getDirectionInWorldSpace(Vector3(0, 1, 0));
    const Vector3 side = pBox->body.getDirectionInWorldSpace(Vector3(1, 0, 0));
    const Vector3 front = pBox->body.getDirectionInWorldSpace(Vector3(0, 0, 1));
    const real alignment = std::max({ std::abs(up.y), std::abs(side.y), std::abs(front.y) });
    CHECK(alignment > 0.99f);
}

TEST_CASE("Box-ground contacts come from every corner touching the ground")
{
    World world;
    PlaneCollider ground(Vector3(0, 1, 0), 0);
    world.colliders.push_back(&ground);
    auto pBox = makeBox(world, Vector3(0, 0.49f, 0), Vector3(0.5f, 0.5f, 0.5f), 1);
    CHECK(world.generateContacts() == 4u);
}

TEST_CASE("An off-centre hit makes a box spin")
{
    World world;
    world.restitution = (real)0.5;
    world.friction = 0;
    auto pBox = makeBox(world, Vector3(0, 0, 0), Vector3(1, 1, 1), 1);
    auto pBall = makeBall(world, Vector3(-3, 0.8f, 0), (real)0.25, 1);
    pBox->body.setAcceleration(Vector3());
    pBall->body.setAcceleration(Vector3());
    pBall->body.setLinearVelocity(Vector3(10, 0, 0));

    run(world, 0.5f);
    // Hit above the centre while moving +x: the box turns about -z
    CHECK(pBox->body.getAngularVelocity().z < -0.5f);
    CHECK(pBox->body.getLinearVelocity().x > 1);
    CHECK(pBall->body.getLinearVelocity().x < 10);
}

TEST_CASE("A head-on hit through the centre doesn't spin anything")
{
    World world;
    world.friction = 0;
    auto pBox = makeBox(world, Vector3(0, 0, 0), Vector3(1, 1, 1), 1);
    auto pBall = makeBall(world, Vector3(-3, 0, 0), (real)0.25, 1);
    pBox->body.setAcceleration(Vector3());
    pBall->body.setAcceleration(Vector3());
    pBall->body.setLinearVelocity(Vector3(10, 0, 0));
    run(world, 0.5f);
    CHECK(pBox->body.getAngularVelocity().magnitude() < 0.01f);
    CHECK(pBox->body.getLinearVelocity().x > 1);
}

TEST_CASE("Friction stops a sliding box; without friction it keeps going")
{
    for (real friction : { (real)0.8, (real)0.0 })
    {
        World world;
        world.friction = friction;
        world.restitution = 0;
        PlaneCollider ground(Vector3(0, 1, 0), 0);
        world.colliders.push_back(&ground);
        auto pBox = makeBox(world, Vector3(0, 0.5f, 0), Vector3(0.5f, 0.5f, 0.5f), 1);
        pBox->body.setLinearDamping(1);
        pBox->body.setAngularDamping(1);
        pBox->body.setCanSleep(false);
        pBox->body.setLinearVelocity(Vector3(5, 0, 0));
        run(world, 2);
        if (friction > 0)
        {
            CHECK(std::abs(pBox->body.getLinearVelocity().x) < 0.1f);
            // It stops about as far along as friction predicts
            CHECK(pBox->body.getPosition().x == doctest::Approx(25.0 / (2 * 0.8 * 9.81)).epsilon(0.25));
        }
        else
        {
            CHECK(pBox->body.getLinearVelocity().x == doctest::Approx(5).epsilon(0.02));
        }
    }
}

TEST_CASE("BoxBox finds face contacts for a box resting on another and nothing for separated boxes")
{
    RigidBody bottomBody;
    RigidBody topBody;
    BoxCollider bottom(Vector3(1, 0.5f, 1), &bottomBody);
    BoxCollider top(Vector3(0.5f, 0.5f, 0.5f), &topBody);
    topBody.setPosition(Vector3(0.1f, 0.98f, 0));
    bottom.updateInternals();
    top.updateInternals();

    Contact contacts[16];
    CollisionData data;
    data.contactArray = contacts;
    data.reset(16);
    const int added = CollisionTests::BoxBox(bottom, top, &data);
    CHECK(added == 4);
    for (int i = 0; i < added; i++)
    {
        // The normal points from the second box toward the first: down
        CHECK(contacts[i].normal.y == doctest::Approx(-1).epsilon(0.001));
        CHECK(contacts[i].penetration == doctest::Approx(0.02).epsilon(0.01));
    }

    topBody.setPosition(Vector3(0, 1.2f, 0));
    top.updateInternals();
    data.reset(16);
    CHECK(CollisionTests::BoxBox(bottom, top, &data) == 0);
}

TEST_CASE("BoxBox gives an offset box a contact at every corner of the overlap, not just the box corners")
{
    RigidBody bottomBody;
    RigidBody topBody;
    BoxCollider bottom(Vector3(0.5f, 0.5f, 0.5f), &bottomBody);
    BoxCollider top(Vector3(0.5f, 0.5f, 0.5f), &topBody);
    topBody.setPosition(Vector3(0.3f, 0.99f, 0.2f));
    bottom.updateInternals();
    top.updateInternals();

    Contact contacts[16];
    CollisionData data;
    data.contactArray = contacts;
    data.reset(16);
    const int added = CollisionTests::BoxBox(bottom, top, &data);

    // Two corners of the overlap are box corners and two are where the outlines cross; without those two the
    // top box would rock on a diagonal
    REQUIRE(added == 4);
    for (real x : { (real)-0.2, (real)0.5 })
    {
        for (real z : { (real)-0.3, (real)0.5 })
        {
            bool found = false;
            for (int i = 0; i < added; i++)
            {
                found = found || (std::abs(contacts[i].point.x - x) < 0.01f && std::abs(contacts[i].point.z - z) < 0.01f);
            }
            CHECK(found);
        }
    }
}

TEST_CASE("BoxBox finds a single edge contact for crossed edges")
{
    RigidBody aBody;
    RigidBody bBody;
    BoxCollider a(Vector3(0.5f, 0.5f, 0.5f), &aBody);
    BoxCollider b(Vector3(0.5f, 0.5f, 0.5f), &bBody);
    // Both turned 45 degrees about different axes, so an edge of each meets in the middle
    const real c = std::cos((real)R_PI / 8);
    const real s = std::sin((real)R_PI / 8);
    aBody.setOrientation(Quaternion(c, 0, 0, s));
    bBody.setOrientation(Quaternion(c, s, 0, 0));
    bBody.setPosition(Vector3(0, 1.38f, 0));
    a.updateInternals();
    b.updateInternals();

    Contact contacts[16];
    CollisionData data;
    data.contactArray = contacts;
    data.reset(16);
    REQUIRE(CollisionTests::BoxBox(a, b, &data) == 1);
    CHECK(std::abs(contacts[0].normal.y) == doctest::Approx(1).epsilon(0.01));
    CHECK(contacts[0].point.y == doctest::Approx(0.69).epsilon(0.05));
    CHECK(contacts[0].penetration > 0);
}

TEST_CASE("BoxSphere pushes a sphere out of the nearest face")
{
    RigidBody boxBody;
    RigidBody sphereBody;
    BoxCollider box(Vector3(1, 1, 1), &boxBody);
    SphereCollider sphere(0.5f, &sphereBody);
    sphereBody.setPosition(Vector3(1.3f, 0, 0));
    box.updateInternals();
    sphere.updateInternals();

    Contact contacts[4];
    CollisionData data;
    data.contactArray = contacts;
    data.reset(4);
    REQUIRE(CollisionTests::BoxSphere(box, sphere, &data) == 1);
    CHECK(contacts[0].normal.x == doctest::Approx(-1)); // From the sphere toward the box
    CHECK(contacts[0].penetration == doctest::Approx(0.2f));

    // Centre inside the box still gives a sensible way out
    sphereBody.setPosition(Vector3(0.9f, 0, 0));
    sphere.updateInternals();
    data.reset(4);
    REQUIRE(CollisionTests::BoxSphere(box, sphere, &data) == 1);
    CHECK(contacts[0].normal.x == doctest::Approx(-1));
    CHECK(contacts[0].penetration == doctest::Approx(0.6f));
}

TEST_CASE("SphereTruePlane pushes a sphere back to the side its centre is on")
{
    SphereCollider sphere(1);
    PlaneCollider plane(Vector3(0, 1, 0), 0);
    sphere.offset = Matrix4();
    sphere.offset.data[7] = -0.5f;
    placeStatic(sphere);

    Contact contacts[2];
    CollisionData data;
    data.contactArray = contacts;
    data.reset(2);
    REQUIRE(CollisionTests::SphereTruePlane(sphere, plane, &data) == 1);
    CHECK(contacts[0].normal.y == doctest::Approx(-1));
    CHECK(contacts[0].penetration == doctest::Approx(0.5f));
}

TEST_CASE("A stack of boxes stays standing and goes to sleep")
{
    World world;
    world.restitution = 0;
    PlaneCollider ground(Vector3(0, 1, 0), 0);
    world.colliders.push_back(&ground);
    std::vector<std::unique_ptr<Box>> boxes;
    for (int i = 0; i < 4; i++)
    {
        boxes.push_back(makeBox(world, Vector3(0, 0.5f + i * 1.0f, 0), Vector3(0.5f, 0.5f, 0.5f), 1));
    }
    run(world, 8);
    for (int i = 0; i < 4; i++)
    {
        const Vector3 p = boxes[i]->body.getPosition();
        CHECK(p.y == doctest::Approx(0.5 + i).epsilon(0.03));
        CHECK(std::abs(p.x) < 0.05f);
        CHECK(std::abs(p.z) < 0.05f);
    }
    CHECK(world.getAwakeCount() == 0u);
}

TEST_CASE("A sleeping body wakes when pushed or hit")
{
    World world;
    PlaneCollider ground(Vector3(0, 1, 0), 0);
    world.colliders.push_back(&ground);
    auto pBox = makeBox(world, Vector3(0, 0.5f, 0), Vector3(0.5f, 0.5f, 0.5f), 1);
    run(world, 4);
    REQUIRE_FALSE(pBox->body.isAwake());

    // The push has to survive the step, not just flip the flag
    pBox->body.addForce(Vector3(300, 0, 0));
    CHECK(pBox->body.isAwake());
    world.runPhysics(DT);
    CHECK(pBox->body.isAwake());
    CHECK(pBox->body.getLinearVelocity().x > 1);
    run(world, 6);
    REQUIRE_FALSE(pBox->body.isAwake());
    CHECK(pBox->body.getPosition().x > 0.2f);

    // A ball landing on it wakes it through the contact
    auto pBall = makeBall(world, Vector3(0.3f, 3, 0), 0.3f, 1);
    bool woke = false;
    for (int i = 0; i < 200 && !woke; i++)
    {
        world.runPhysics(DT);
        woke = pBox->body.isAwake();
    }
    CHECK(woke);
}

TEST_CASE("A joint holds a box hanging from a fixed point")
{
    World world;
    auto pBox = makeBox(world, Vector3(1, 5, 0), Vector3(0.25f, 0.25f, 0.25f), 1);
    pBox->body.setCanSleep(false);
    Joint joint;
    joint.set(&pBox->body, Vector3(0, 0.25f, 0), nullptr, Vector3(0, 6, 0), 0);
    world.contactGenerators.push_back(&joint);

    run(world, 6);
    CHECK((joint.getWorldPosition(0) - joint.getWorldPosition(1)).magnitude() < 0.05f);
    // Swung down below the pivot
    CHECK(pBox->body.getPosition().y < 6);
}

TEST_CASE("An explosion throws nearby bodies away and up")
{
    World world;
    PlaneCollider ground(Vector3(0, 1, 0), 0);
    world.colliders.push_back(&ground);
    auto pBox = makeBox(world, Vector3(2, 0.5f, 0), Vector3(0.5f, 0.5f, 0.5f), 1);
    run(world, 3);

    Explosion explosion;
    explosion.shockwaveSpeed = 10;
    explosion.peakConcussionForce = 2000;
    world.forces.add(&pBox->body, &explosion);
    explosion.detonate(Vector3(0, 0, 0));
    real highest = 0;
    for (int i = 0; i < 100; i++)
    {
        explosion.advance(DT);
        world.runPhysics(DT);
        highest = std::max(highest, pBox->body.getPosition().y);
    }
    CHECK(pBox->body.getPosition().x > 2.5f);
    CHECK(highest > 0.6f);
}

TEST_CASE("Aero turns air speed into force through its tensor")
{
    RigidBody body;
    body.setAcceleration(Vector3());
    body.setLinearVelocity(Vector3(10, 0, 0));
    // Lift: speed along x turns into force along y
    Matrix3 tensor(0, 0, 0, 1, 0, 0, 0, 0, 0);
    Aero wing(tensor, Vector3());
    wing.updateForce(&body, DT);
    body.integrate(DT);
    CHECK(body.getLinearVelocity().y == doctest::Approx(10 * DT).epsilon(0.01));
}

TEST_CASE("AeroControl blends its tensor with the control setting")
{
    Matrix3 base(1, 0, 0, 0, 1, 0, 0, 0, 1);
    Matrix3 low;
    Matrix3 high(2, 0, 0, 0, 2, 0, 0, 0, 2);
    AeroControl control(base, low, high, Vector3());
    control.setControl(0.5f);
    CHECK(control.getTensor().data[0] == doctest::Approx(1.5f));
    control.setControl(-2);
    CHECK(control.getControl() == doctest::Approx(-1));
    CHECK(control.getTensor().data[0] == doctest::Approx(0));
}

TEST_CASE("Buoyancy floats a body at the water line")
{
    World world;
    auto pBox = makeBox(world, Vector3(0, 3, 0), Vector3(0.5f, 0.5f, 0.5f), 500);
    pBox->body.setCanSleep(false);
    pBox->body.setLinearDamping(0.5f);
    // Half a cubic metre of water weighs 500 kg, so it floats half under
    RigidBodyBuoyancy buoyancy(Vector3(), 0.5f, 1, 0);
    world.forces.add(&pBox->body, &buoyancy);
    run(world, 20);
    CHECK(pBox->body.getPosition().y == doctest::Approx(0).epsilon(0.05));
}

TEST_CASE("A dropped ball bounces lower each time and comes to rest")
{
    World world;
    world.restitution = 0.5f;
    world.friction = 0;
    PlaneCollider ground(Vector3(0, 1, 0), 0);
    world.colliders.push_back(&ground);
    auto pBall = makeBall(world, Vector3(0, 2.5f, 0), 0.5f, 1);
    pBall->body.setLinearDamping(1);

    real peakAfterFirstBounce = 0;
    bool bounced = false;
    for (int i = 0; i < 300; i++)
    {
        world.runPhysics(DT);
        if (pBall->body.getLinearVelocity().y > 0)
        {
            bounced = true;
        }
        if (bounced)
        {
            peakAfterFirstBounce = std::max(peakAfterFirstBounce, pBall->body.getPosition().y);
        }
    }
    // Dropped 2 m above contact, restitution 0.5: back up about 0.5 m
    CHECK(peakAfterFirstBounce - 0.5f == doctest::Approx(0.5).epsilon(0.2));
    run(world, 5);
    CHECK(pBall->body.getPosition().y == doctest::Approx(0.5).epsilon(0.02));
}

TEST_CASE("A resting stack doesn't shake while it settles")
{
    World world;
    PlaneCollider ground(Vector3(0, 1, 0), 0);
    world.colliders.push_back(&ground);
    std::vector<std::unique_ptr<Box>> boxes;
    for (int i = 0; i < 4; i++)
    {
        boxes.push_back(makeBox(world, Vector3(0, 0.5f + i * 1.0f, 0), Vector3(0.5f, 0.5f, 0.5f), 1));
    }

    // The top box moves the most if the stack shakes; measure its step-to-step movement while awake
    const real step = (real)0.02;
    real lastY = boxes[3]->body.getPosition().y;
    real largestMove = 0;
    for (int i = 0; i < 100; i++)
    {
        world.runPhysics(step);
        const real y = boxes[3]->body.getPosition().y;
        if (i >= 5 && world.getAwakeCount() > 0)
        {
            largestMove = std::max(largestMove, real_abs(y - lastY));
            CHECK(world.getContactCount() == 16u); // four corners per face, none doubled
        }
        lastY = y;
    }
    CHECK(largestMove < 0.001f);
    CHECK(boxes[3]->body.getPosition().y == doctest::Approx(3.5).epsilon(0.002));
    CHECK(world.getAwakeCount() == 0u);
}

TEST_CASE("A box landing on a sleeping stack doesn't push the boxes into each other")
{
    World world;
    PlaneCollider ground(Vector3(0, 1, 0), 0);
    world.colliders.push_back(&ground);
    std::vector<std::unique_ptr<Box>> boxes;
    for (int i = 0; i < 4; i++)
    {
        boxes.push_back(makeBox(world, Vector3(0, 0.5f + i * 1.0f, 0), Vector3(0.5f, 0.5f, 0.5f), 1));
    }
    run(world, 3);
    REQUIRE(world.getAwakeCount() == 0u);

    boxes.push_back(makeBox(world, Vector3(0.1f, 6.0f, -0.1f), Vector3(0.5f, 0.5f, 0.5f), 1));
    real deepest = 0;
    for (int i = 0; i < 150; i++)
    {
        world.runPhysics((real)0.02);
        // How far boxes still overlap once the step is done, from a fresh contact pass
        const unsigned count = world.generateContacts();
        for (unsigned c = 0; c < count; c++)
        {
            deepest = std::max(deepest, world.getContacts()[c].penetration);
        }
    }
    CHECK(deepest < 0.02f);
    for (int i = 0; i < 5; i++)
    {
        CHECK(boxes[i]->body.getPosition().y == doctest::Approx(0.5 + i).epsilon(0.01));
    }
}

TEST_CASE("A box landing off-centre on a stack rests flat instead of rocking and creeping")
{
    World world;
    PlaneCollider ground(Vector3(0, 1, 0), 0);
    world.colliders.push_back(&ground);
    std::vector<std::unique_ptr<Box>> boxes;
    for (int i = 0; i < 4; i++)
    {
        boxes.push_back(makeBox(world, Vector3(0, 0.5f + i * 1.0f, 0), Vector3(0.5f, 0.5f, 0.5f), 1));
    }
    run(world, 3);
    boxes.push_back(makeBox(world, Vector3(0.1f, 6.0f, -0.1f), Vector3(0.5f, 0.5f, 0.5f), 1));
    const RigidBody& landed = boxes[4]->body;

    Vector3 settled;
    real fastestSpin = 0;
    for (int i = 0; i < 300; i++)
    {
        world.runPhysics((real)0.02);
        if (i == 50)
        {
            settled = landed.getPosition();
        }
        if (i <= 50 || !landed.isAwake())
        {
            continue;
        }
        // It should keep resting on the whole overlap, never on one or two corners
        int touching = 0;
        for (unsigned c = 0; c < world.getContactCount(); c++)
        {
            const Contact& contact = world.getContacts()[c];
            touching += (contact.body[0] == &landed || contact.body[1] == &landed) ? 1 : 0;
        }
        CHECK(touching >= 4);
        fastestSpin = std::max(fastestSpin, landed.getAngularVelocity().magnitude());
    }
    CHECK(fastestSpin < 0.05f);
    CHECK((landed.getPosition() - settled).magnitude() < 0.03f);
    CHECK(world.getAwakeCount() == 0u);
}
