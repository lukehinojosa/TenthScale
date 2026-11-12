#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include <cyclone/cyclone.h>

using namespace cyclone;

namespace
{
    constexpr real DT = (real)0.01;

    std::vector<BroadPhaseItem> randomItems(int count, unsigned seed, real minRadius, real maxRadius, real spread)
    {
        std::mt19937 random(seed);
        std::uniform_real_distribution<float> position(-spread, spread);
        std::uniform_real_distribution<float> size(minRadius, maxRadius);
        std::vector<BroadPhaseItem> items;
        for (int i = 0; i < count; i++)
        {
            items.push_back(BroadPhaseItem{ i * 3 + 7, Vector3(position(random), position(random), position(random)), size(random) });
        }
        return items;
    }

    std::set<std::pair<int, int>> bruteForcePairs(const std::vector<BroadPhaseItem>& items)
    {
        std::set<std::pair<int, int>> pairs;
        for (std::size_t i = 0; i < items.size(); i++)
        {
            for (std::size_t j = i + 1; j < items.size(); j++)
            {
                const real reach = items[i].radius + items[j].radius;
                if ((items[i].center - items[j].center).squareMagnitude() <= reach * reach)
                {
                    pairs.insert({ std::min(items[i].id, items[j].id), std::max(items[i].id, items[j].id) });
                }
            }
        }
        return pairs;
    }

    std::set<std::pair<int, int>> toSet(const std::vector<PotentialContact>& contacts)
    {
        std::set<std::pair<int, int>> pairs;
        for (const PotentialContact& c : contacts)
        {
            CHECK(c.a < c.b);
            pairs.insert({ c.a, c.b });
        }
        CHECK(pairs.size() == contacts.size()); // No pair reported twice
        return pairs;
    }

    std::set<int> bruteForceQuery(const std::vector<BroadPhaseItem>& items, const BoundingSphere& volume)
    {
        std::set<int> ids;
        for (const BroadPhaseItem& item : items)
        {
            const real reach = item.radius + volume.radius;
            if ((item.center - volume.center).squareMagnitude() <= reach * reach)
            {
                ids.insert(item.id);
            }
        }
        return ids;
    }

    struct Body
    {
        RigidBody body;
        std::unique_ptr<Collider> pCollider;
    };

    Body* addBox(World& world, std::vector<std::unique_ptr<Body>>& bodies, Vector3 position, Vector3 half, real mass)
    {
        auto pBody = std::make_unique<Body>();
        pBody->body.setPosition(position);
        pBody->body.setMass(mass);
        Matrix3 tensor;
        tensor.setBlockInertiaTensor(half, mass);
        pBody->body.setInertiaTensor(tensor);
        pBody->pCollider = std::make_unique<BoxCollider>(half, &pBody->body);
        world.bodies.push_back(&pBody->body);
        world.colliders.push_back(pBody->pCollider.get());
        bodies.push_back(std::move(pBody));
        return bodies.back().get();
    }

    Body* addConvex(World& world, std::vector<std::unique_ptr<Body>>& bodies, const ConvexCollider& shape, Vector3 position, real mass)
    {
        auto pBody = std::make_unique<Body>();
        pBody->body.setPosition(position);
        pBody->body.setMass(mass);
        pBody->body.setInertiaTensor(shape.computeInertiaTensor(mass));
        auto pCollider = std::make_unique<ConvexCollider>(shape);
        pCollider->pBody = &pBody->body;
        pBody->pCollider = std::move(pCollider);
        world.bodies.push_back(&pBody->body);
        world.colliders.push_back(pBody->pCollider.get());
        bodies.push_back(std::move(pBody));
        return bodies.back().get();
    }

    Body* addBall(World& world, std::vector<std::unique_ptr<Body>>& bodies, Vector3 position, real radius, real mass)
    {
        auto pBody = std::make_unique<Body>();
        pBody->body.setPosition(position);
        pBody->body.setMass(mass);
        const real inertia = (real)0.4 * mass * radius * radius;
        pBody->body.setInertiaTensor(Matrix3(inertia, 0, 0, 0, inertia, 0, 0, 0, inertia));
        pBody->pCollider = std::make_unique<SphereCollider>(radius, &pBody->body);
        world.bodies.push_back(&pBody->body);
        world.colliders.push_back(pBody->pCollider.get());
        bodies.push_back(std::move(pBody));
        return bodies.back().get();
    }

    void run(World& world, real seconds)
    {
        for (int i = 0; i < (int)(seconds / DT); i++)
        {
            world.runPhysics(DT);
        }
    }

    real upAlignment(const RigidBody& body)
    {
        const Vector3 x = body.getDirectionInWorldSpace(Vector3(1, 0, 0));
        const Vector3 y = body.getDirectionInWorldSpace(Vector3(0, 1, 0));
        const Vector3 z = body.getDirectionInWorldSpace(Vector3(0, 0, 1));
        return std::max({ std::abs(x.y), std::abs(y.y), std::abs(z.y) });
    }
}

TEST_CASE("Every broad phase finds exactly the overlapping pairs")
{
    // Mostly small spheres with a few big ones, which is what multiresolution maps are for
    std::vector<BroadPhaseItem> items = randomItems(300, 42, 0.1f, 0.6f, 7);
    std::vector<BroadPhaseItem> big = randomItems(8, 7, 2, 6, 12);
    for (BroadPhaseItem& item : big)
    {
        item.id += 100000;
        items.push_back(item);
    }
    const auto expected = bruteForcePairs(items);
    REQUIRE(expected.size() > 20);

    BoundingVolumeHierarchy hierarchy;
    for (const BroadPhaseItem& item : items)
    {
        hierarchy.insert(item.id, BoundingSphere(item.center, item.radius));
    }
    std::vector<PotentialContact> found;
    hierarchy.getPotentialContacts(found);
    CHECK(toSet(found) == expected);
    CHECK(hierarchy.getDepth() < 40);

    BspTree bsp;
    bsp.build(items);
    found.clear();
    bsp.getPotentialContacts(found);
    CHECK(toSet(found) == expected);
    CHECK(bsp.getLeafCount() > 10);

    MultiResolutionMap map(0.25f, 4); // The biggest spheres don't fit any grid, which has to work too
    for (const BroadPhaseItem& item : items)
    {
        map.insert(item);
    }
    found.clear();
    map.getPotentialContacts(found);
    CHECK(toSet(found) == expected);
}

TEST_CASE("Broad phase queries return everything a sphere touches")
{
    std::vector<BroadPhaseItem> items = randomItems(200, 3, 0.1f, 1.5f, 10);
    BoundingVolumeHierarchy hierarchy;
    MultiResolutionMap map(0.5f, 6);
    BspTree bsp;
    for (const BroadPhaseItem& item : items)
    {
        hierarchy.insert(item.id, BoundingSphere(item.center, item.radius));
        map.insert(item);
    }
    bsp.build(items);

    for (const BoundingSphere& volume : { BoundingSphere(Vector3(0, 0, 0), 2), BoundingSphere(Vector3(5, -3, 2), 4), BoundingSphere(Vector3(-8, 8, 8), 0.5f) })
    {
        const auto expected = bruteForceQuery(items, volume);
        std::vector<int> a;
        std::vector<int> b;
        std::vector<int> c;
        hierarchy.query(volume, a);
        map.query(volume, b);
        bsp.query(volume, c);
        CHECK(std::set<int>(a.begin(), a.end()) == expected);
        CHECK(std::set<int>(b.begin(), b.end()) == expected);
        CHECK(std::set<int>(c.begin(), c.end()) == expected);
    }
}

TEST_CASE("The BVH keeps working as objects are removed and moved")
{
    std::vector<BroadPhaseItem> items = randomItems(100, 9, 0.2f, 0.8f, 6);
    BoundingVolumeHierarchy hierarchy;
    for (const BroadPhaseItem& item : items)
    {
        hierarchy.insert(item.id, BoundingSphere(item.center, item.radius));
    }
    // Remove every third, move every fifth
    std::vector<BroadPhaseItem> remaining;
    for (std::size_t i = 0; i < items.size(); i++)
    {
        if (i % 3 == 0)
        {
            CHECK(hierarchy.remove(items[i].id));
            continue;
        }
        if (i % 5 == 0)
        {
            items[i].center += Vector3(1.5f, 0, -1);
            hierarchy.update(items[i].id, BoundingSphere(items[i].center, items[i].radius));
        }
        remaining.push_back(items[i]);
    }
    CHECK_FALSE(hierarchy.remove(-5));
    CHECK(hierarchy.getLeafCount() == (int)remaining.size());
    std::vector<PotentialContact> found;
    hierarchy.getPotentialContacts(found);
    CHECK(toSet(found) == bruteForcePairs(remaining));
}

TEST_CASE("BSP trees use hand-placed planes first")
{
    // Two rooms either side of a wall through the origin
    std::vector<BroadPhaseItem> items;
    for (int i = 0; i < 20; i++)
    {
        items.push_back(BroadPhaseItem{ i, Vector3(i < 10 ? -5.0f + (i % 3) : 5.0f - (i % 3), (real)(i % 4), 0), 0.6f });
    }
    BspTree bsp;
    bsp.build(items, { BspPlane{ Vector3(1, 0, 0), 0 } }, 4);
    std::vector<PotentialContact> found;
    bsp.getPotentialContacts(found);
    CHECK(toSet(found) == bruteForcePairs(items));
    for (const PotentialContact& pair : found)
    {
        // Nothing reaches through the wall
        CHECK((pair.a < 10) == (pair.b < 10));
    }
}

TEST_CASE("Convex shapes have the right mass properties")
{
    const ConvexCollider box = ConvexCollider::box(Vector3(1, 2, 3));
    CHECK(box.getVolume() == doctest::Approx(48));
    const Matrix3 inertia = box.computeInertiaTensor(6);
    CHECK(inertia.data[0] == doctest::Approx(6.0 / 3 * (4 + 9)));
    CHECK(inertia.data[4] == doctest::Approx(6.0 / 3 * (1 + 9)));
    CHECK(inertia.data[8] == doctest::Approx(6.0 / 3 * (1 + 4)));
    CHECK(std::abs(inertia.data[1]) < 1e-4f);

    // Every helper shape is centred on its centre of mass
    for (const ConvexCollider& shape : { ConvexCollider::wedge(Vector3(1, 0.5f, 0.5f)), ConvexCollider::tetrahedron(1), ConvexCollider::octahedron(1) })
    {
        CHECK(shape.getCentroid().magnitude() < 1e-4f);
        CHECK(shape.getVolume() > 0);
        CHECK(shape.getFaces().size() >= 4);
    }
    CHECK(ConvexCollider::tetrahedron(1).getVolume() == doctest::Approx(1.0 / 3));
    CHECK(ConvexCollider::octahedron(1).getVolume() == doctest::Approx(4.0 / 3));
    CHECK(ConvexCollider::wedge(Vector3(1, 0.5f, 0.5f)).getVolume() == doctest::Approx(0.5 * 2 * 1 * 1));
}

TEST_CASE("Convex shapes dropped on the ground settle and sleep")
{
    for (int shapeIndex = 0; shapeIndex < 4; shapeIndex++)
    {
        World world;
        PlaneCollider ground(Vector3(0, 1, 0), 0);
        world.colliders.push_back(&ground);
        std::vector<std::unique_ptr<Body>> bodies;
        const ConvexCollider shapes[4] = { ConvexCollider::box(Vector3(0.5f, 0.5f, 0.5f)), ConvexCollider::wedge(Vector3(0.6f, 0.4f, 0.5f)),
            ConvexCollider::tetrahedron(1.2f), ConvexCollider::octahedron(0.6f) };
        Body* pBody = addConvex(world, bodies, shapes[shapeIndex], Vector3(0, 2, 0), 1);
        pBody->body.setOrientation(Quaternion(0.95f, 0.2f, 0.1f, 0.15f));
        run(world, 10);
        CAPTURE(shapeIndex);
        CHECK_FALSE(pBody->body.isAwake());
        CHECK(pBody->body.getPosition().y > 0.1f);
        CHECK(pBody->body.getPosition().y < 1);
        // Resting on a face: its lowest corners all touch the ground
        pBody->pCollider->updateInternals();
        const auto& convex = static_cast<const ConvexCollider&>(*pBody->pCollider);
        int touching = 0;
        for (const Vector3& v : convex.getVertices())
        {
            if (convex.transform.transform(v).y < 0.05f)
            {
                touching++;
            }
        }
        CHECK(touching >= 3);
    }
}

TEST_CASE("A convex box on a box gives face contacts; apart gives none")
{
    RigidBody bottomBody;
    RigidBody topBody;
    BoxCollider bottom(Vector3(1, 0.5f, 1), &bottomBody);
    ConvexCollider top = ConvexCollider::box(Vector3(0.5f, 0.5f, 0.5f), &topBody);
    topBody.setPosition(Vector3(0.1f, 0.98f, 0));
    bottom.updateInternals();
    top.updateInternals();

    Contact contacts[16];
    CollisionData data;
    data.contactArray = contacts;
    data.reset(16);
    REQUIRE(CollisionTests::Generate(top, bottom, &data) == 4);
    for (int i = 0; i < 4; i++)
    {
        CHECK(std::abs(contacts[i].normal.y) == doctest::Approx(1).epsilon(0.001));
        CHECK(contacts[i].penetration == doctest::Approx(0.02).epsilon(0.05));
    }

    topBody.setPosition(Vector3(0, 1.3f, 0));
    top.updateInternals();
    data.reset(16);
    CHECK(CollisionTests::Generate(top, bottom, &data) == 0);
}

TEST_CASE("Convex-convex finds an edge contact for crossed edges")
{
    RigidBody aBody;
    RigidBody bBody;
    ConvexCollider a = ConvexCollider::box(Vector3(0.5f, 0.5f, 0.5f), &aBody);
    ConvexCollider b = ConvexCollider::box(Vector3(0.5f, 0.5f, 0.5f), &bBody);
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
    REQUIRE(CollisionTests::ConvexConvex(a, b, &data) == 1);
    CHECK(std::abs(contacts[0].normal.y) == doctest::Approx(1).epsilon(0.01));
    CHECK(contacts[0].point.y == doctest::Approx(0.69).epsilon(0.05));
    CHECK(std::abs(contacts[0].point.x) < 0.05f);
    CHECK(std::abs(contacts[0].point.z) < 0.05f);
}

TEST_CASE("Convex-sphere contacts near a face, near an edge, and from inside")
{
    RigidBody hullBody;
    RigidBody ballBody;
    ConvexCollider hull = ConvexCollider::box(Vector3(1, 1, 1), &hullBody);
    SphereCollider ball(0.5f, &ballBody);
    Contact contacts[4];
    CollisionData data;
    data.contactArray = contacts;

    ballBody.setPosition(Vector3(1.3f, 0, 0));
    hull.updateInternals();
    ball.updateInternals();
    data.reset(4);
    REQUIRE(CollisionTests::ConvexSphere(hull, ball, &data) == 1);
    CHECK(contacts[0].normal.x == doctest::Approx(-1));
    CHECK(contacts[0].penetration == doctest::Approx(0.2f));

    // Off the corner edge: distance to the edge, not to either face plane
    ballBody.setPosition(Vector3(1.3f, 1.3f, 0));
    ball.updateInternals();
    data.reset(4);
    REQUIRE(CollisionTests::ConvexSphere(hull, ball, &data) == 1);
    CHECK(contacts[0].penetration == doctest::Approx(0.5 - std::sqrt(0.18)).epsilon(0.01));
    ballBody.setPosition(Vector3(1.4f, 1.4f, 0));
    ball.updateInternals();
    data.reset(4);
    CHECK(CollisionTests::ConvexSphere(hull, ball, &data) == 0);

    ballBody.setPosition(Vector3(0, 0.8f, 0));
    ball.updateInternals();
    data.reset(4);
    REQUIRE(CollisionTests::ConvexSphere(hull, ball, &data) == 1);
    CHECK(contacts[0].normal.y == doctest::Approx(-1));
    CHECK(contacts[0].penetration == doctest::Approx(0.7f));
}

TEST_CASE("Coherence skips the full test for pairs that stay apart, without changing the result")
{
    auto build = [](World& world, std::vector<std::unique_ptr<Body>>& bodies)
    {
        PlaneCollider* pGround = new PlaneCollider(Vector3(0, 1, 0), 0);
        world.colliders.push_back(pGround);
        for (int i = 0; i < 6; i++)
        {
            addBox(world, bodies, Vector3((real)(i % 3) * 1.2f, 0.5f + (real)(i / 3) * 1.6f, 0), Vector3(0.5f, 0.5f, 0.5f), 1);
        }
        addConvex(world, bodies, ConvexCollider::wedge(Vector3(0.5f, 0.3f, 0.4f)), Vector3(5, 0.5f, 0), 1);
        return pGround;
    };
    World with;
    World without;
    without.useCoherence = false;
    with.broadPhase = World::BroadPhase::ALL_PAIRS;
    without.broadPhase = World::BroadPhase::ALL_PAIRS;
    std::vector<std::unique_ptr<Body>> a;
    std::vector<std::unique_ptr<Body>> b;
    std::unique_ptr<PlaneCollider> groundA(build(with, a));
    std::unique_ptr<PlaneCollider> groundB(build(without, b));

    unsigned hits = 0;
    for (int i = 0; i < 60; i++)
    {
        with.runPhysics(DT);
        without.runPhysics(DT);
        hits += with.getCoherenceHits();
    }
    CHECK(hits > 0);
    CHECK(without.getCoherenceHits() == 0u);
    for (std::size_t i = 0; i < a.size(); i++)
    {
        CHECK((a[i]->body.getPosition() - b[i]->body.getPosition()).magnitude() < 1e-4f);
    }
}

TEST_CASE("Every broad phase gives the World the same contacts")
{
    const World::BroadPhase kinds[4] = { World::BroadPhase::ALL_PAIRS, World::BroadPhase::BOUNDING_VOLUME_HIERARCHY,
        World::BroadPhase::MULTI_RESOLUTION_MAP, World::BroadPhase::BSP_TREE };
    unsigned counts[4] = {};
    for (int k = 0; k < 4; k++)
    {
        World world;
        world.broadPhase = kinds[k];
        PlaneCollider ground(Vector3(0, 1, 0), 0);
        world.colliders.push_back(&ground);
        std::vector<std::unique_ptr<Body>> bodies;
        for (int i = 0; i < 30; i++)
        {
            addBox(world, bodies, Vector3((real)(i % 5) * 0.9f, 0.45f + (real)(i / 5) * 0.85f, (real)(i % 2) * 0.3f), Vector3(0.45f, 0.45f, 0.45f), 1);
        }
        counts[k] = world.generateContacts();
    }
    CHECK(counts[0] > 30);
    CHECK(counts[1] == counts[0]);
    CHECK(counts[2] == counts[0]);
    CHECK(counts[3] == counts[0]);
}

TEST_CASE("Fracturing a box keeps its mass and its motion")
{
    RigidBody body;
    body.setMass(8);
    body.setPosition(Vector3(1, 2, 3));
    body.setLinearVelocity(Vector3(1, 0, 0));
    body.setAngularVelocity(Vector3(0, 2, 0));

    const Vector3 half(1, 1, 1);
    std::vector<BoxFragment> pieces = fractureBox(body, half, Vector3(0.2f, -0.1f, 0.3f));
    REQUIRE(pieces.size() == 8u);
    real mass = 0;
    real volume = 0;
    for (const BoxFragment& piece : pieces)
    {
        mass += piece.mass;
        volume += 8 * piece.halfSize.x * piece.halfSize.y * piece.halfSize.z;
        // Velocity is what the spinning original had at that spot
        const Vector3 expected = Vector3(1, 0, 0) + Vector3(0, 2, 0) % (piece.position - body.getPosition());
        CHECK((piece.linearVelocity - expected).magnitude() < 1e-4f);
    }
    CHECK(mass == doctest::Approx(8));
    CHECK(volume == doctest::Approx(8));

    // Hit at the middle of a face: no sliver along that axis, so four pieces
    CHECK(fractureBox(body, half, Vector3(1, 0, 0)).size() == 4u);
    // A burst pushes pieces away from the impact
    std::vector<BoxFragment> burst = fractureBox(body, half, Vector3(0, 0, 0), 0.05f, 3);
    for (const BoxFragment& piece : burst)
    {
        const Vector3 base = Vector3(1, 0, 0) + Vector3(0, 2, 0) % (piece.position - body.getPosition());
        CHECK((piece.linearVelocity - base) * (piece.position - body.getPosition()) > 0);
    }
}

TEST_CASE("The simultaneous resolver stacks, settles, and sleeps")
{
    World world;
    world.resolverType = World::Resolver::SIMULTANEOUS;
    world.restitution = 0;
    PlaneCollider ground(Vector3(0, 1, 0), 0);
    world.colliders.push_back(&ground);
    std::vector<std::unique_ptr<Body>> bodies;
    for (int i = 0; i < 4; i++)
    {
        addBox(world, bodies, Vector3(0, 0.5f + i * 1.0f, 0), Vector3(0.5f, 0.5f, 0.5f), 1);
    }
    run(world, 8);
    for (int i = 0; i < 4; i++)
    {
        const Vector3 p = bodies[i]->body.getPosition();
        CHECK(p.y == doctest::Approx(0.5 + i).epsilon(0.03));
        CHECK(std::abs(p.x) < 0.05f);
        CHECK(std::abs(p.z) < 0.05f);
    }
    CHECK(world.getAwakeCount() == 0u);
}

TEST_CASE("The simultaneous resolver handles friction, spin, and bounce")
{
    SUBCASE("friction stops a sliding box")
    {
        World world;
        world.resolverType = World::Resolver::SIMULTANEOUS;
        world.friction = 0.8f;
        world.restitution = 0;
        PlaneCollider ground(Vector3(0, 1, 0), 0);
        world.colliders.push_back(&ground);
        std::vector<std::unique_ptr<Body>> bodies;
        Body* pBox = addBox(world, bodies, Vector3(0, 0.5f, 0), Vector3(0.5f, 0.5f, 0.5f), 1);
        pBox->body.setCanSleep(false);
        pBox->body.setLinearDamping(1);
        pBox->body.setAngularDamping(1);
        pBox->body.setLinearVelocity(Vector3(5, 0, 0));
        run(world, 2);
        CHECK(std::abs(pBox->body.getLinearVelocity().x) < 0.1f);
        CHECK(pBox->body.getPosition().x == doctest::Approx(25.0 / (2 * 0.8 * 9.81)).epsilon(0.25));
        CHECK(upAlignment(pBox->body) > 0.99f);
    }
    SUBCASE("an off-centre hit spins the box")
    {
        World world;
        world.resolverType = World::Resolver::SIMULTANEOUS;
        world.friction = 0;
        world.restitution = 0.5f;
        std::vector<std::unique_ptr<Body>> bodies;
        Body* pBox = addBox(world, bodies, Vector3(0, 0, 0), Vector3(1, 1, 1), 1);
        Body* pBall = addBall(world, bodies, Vector3(-3, 0.8f, 0), 0.25f, 1);
        pBox->body.setAcceleration(Vector3());
        pBall->body.setAcceleration(Vector3());
        pBall->body.setLinearVelocity(Vector3(10, 0, 0));
        run(world, 0.5f);
        CHECK(pBox->body.getAngularVelocity().z < -0.5f);
        CHECK(pBox->body.getLinearVelocity().x > 1);
    }
    SUBCASE("a dropped ball bounces to about the expected height")
    {
        World world;
        world.resolverType = World::Resolver::SIMULTANEOUS;
        world.friction = 0;
        world.restitution = 0.5f;
        PlaneCollider ground(Vector3(0, 1, 0), 0);
        world.colliders.push_back(&ground);
        std::vector<std::unique_ptr<Body>> bodies;
        Body* pBall = addBall(world, bodies, Vector3(0, 2.5f, 0), 0.5f, 1);
        pBall->body.setLinearDamping(1);
        real peak = 0;
        bool bounced = false;
        for (int i = 0; i < 300; i++)
        {
            world.runPhysics(DT);
            bounced = bounced || pBall->body.getLinearVelocity().y > 0;
            if (bounced)
            {
                peak = std::max(peak, pBall->body.getPosition().y);
            }
        }
        CHECK(peak - 0.5f == doctest::Approx(0.5).epsilon(0.25));
    }
}

TEST_CASE("Convex shapes rest on a box platform under both resolvers")
{
    for (World::Resolver kind : { World::Resolver::SEQUENTIAL, World::Resolver::SIMULTANEOUS })
    {
        World world;
        world.resolverType = kind;
        BoxCollider platform(Vector3(4, 0.5f, 4));
        platform.offset.data[7] = -0.5f;
        world.colliders.push_back(&platform);
        std::vector<std::unique_ptr<Body>> bodies;
        Body* pWedge = addConvex(world, bodies, ConvexCollider::wedge(Vector3(0.6f, 0.4f, 0.5f)), Vector3(0, 1, 0), 1);
        Body* pBox = addBox(world, bodies, Vector3(0, 2.5f, 0.1f), Vector3(0.3f, 0.3f, 0.3f), 0.5f);
        run(world, 10);
        CHECK(pWedge->body.getPosition().y > 0.1f);
        CHECK(pWedge->body.getPosition().y < 0.8f);
        CHECK(pBox->body.getPosition().y > 0.2f);
        CHECK(world.getAwakeCount() == 0u);
    }
}

TEST_CASE("A wedge tipped very slightly on a box still rests on all four corners of its base")
{
    BoxCollider platform(Vector3(4, 0.5f, 4));
    platform.offset.data[7] = -0.5f;
    platform.updateInternals();
    for (real tilt : { (real)0, (real)0.003, (real)-0.006 })
    {
        RigidBody body;
        ConvexCollider wedge = ConvexCollider::wedge(Vector3(0.6f, 0.4f, 0.5f));
        wedge.pBody = &body;
        body.setPosition(Vector3(0, 0.2628f, 0));
        Quaternion orientation(1, tilt * 0.5f, 0, tilt * 0.3f);
        orientation.normalise();
        body.setOrientation(orientation);
        wedge.updateInternals();

        // The platform's face can win the axis test, and a box has a face each way along that axis; picking the
        // wrong one used to leave the wedge balanced on a single corner
        Contact contacts[16];
        CollisionData data;
        data.contactArray = contacts;
        data.reset(16);
        CHECK(CollisionTests::Generate(wedge, platform, &data) == 4);
    }
}

TEST_CASE("Convex shapes settle and sleep from many slightly different starts")
{
    // Settling shouldn't depend on one lucky starting position, or on how a compiler rounds
    for (World::Resolver kind : { World::Resolver::SEQUENTIAL, World::Resolver::SIMULTANEOUS })
    {
        for (int n = 0; n < 24; n++)
        {
            World world;
            world.resolverType = kind;
            BoxCollider platform(Vector3(4, 0.5f, 4));
            platform.offset.data[7] = -0.5f;
            world.colliders.push_back(&platform);
            std::vector<std::unique_ptr<Body>> bodies;
            addConvex(world, bodies, ConvexCollider::wedge(Vector3(0.6f, 0.4f, 0.5f)), Vector3(0, 1, 0), 1);
            const Vector3 start((real)(n - 12) * 0.0025f, 2.5f, 0.1f + (real)(n % 5) * 0.004f);
            addBox(world, bodies, start, Vector3(0.3f, 0.3f, 0.3f), 0.5f);
            run(world, 10);
            CHECK(world.getAwakeCount() == 0u);
        }
    }
}
