#include "world.h"

#include <numeric>

using namespace cyclone;

namespace
{
    // Nothing to do when neither side can move, or both are asleep
    bool worthTesting(const Collider& a, const Collider& b)
    {
        const RigidBody* bodyA = a.pBody;
        const RigidBody* bodyB = b.pBody;
        if (bodyA != nullptr && bodyA == bodyB)
        {
            return false;
        }
        const bool movesA = bodyA != nullptr && bodyA->hasFiniteMass();
        const bool movesB = bodyB != nullptr && bodyB->hasFiniteMass();
        if (!movesA && !movesB)
        {
            return false;
        }
        const bool awakeA = movesA && bodyA->isAwake();
        const bool awakeB = movesB && bodyB->isAwake();
        return awakeA || awakeB;
    }

    // How far the collider reaches from its centre; planes reach everywhere
    real reachOf(const Collider& collider)
    {
        switch (collider.type)
        {
        case ColliderType::Sphere:
            return static_cast<const SphereCollider&>(collider).radius;
        case ColliderType::Box:
            return static_cast<const BoxCollider&>(collider).halfSize.magnitude();
        case ColliderType::Convex:
            return static_cast<const ConvexCollider&>(collider).getBoundingRadius();
        case ColliderType::Plane:
            return -1;
        }
        return -1;
    }

    bool usesSat(const Collider& collider)
    {
        return collider.type == ColliderType::Box || collider.type == ColliderType::Convex;
    }
}

World::World(unsigned maxContacts) : contacts(maxContacts > 0 ? maxContacts : 1)
{
}

void World::integrate(real duration)
{
    for (RigidBody* pBody : bodies)
    {
        if (pBody->hasFiniteMass())
        {
            pBody->integrate(duration, false);
        }
    }
}

void World::testPair(Collider& a, Collider& b, CollisionData& data)
{
    if (!worthTesting(a, b))
    {
        return;
    }
    pairsTested++;
    SatCache* cache = nullptr;
    if (useCoherence && usesSat(a) && usesSat(b))
    {
        const std::uint64_t ka = (std::uint64_t)(std::uintptr_t)&a;
        const std::uint64_t kb = (std::uint64_t)(std::uintptr_t)&b;
        cache = &satCaches[ka * 0x9E3779B97F4A7C15ull ^ kb];
    }
    CollisionTests::Generate(a, b, &data, cache);
    if (cache != nullptr && cache->lastTestHit)
    {
        coherenceHits++;
    }
}

void World::findPairs(std::vector<PotentialContact>& pairs)
{
    // Planes have no bounding sphere, so they're tested against everything; the rest go through the broad phase
    std::vector<BroadPhaseItem> items;
    std::vector<int> planes;
    for (int i = 0; i < (int)colliders.size(); i++)
    {
        const real reach = reachOf(*colliders[i]);
        if (reach < 0)
        {
            planes.push_back(i);
        }
        else
        {
            items.push_back(BroadPhaseItem{ i, colliders[i]->getAxis(3), reach });
        }
    }

    switch (broadPhase)
    {
    case BroadPhase::ALL_PAIRS:
        for (std::size_t i = 0; i < items.size(); i++)
        {
            for (std::size_t j = i + 1; j < items.size(); j++)
            {
                pairs.push_back(PotentialContact{ items[i].id, items[j].id });
            }
        }
        break;
    case BroadPhase::BOUNDING_VOLUME_HIERARCHY:
        hierarchy.clear();
        for (const BroadPhaseItem& item : items)
        {
            hierarchy.insert(item.id, BoundingSphere(item.center, item.radius));
        }
        hierarchy.getPotentialContacts(pairs);
        break;
    case BroadPhase::MULTI_RESOLUTION_MAP:
        multiResolution.clear();
        for (const BroadPhaseItem& item : items)
        {
            multiResolution.insert(item);
        }
        multiResolution.getPotentialContacts(pairs);
        break;
    case BroadPhase::BSP_TREE:
        bspTree.build(items);
        bspTree.getPotentialContacts(pairs);
        break;
    }

    for (int plane : planes)
    {
        for (int i = 0; i < (int)colliders.size(); i++)
        {
            if (i != plane)
            {
                pairs.push_back(PotentialContact{ std::min(plane, i), std::max(plane, i) });
            }
        }
    }
}

unsigned World::generateContacts()
{
    for (Collider* pCollider : colliders)
    {
        pCollider->updateInternals();
    }

    // Pairs of sleeping bodies aren't tested, so a sleeper touched by an awake body is woken and the contacts found
    // again; otherwise it would be pushed into its sleeping neighbours with no contact to stop it
    unsigned count = 0;
    for (std::size_t pass = 0; pass <= bodies.size(); pass++)
    {
        count = generateContactsOnce();
        if (!wakeTouchedSleepers(count))
        {
            break;
        }
    }
    return count;
}

bool World::wakeTouchedSleepers(unsigned count)
{
    bool woke = false;
    for (unsigned c = 0; c < count; c++)
    {
        RigidBody* pA = contacts[c].body[0];
        RigidBody* pB = contacts[c].body[1];
        if (pA == nullptr || pB == nullptr || !pA->hasFiniteMass() || !pB->hasFiniteMass() || pA->isAwake() == pB->isAwake())
        {
            continue;
        }
        (pA->isAwake() ? pB : pA)->setAwake(true);
        woke = true;
    }
    return woke;
}

unsigned World::generateContactsOnce()
{
    CollisionData data;
    data.contactArray = contacts.data();
    data.friction = friction;
    data.restitution = restitution;
    data.reset((int)contacts.size());
    pairsTested = 0;
    coherenceHits = 0;

    std::vector<PotentialContact> pairs;
    findPairs(pairs);
    for (const PotentialContact& pair : pairs)
    {
        if (data.contactsLeft <= 0)
        {
            break;
        }
        // Lower index first, so contacts come out the same whichever broad phase found the pair
        testPair(*colliders[pair.a], *colliders[pair.b], data);
    }

    for (ContactGenerator* pGenerator : contactGenerators)
    {
        if (data.contactsLeft <= 0)
        {
            break;
        }
        const unsigned used = pGenerator->addContact(data.contacts, (unsigned)data.contactsLeft);
        data.addContacts((int)used);
    }
    return (unsigned)data.contactCount;
}

void World::runPhysics(real duration)
{
    forces.updateForces(duration);
    integrate(duration);
    contactCount = generateContacts();
    if (resolverType == Resolver::SIMULTANEOUS)
    {
        simultaneousResolver.resolveContacts(contacts.data(), contactCount, duration);
    }
    else
    {
        resolver.resolveContacts(contacts.data(), contactCount, duration);
    }
    updateSleep();
}

void World::updateSleep()
{
    std::unordered_map<const RigidBody*, int> indexOf;
    for (int i = 0; i < (int)bodies.size(); i++)
    {
        indexOf[bodies[i]] = i;
    }

    // Union-find over the contacts between movable bodies
    std::vector<int> parent(bodies.size());
    std::iota(parent.begin(), parent.end(), 0);
    auto find = [&](int i)
    {
        while (parent[i] != i)
        {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    };
    for (unsigned c = 0; c < contactCount; c++)
    {
        const Contact& contact = contacts[c];
        if (contact.body[0] == nullptr || contact.body[1] == nullptr || !contact.body[0]->hasFiniteMass() ||
            !contact.body[1]->hasFiniteMass())
        {
            continue;
        }
        auto a = indexOf.find(contact.body[0]);
        auto b = indexOf.find(contact.body[1]);
        if (a != indexOf.end() && b != indexOf.end())
        {
            parent[find(a->second)] = find(b->second);
        }
    }

    // An island sleeps only when every awake body in it wants to
    std::vector<char> restless(bodies.size(), 0);
    for (int i = 0; i < (int)bodies.size(); i++)
    {
        const RigidBody* pBody = bodies[i];
        if (pBody->hasFiniteMass() && pBody->isAwake() && !pBody->wantsToSleep())
        {
            restless[find(i)] = 1;
        }
    }
    for (int i = 0; i < (int)bodies.size(); i++)
    {
        RigidBody* pBody = bodies[i];
        if (pBody->hasFiniteMass() && pBody->isAwake() && !restless[find(i)])
        {
            pBody->setAwake(false);
        }
    }
}

unsigned World::getAwakeCount() const
{
    unsigned count = 0;
    for (const RigidBody* pBody : bodies)
    {
        if (pBody->hasFiniteMass() && pBody->isAwake())
        {
            count++;
        }
    }
    return count;
}
