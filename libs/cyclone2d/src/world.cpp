#include "world.h"

#include <algorithm>
#include <numeric>
#include <unordered_map>

using namespace cyclone2d;

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

    // Planes reach everywhere; other shapes are skipped when their bounding circles are apart
    bool mightTouch(const Collider& a, const Collider& b)
    {
        const real reachA = a.getBoundingRadius();
        const real reachB = b.getBoundingRadius();
        if (reachA < 0 || reachB < 0)
        {
            return true;
        }
        const real reach = reachA + reachB;
        return (a.getPosition() - b.getPosition()).squareMagnitude() <= reach * reach;
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

    for (std::size_t i = 0; i < colliders.size() && data.contactsLeft > 0; i++)
    {
        for (std::size_t j = i + 1; j < colliders.size() && data.contactsLeft > 0; j++)
        {
            const Collider& a = *colliders[i];
            const Collider& b = *colliders[j];
            if (!worthTesting(a, b) || !mightTouch(a, b) || joined(a.pBody, b.pBody))
            {
                continue;
            }
            pairsTested++;
            CollisionTests::Generate(a, b, &data);
        }
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
    resolver.resolveContacts(contacts.data(), contactCount, duration);
    solveJoints();
    updateSleep();
}

void World::solveJoints()
{
    std::vector<const Joint*> active;
    for (const Joint* pJoint : joints)
    {
        if (pJoint->prepare())
        {
            active.push_back(pJoint);
        }
    }
    for (unsigned i = 0; i < jointVelocityIterations; i++)
    {
        for (const Joint* pJoint : active)
        {
            pJoint->solveVelocity();
        }
    }

    // Each pass works from where the bodies are now, so it converges even when they have turned a long way
    for (unsigned i = 0; i < jointPositionIterations; i++)
    {
        real worst = 0;
        for (const Joint* pJoint : active)
        {
            worst = std::max(worst, pJoint->solvePosition() - pJoint->error);
        }
        if (worst < (real)0.0005)
        {
            break;
        }
    }
}

bool World::joined(const RigidBody* a, const RigidBody* b) const
{
    if (a == nullptr || b == nullptr)
    {
        return false;
    }
    for (const Joint* pJoint : joints)
    {
        if ((pJoint->body[0] == a && pJoint->body[1] == b) || (pJoint->body[0] == b && pJoint->body[1] == a))
        {
            return true;
        }
    }
    return false;
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
        while (parent[(std::size_t)i] != i)
        {
            parent[(std::size_t)i] = parent[(std::size_t)parent[(std::size_t)i]];
            i = parent[(std::size_t)i];
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
            parent[(std::size_t)find(a->second)] = find(b->second);
        }
    }

    for (const Joint* pJoint : joints)
    {
        auto a = indexOf.find(pJoint->body[0]);
        auto b = indexOf.find(pJoint->body[1]);
        if (a != indexOf.end() && b != indexOf.end() && pJoint->body[0]->hasFiniteMass() && pJoint->body[1]->hasFiniteMass())
        {
            parent[(std::size_t)find(a->second)] = find(b->second);
        }
    }

    // An island sleeps only when every awake body in it wants to
    std::vector<char> restless(bodies.size(), 0);
    for (int i = 0; i < (int)bodies.size(); i++)
    {
        const RigidBody* pBody = bodies[(std::size_t)i];
        if (pBody->hasFiniteMass() && pBody->isAwake() && !pBody->wantsToSleep())
        {
            restless[(std::size_t)find(i)] = 1;
        }
    }
    for (int i = 0; i < (int)bodies.size(); i++)
    {
        RigidBody* pBody = bodies[(std::size_t)i];
        if (pBody->hasFiniteMass() && pBody->isAwake() && !restless[(std::size_t)find(i)])
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

Collider* World::findColliderAt(const Vector2& worldPoint) const
{
    for (Collider* pCollider : colliders)
    {
        if (pCollider->type != ColliderType::Plane && pCollider->containsPoint(worldPoint))
        {
            return pCollider;
        }
    }
    return nullptr;
}
