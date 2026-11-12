#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "broadPhase.h"
#include "collision.h"
#include "rigidBody.h"
#include "rigidBodyForces.h"
#include "rigidContactResolver.h"
#include "simultaneousResolver.h"

namespace cyclone
{
    // The rigid-body engine (chapters 11 and 17): forces, integration, contacts and resolution each frame. It doesn't
    // own what's added. Static scenery is simplest as a collider with no body.
    class World
    {
    public:
        // How pairs of colliders are found before the exact tests (chapter 12)
        enum class BroadPhase
        {
            ALL_PAIRS,
            BOUNDING_VOLUME_HIERARCHY,
            MULTI_RESOLUTION_MAP,
            BSP_TREE
        };

        // One contact at a time, worst first (chapters 14 to 16), or all together (chapter 20)
        enum class Resolver
        {
            SEQUENTIAL,
            SIMULTANEOUS
        };

        explicit World(unsigned maxContacts = 1024);

        std::vector<RigidBody*> bodies;
        std::vector<Collider*> colliders;
        std::vector<ContactGenerator*> contactGenerators;
        RigidBodyForceRegistry forces;
        RigidContactResolver resolver;
        SimultaneousContactResolver simultaneousResolver;

        BroadPhase broadPhase = BroadPhase::BOUNDING_VOLUME_HIERARCHY;
        Resolver resolverType = Resolver::SEQUENTIAL;

        // Remembers which axis separated each pair of boxes or convex shapes, and tries it first next frame
        bool useCoherence = true;

        // Given to every contact between colliders; generators such as joints set their own
        real friction = (real)0.6;
        real restitution = (real)0.2;

        void runPhysics(real duration);
        void integrate(real duration);
        unsigned generateContacts();

        unsigned getContactCount() const { return contactCount; }
        const Contact* getContacts() const { return contacts.data(); }
        unsigned getAwakeCount() const;

        // Pairs that reached the exact tests last frame, and how many of those the cached axis settled at once
        unsigned getPairsTested() const { return pairsTested; }
        unsigned getCoherenceHits() const { return coherenceHits; }
        void clearCoherence() { satCaches.clear(); }

    private:
        // Touching bodies only sleep together, or an awake one would keep waking its sleeping neighbour
        void updateSleep();
        unsigned generateContactsOnce();
        bool wakeTouchedSleepers(unsigned count);
        void testPair(Collider& a, Collider& b, CollisionData& data);
        void findPairs(std::vector<PotentialContact>& pairs);

        std::vector<Contact> contacts;
        unsigned contactCount = 0;
        unsigned pairsTested = 0;
        unsigned coherenceHits = 0;
        std::unordered_map<std::uint64_t, SatCache> satCaches;

        BoundingVolumeHierarchy hierarchy;
        MultiResolutionMap multiResolution{ (real)0.25, 10 };
        BspTree bspTree;
    };
}
