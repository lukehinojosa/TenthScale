#pragma once

#include <vector>

#include "collision.h"
#include "contactResolver.h"
#include "rigidBody.h"
#include "rigidBodyForces.h"

namespace cyclone2d
{
    // The rigid-body engine (chapters 11 and 17) in the plane: forces, integration, contacts and resolution each
    // frame. It doesn't own what's added. Static scenery is simplest as a collider with no body.
    class World
    {
    public:
        explicit World(unsigned maxContacts = 1024);

        std::vector<RigidBody*> bodies;
        std::vector<Collider*> colliders;
        std::vector<ContactGenerator*> contactGenerators;

        // Pins solved after the contacts each frame; bodies on the same joint don't collide with each other
        std::vector<Joint*> joints;
        unsigned jointVelocityIterations = 10;
        unsigned jointPositionIterations = 10;
        RigidBodyForceRegistry forces;
        ContactResolver resolver;

        // Given to every contact between colliders; generators such as joints set their own
        real friction = (real)0.6;
        real restitution = (real)0.2;

        void runPhysics(real duration);
        void integrate(real duration);
        unsigned generateContacts();

        unsigned getContactCount() const { return contactCount; }
        const Contact* getContacts() const { return contacts.data(); }
        unsigned getAwakeCount() const;

        // Pairs whose bounding circles overlapped last frame, so they reached the exact tests
        unsigned getPairsTested() const { return pairsTested; }

        // The first collider holding the point, from the last frame's transforms; null when there is none
        Collider* findColliderAt(const Vector2& worldPoint) const;

    private:
        // Touching bodies only sleep together, or an awake one would keep waking its sleeping neighbour
        void updateSleep();
        unsigned generateContactsOnce();
        bool wakeTouchedSleepers(unsigned count);
        void solveJoints();
        bool joined(const RigidBody* a, const RigidBody* b) const;

        std::vector<Contact> contacts;
        unsigned contactCount = 0;
        unsigned pairsTested = 0;
    };
}
