#pragma once

#include <vector>

#include "collision.h"
#include "core.h"
#include "rigidBody.h"

namespace cyclone2d
{
    // The rigid-body contact resolver (chapters 14 to 16) in the plane: spin from off-centre hits, friction, still
    // resting contacts, and overlap fixed by moving and turning. Worst contacts go first.
    class ContactResolver
    {
    public:
        // 0 means 32 times the number of contacts, decided on each call
        unsigned velocityIterations = 0;
        unsigned positionIterations = 0;

        // Changes smaller than these are left alone
        real velocityEpsilon = (real)0.001;
        real positionEpsilon = (real)0.001;

        // Below this closing speed a contact doesn't bounce, however high its restitution
        real velocityLimit = (real)0.25;

        // How much of the interpenetration may be fixed by turning instead of moving, per unit of lever arm
        real angularLimit = (real)0.2;

        void resolveContacts(Contact* contacts, unsigned numContacts, real duration);

        unsigned getVelocityIterationsUsed() const { return velocityIterationsUsed; }
        unsigned getPositionIterationsUsed() const { return positionIterationsUsed; }

    private:
        // Per-contact working data. Velocities are in contact coordinates: x along the normal, y along the tangent.
        struct ContactState
        {
            RigidBody* body[2] = { nullptr, nullptr };
            Vector2 normal;
            Vector2 tangent;
            real penetration = 0;
            real friction = 0;
            real restitution = 0;
            Vector2 contactVelocity;
            real desiredDeltaVelocity = 0;
            Vector2 relativeContactPosition[2];
        };

        void prepareContacts(const Contact* contacts, unsigned numContacts, real duration);
        void adjustPositions();
        void adjustVelocities(real duration);

        Vector2 calculateLocalVelocity(const ContactState& c, int bodyIndex, real duration) const;
        void calculateDesiredDeltaVelocity(ContactState& c, real duration) const;
        Vector2 calculateFrictionlessImpulse(const ContactState& c) const;
        Vector2 calculateFrictionImpulse(const ContactState& c) const;
        void applyVelocityChange(ContactState& c, Vector2 velocityChange[2], real rotationChange[2]) const;
        void applyPositionChange(ContactState& c, Vector2 linearChange[2], real angularChange[2], real penetration) const;
        static void matchAwakeState(ContactState& c);

        std::vector<ContactState> states;
        unsigned velocityIterationsUsed = 0;
        unsigned positionIterationsUsed = 0;
    };
}
