#pragma once

#include <vector>

#include "collision.h"
#include "core.h"
#include "rigidBody.h"

namespace cyclone
{
    // The rigid-body contact resolver (chapters 14 to 16): spin from off-centre hits, friction, still resting
    // contacts, and overlap fixed by moving and turning. Worst contacts go first.
    class RigidContactResolver
    {
    public:
        // 0 means 32 times the number of contacts, decided on each call. Fewer leave boxes overlapping after an impact
        // on a stack.
        unsigned velocityIterations = 0;
        unsigned positionIterations = 0;

        // Changes smaller than these are left alone. Larger ones let stacks sink and snap back out (shaking) or
        // slowly lean under an off-centre load.
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
        // Per-contact working data, in the contact's own coordinates: x along the normal, y and z across it
        struct ContactState
        {
            RigidBody* body[2] = { nullptr, nullptr };
            Vector3 normal;
            real penetration = 0;
            real friction = 0;
            real restitution = 0;
            Matrix3 contactToWorld;
            Vector3 contactVelocity;
            real desiredDeltaVelocity = 0;
            Vector3 relativeContactPosition[2];
        };

        void prepareContacts(const Contact* contacts, unsigned numContacts, real duration);
        void adjustPositions(real duration);
        void adjustVelocities(real duration);

        void calculateContactBasis(ContactState& c) const;
        Vector3 calculateLocalVelocity(const ContactState& c, int bodyIndex, real duration) const;
        void calculateDesiredDeltaVelocity(ContactState& c, real duration) const;
        Vector3 calculateFrictionlessImpulse(const ContactState& c, const Matrix3* inverseInertia) const;
        Vector3 calculateFrictionImpulse(const ContactState& c, const Matrix3* inverseInertia) const;
        void applyVelocityChange(ContactState& c, Vector3 velocityChange[2], Vector3 rotationChange[2]) const;
        void applyPositionChange(ContactState& c, Vector3 linearChange[2], Vector3 angularChange[2], real penetration) const;
        static void matchAwakeState(ContactState& c);

        std::vector<ContactState> states;
        unsigned velocityIterationsUsed = 0;
        unsigned positionIterationsUsed = 0;
    };
}
