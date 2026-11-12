#pragma once

#include <vector>

#include "collision.h"
#include "core.h"
#include "rigidBody.h"

namespace cyclone
{
    // Resolves all contacts together (chapter 20) by projected Gauss-Seidel, reusing last frame's impulses so
    // stacks settle quickly. Overlap is pushed out gently instead of by moving bodies.
    class SimultaneousContactResolver
    {
    public:
        unsigned iterations = 20;

        // Fraction of the penetration (beyond the slop) removed per second, divided by the time step
        real positionCorrection = (real)0.2;
        real penetrationSlop = (real)0.01;

        // Below this closing speed a contact doesn't bounce
        real velocityLimit = (real)0.5;

        bool warmStarting = true;

        void resolveContacts(Contact* contacts, unsigned numContacts, real duration);

    private:
        struct Row
        {
            RigidBody* body[2] = { nullptr, nullptr };
            Vector3 relative[2];
            Vector3 normal;
            Vector3 tangent[2];
            real normalMass = 0;
            real tangentMass[2] = { 0, 0 };
            real bias = 0;
            real friction = 0;
            real normalImpulse = 0;
            real tangentImpulse[2] = { 0, 0 };
            Vector3 localPoint; // On body[0], for matching against last frame
        };

        void applyImpulse(Row& row, const Vector3& impulse) const;
        Vector3 relativeVelocity(const Row& row) const;
        real effectiveMass(const Row& row, const Vector3& direction) const;

        std::vector<Row> rows;
        std::vector<Row> previous;
    };
}
