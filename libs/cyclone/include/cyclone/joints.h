#pragma once

#include "collision.h"
#include "core.h"
#include "rigidBody.h"

namespace cyclone
{
    // Pins a point on one body to a point on another, or to a fixed point when body[1] is null; the bodies turn
    // freely about it, like a ball joint. Add it to World::joints, which holds the points together exactly.
    class Joint : public ContactGenerator
    {
    public:
        RigidBody* body[2] = { nullptr, nullptr };
        Vector3 position[2]; // Each in its own body's space; position[1] is in world space when body[1] is null
        real error = 0;

        void set(RigidBody* a, const Vector3& aPosition, RigidBody* b, const Vector3& bPosition, real error);

        // The book's version (chapter 17.2.1): one contact pulling the points together. Added to contactGenerators
        // it lets them slide sideways, so chains wobble and spin.
        unsigned addContact(Contact* contact, unsigned limit) const override;

        Vector3 getWorldPosition(int index) const;

        // Gives both points the same velocity, as one impulse in any direction
        void solveVelocity() const;

        // Moves and turns the bodies so the points meet; returns how far apart they were
        real solvePosition() const;

        // A sleeping body on an awake one is woken; returns false when nothing on the joint can move
        bool prepare() const;
    };
}
