#pragma once

#include "collision.h"
#include "core.h"
#include "rigidBody.h"

namespace cyclone
{
    // Holds a point on one body near a point on another, or on a fixed point when body[1] is null, by making a
    // contact that pulls them together (chapter 17.2.1).
    class Joint : public ContactGenerator
    {
    public:
        RigidBody* body[2] = { nullptr, nullptr };
        Vector3 position[2]; // Each in its own body's space; position[1] is in world space when body[1] is null
        real error = 0;

        void set(RigidBody* a, const Vector3& aPosition, RigidBody* b, const Vector3& bPosition, real error);
        unsigned addContact(Contact* contact, unsigned limit) const override;

        Vector3 getWorldPosition(int index) const;
    };
}
