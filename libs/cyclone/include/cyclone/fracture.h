#pragma once

#include <vector>

#include "core.h"
#include "rigidBody.h"

namespace cyclone
{
    // One piece of a broken box, ready to become a body of its own
    struct BoxFragment
    {
        Vector3 position;
        Quaternion orientation;
        Vector3 halfSize;
        real mass = 0;
        Matrix3 inertiaTensor;
        Vector3 linearVelocity;
        Vector3 angularVelocity;
    };

    // Breaks a box into up to eight pieces through the impact point, given in the box's own space (chapter 17.2.2).
    // Each piece keeps the motion the box had at that spot, plus an outward burst.
    std::vector<BoxFragment> fractureBox(const RigidBody& body, const Vector3& halfSize, const Vector3& localImpactPoint,
                                         real minHalfSize = (real)0.05, real burstSpeed = 0);
}
