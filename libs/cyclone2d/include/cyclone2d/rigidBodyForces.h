#pragma once

#include <vector>

#include "core.h"
#include "rigidBody.h"

namespace cyclone2d
{
    // Anything that pushes or turns rigid bodies each frame. Chapter 10.3.1.
    class RigidBodyForceGenerator
    {
    public:
        virtual ~RigidBodyForceGenerator() = default;
        virtual void updateForce(RigidBody* body, real duration) = 0;
    };

    // Which generators act on which bodies. Doesn't own either.
    class RigidBodyForceRegistry
    {
    public:
        void add(RigidBody* body, RigidBodyForceGenerator* generator);
        void remove(RigidBody* body, RigidBodyForceGenerator* generator);
        void clear() { registrations.clear(); }
        void updateForces(real duration);

    private:
        struct Registration
        {
            RigidBody* body;
            RigidBodyForceGenerator* generator;
        };
        std::vector<Registration> registrations;
    };

    // Gravity as a force, for bodies whose built-in acceleration is set to zero
    class RigidBodyGravity : public RigidBodyForceGenerator
    {
    public:
        explicit RigidBodyGravity(const Vector2& gravity = Vector2::GRAVITY) : gravity(gravity) {}
        void updateForce(RigidBody* body, real duration) override;

        Vector2 gravity;
    };

    // A spring between a point on this body and a point on another, both in each body's own space. With no other
    // body, otherConnectionPoint is a fixed point in the world.
    class RigidBodySpring : public RigidBodyForceGenerator
    {
    public:
        RigidBodySpring(const Vector2& localConnectionPoint, RigidBody* other, const Vector2& otherConnectionPoint, real springConstant,
                        real restLength, real damping = 0)
            : connectionPoint(localConnectionPoint), other(other), otherConnectionPoint(otherConnectionPoint),
              springConstant(springConstant), restLength(restLength), damping(damping)
        {
        }
        void updateForce(RigidBody* body, real duration) override;

        Vector2 connectionPoint;
        RigidBody* other;
        Vector2 otherConnectionPoint;
        real springConstant;
        real restLength;
        real damping;
    };

    // Buoyancy at one point of the body (chapter 11.2.2). Several at different points let a hull float level and rock.
    class RigidBodyBuoyancy : public RigidBodyForceGenerator
    {
    public:
        RigidBodyBuoyancy(const Vector2& centreOfBuoyancy, real maxDepth, real volume, real waterHeight, real liquidDensity = 1000,
                          real gravity = (real)9.81)
            : centreOfBuoyancy(centreOfBuoyancy), maxDepth(maxDepth), volume(volume), waterHeight(waterHeight),
              liquidDensity(liquidDensity), gravity(gravity)
        {
        }
        void updateForce(RigidBody* body, real duration) override;

        Vector2 centreOfBuoyancy; // In the body's own space
        real maxDepth;
        real volume;
        real waterHeight;
        real liquidDensity;
        real gravity;
    };
}
