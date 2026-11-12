#pragma once

#include <vector>

#include "core.h"
#include "rigidBody.h"

namespace cyclone
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
        void clear();
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
        explicit RigidBodyGravity(const Vector3& gravity) : gravity(gravity) {}
        void updateForce(RigidBody* body, real duration) override;

        Vector3 gravity;
    };

    // A spring between a point on this body and a point on another, both in each body's own space
    class RigidBodySpring : public RigidBodyForceGenerator
    {
    public:
        RigidBodySpring(const Vector3& localConnectionPoint, RigidBody* other, const Vector3& otherConnectionPoint, real springConstant,
                        real restLength, real damping = 0);
        void updateForce(RigidBody* body, real duration) override;

        Vector3 connectionPoint;
        RigidBody* other;
        Vector3 otherConnectionPoint;
        real springConstant;
        real restLength;
        real damping;
    };

    // Buoyancy at one point of the body (chapter 11.2.2). Several at different points let a hull float level
    // and rock.
    class RigidBodyBuoyancy : public RigidBodyForceGenerator
    {
    public:
        RigidBodyBuoyancy(const Vector3& centreOfBuoyancy, real maxDepth, real volume, real waterHeight, real liquidDensity = 1000,
                          real gravity = (real)9.81);
        void updateForce(RigidBody* body, real duration) override;

        Vector3 centreOfBuoyancy;
        real maxDepth; // How far below the point the body is fully submerged
        real volume;
        real waterHeight;
        real liquidDensity;
        real gravity;
    };

    // An aerodynamic surface (chapter 11.2.1): its tensor turns the airflow over the body into a force at position.
    // windSpeed may be null for still air.
    class Aero : public RigidBodyForceGenerator
    {
    public:
        Aero(const Matrix3& tensor, const Vector3& position, const Vector3* windSpeed = nullptr);
        void updateForce(RigidBody* body, real duration) override;

        Matrix3 tensor;
        Vector3 position;
        const Vector3* windSpeed;

    protected:
        void updateForceFromTensor(RigidBody* body, const Matrix3& tensorToUse) const;
    };

    // A control surface such as an aileron or rudder: the tensor blends between three as the control moves from -1 to 1
    class AeroControl : public Aero
    {
    public:
        AeroControl(const Matrix3& base, const Matrix3& min, const Matrix3& max, const Vector3& position,
                    const Vector3* windSpeed = nullptr);
        void updateForce(RigidBody* body, real duration) override;

        void setControl(real value);
        real getControl() const { return controlSetting; }
        Matrix3 getTensor() const;

        Matrix3 minTensor;
        Matrix3 maxTensor;

    private:
        real controlSetting = 0;
    };

    // A surface mounted at an angle to the body, such as a sail that can be turned
    class AngledAero : public Aero
    {
    public:
        AngledAero(const Matrix3& tensor, const Vector3& position, const Vector3* windSpeed = nullptr);
        void updateForce(RigidBody* body, real duration) override;

        // The surface's orientation relative to the body
        void setOrientation(const Quaternion& orientation);

    private:
        Quaternion orientation;
    };

    // An explosion (chapter 17.2.3): a brief pull inward, a shock wave pushing outward, then rising hot air above
    // the blast. Call advance() once per frame.
    class Explosion : public RigidBodyForceGenerator
    {
    public:
        Vector3 detonation;

        real implosionMaxRadius = 0;
        real implosionMinRadius = 0;
        real implosionDuration = 0;
        real implosionForce = 0;

        real shockwaveSpeed = 40;
        real shockwaveThickness = 3;
        real peakConcussionForce = 400;
        real concussionDuration = 1;

        real peakConvectionForce = 60;
        real chimneyRadius = 3;
        real chimneyHeight = 20;
        real convectionDuration = 2;

        void detonate(const Vector3& position);
        void advance(real duration);
        bool isActive() const;
        real getTimePassed() const { return timePassed; }

        void updateForce(RigidBody* body, real duration) override;

    private:
        real timePassed = REAL_MAX; // Nothing happens until detonate()
    };
}
