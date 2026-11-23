#pragma once

#include "core.h"

namespace cyclone2d
{
    // A body that moves and turns in the plane (chapter 10). Its orientation is one angle and its inertia one number.
    class RigidBody
    {
    public:
        // Below this average motion (squared speed plus squared spin) a body falls asleep. Chapter 16.2.1.
        static real sleepEpsilon;

        Vector2 getPosition() const { return m_position; }
        void setPosition(Vector2 p) { m_position = p; }
        Vector2 getLinearVelocity() const { return m_linearVelocity; }
        // A non-zero velocity wakes a sleeping body, since it would otherwise be ignored
        void setLinearVelocity(Vector2 v);
        Vector2 getAcceleration() const { return m_acceleration; }
        void setAcceleration(Vector2 a) { m_acceleration = a; }
        real getLinearDamping() const { return m_linearDamping; }
        void setLinearDamping(real d) { m_linearDamping = d; }

        // Radians per second, counter-clockwise
        real getAngularVelocity() const { return m_angularVelocity; }
        void setAngularVelocity(real w);
        real getAngularDamping() const { return m_angularDamping; }
        void setAngularDamping(real d) { m_angularDamping = d; }

        // Radians, counter-clockwise
        real getOrientation() const { return m_orientation; }
        void setOrientation(real angle);
        const Rotation& getRotation() const { return m_rotation; }
        Transform getTransform() const { return Transform(m_position, m_rotation); }

        // A moment of inertia of 0 is infinite: the body never turns
        void setInertia(real inertia);
        real getInverseInertia() const { return m_inverseInertia; }
        void setInverseInertia(real inverseInertia) { m_inverseInertia = inverseInertia; }

        // Sets both mass and inertia for a solid shape of that mass
        void setBoxMass(real mass, const Vector2& halfSize);
        void setCircleMass(real mass, real radius);

        Vector2 getPointInWorldSpace(const Vector2& localPoint) const { return getTransform().transform(localPoint); }
        Vector2 getPointInLocalSpace(const Vector2& worldPoint) const { return getTransform().transformInverse(worldPoint); }
        Vector2 getDirectionInWorldSpace(const Vector2& localDirection) const { return m_rotation.rotate(localDirection); }
        Vector2 getDirectionInLocalSpace(const Vector2& worldDirection) const { return m_rotation.rotateInverse(worldDirection); }

        // The velocity of a point fixed to the body, given in world space
        Vector2 getVelocityAtPoint(const Vector2& worldPoint) const;

        void integrate(real dt);

        // With allowSleep false the body still tracks its motion but never falls asleep by itself; World uses this
        // so touching bodies sleep together
        void integrate(real dt, bool allowSleep);

        // A mass of 0 is infinite: the body never moves
        void setMass(real mass);
        real getInverseMass() const { return m_inverseMass; }
        void setInverseMass(real inverseMass) { m_inverseMass = inverseMass; }
        real getMass() const { return (m_inverseMass <= 0) ? REAL_MAX : ((real)1.0) / m_inverseMass; }
        bool hasFiniteMass() const { return m_inverseMass > 0; }

        // Forces and torques last until the next integrate(). Points are in world space unless named body points.
        void addForce(Vector2 force);
        void addForceAtPoint(Vector2 force, Vector2 point);
        void addForceAtBodyPoint(Vector2 force, Vector2 localPoint);
        void addTorque(real torque);

        // The acceleration the last integrate() used, gravity plus forces. Resting contacts read it.
        Vector2 getLastFrameAcceleration() const { return m_lastFrameAcceleration; }

        bool isAwake() const { return m_isAwake; }
        // Putting a body to sleep stops it dead; waking it gives it a moment before it can sleep again
        void setAwake(bool awake = true);
        bool canSleep() const { return m_canSleep; }
        void setCanSleep(bool canSleep);
        real getMotion() const { return m_motion; }
        bool wantsToSleep() const { return m_canSleep && m_motion < sleepEpsilon; }

    private:
        Vector2 m_position;
        Vector2 m_linearVelocity;
        Vector2 m_acceleration = Vector2::GRAVITY;
        Vector2 m_lastFrameAcceleration;
        Vector2 m_forceAccum;
        real m_linearDamping = (real)0.99;
        real m_inverseMass = 1;
        real m_orientation = 0;
        Rotation m_rotation;
        real m_angularVelocity = 0;
        real m_torqueAccum = 0;
        real m_inverseInertia = 0; // No turning until an inertia is set
        real m_angularDamping = (real)0.99;
        real m_motion = (real)0.6; // Twice the default sleep epsilon, so a new body doesn't sleep straight away
        bool m_isAwake = true;
        bool m_canSleep = true;

        void clearAccumulators();
        void wakeIfAsleep();
    };
}
