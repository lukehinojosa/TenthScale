#pragma once
#include "core.h"

namespace cyclone
{
    class RigidBody
    {
    public:
        // Below this average motion (squared speed plus squared spin) a body falls asleep. Chapter 16.2.1.
        static real sleepEpsilon;

        Vector3 getPosition() const { return m_position; }
        void setPosition(Vector3 p) { m_position = p; }
        Vector3 getLinearVelocity() const { return m_linearVelocity; }
        // A non-zero velocity wakes a sleeping body, since it would otherwise be ignored
        void setLinearVelocity(Vector3 v);
        Vector3 getAcceleration() const { return m_acceleration; }
        void setAcceleration(Vector3 a) { m_acceleration = a; }
        real getLinearDamping() const { return m_linearDamping; }
        void setLinearDamping(real d) { m_linearDamping = d; }
        Vector3 getAngularVelocity() const { return m_angularVelocity; }
        void setAngularVelocity(Vector3 w);
        real getAngularDamping() const { return m_angularDamping; }
        void setAngularDamping(real d) { m_angularDamping = d; }
        Quaternion getOrientation() const { return m_orientation; }
        // Normalised straight away, since only unit quaternions are valid rotations.
        void setOrientation(Quaternion q)
        {
            m_orientation = q;
            m_orientation.normalise();
        }

        void setInertiaTensor(const Matrix3& inertiaTensor);
        const Matrix3& getInverseInertiaTensor() const { return m_inverseInertiaTensor; }

        // The inverse inertia tensor turned into world space by the current orientation
        Matrix3 getInverseInertiaTensorWorld() const;

        // Built from the position and orientation each time it's asked for,
        // instead of being stored, so every body is 48 bytes smaller.
        Matrix4 getTransformationMatrix() const;

        Vector3 getPointInWorldSpace(const Vector3& localPoint) const;
        Vector3 getPointInLocalSpace(const Vector3& worldPoint) const;
        Vector3 getDirectionInWorldSpace(const Vector3& localDirection) const;
        Vector3 getDirectionInLocalSpace(const Vector3& worldDirection) const;

        void integrate(real dt);

        // With allowSleep false the body still tracks its motion but never falls asleep by itself; World uses this
        // so touching bodies sleep together
        void integrate(real dt, bool allowSleep);

        // Sets the mass by storing its inverse. A mass of 0 is treated as
        // infinite (immovable): the inverse mass becomes 0 so no force moves it.
        void setMass(real mass);

        // The inverse of the mass; zero means infinite mass.
        real getInverseMass() const { return m_inverseMass; }

        // Mass in kg. Infinite mass (inverse mass 0) reports REAL_MAX.
        real getMass() const { return (m_inverseMass <= 0) ? REAL_MAX : ((real)1.0) / m_inverseMass; }
        bool hasFiniteMass() const { return m_inverseMass > 0; }

        // Adds the given force to the accumulator, to be applied on the next
        // integrate() and then cleared.
        void addForce(Vector3 force);

        // Adds the given force to the torque accumulator, to be applied on the next integrate() and then cleared.
        // Point is in worldspace.
        void addForceAtPoint(Vector3 force, Vector3 point);

        // The same with the point in the body's own space, such as a wing tip or a thruster
        void addForceAtBodyPoint(Vector3 force, Vector3 localPoint);
        void addTorque(Vector3 torque);

        // The acceleration the last integrate() used, gravity plus forces. Resting contacts read it.
        Vector3 getLastFrameAcceleration() const { return m_lastFrameAcceleration; }

        bool isAwake() const { return m_isAwake; }
        // Putting a body to sleep stops it dead; waking it gives it a moment before it can sleep again
        void setAwake(bool awake = true);
        bool canSleep() const { return m_canSleep; }
        void setCanSleep(bool canSleep);
        real getMotion() const { return m_motion; }
        bool wantsToSleep() const { return m_canSleep && m_motion < sleepEpsilon; }

    private:
        Vector3 m_position = Vector3(0.0f, 0.0f, 0.0f);
        Vector3 m_linearVelocity = Vector3(0.0f, 0.0f, 0.0f);
        Vector3 m_acceleration = Vector3(0.0f, -9.81f, 0.0f);
        Vector3 m_lastFrameAcceleration = Vector3(0.0f, 0.0f, 0.0f);
        real m_linearDamping = 0.99f;
        real m_inverseMass = 1.0f; // The inverse of the mass, one by default; zero means infinite mass.
        Vector3 m_forceAccum = Vector3(0.0f, 0.0f, 0.0f); // Sum of all forces applied since the last integrate().
        Quaternion m_orientation = Quaternion(1.0f, 0.0f, 0.0f, 0.0f); // No rotation
        Vector3 m_angularVelocity = Vector3(0.0f, 0.0f, 0.0f);
        Vector3 m_torqueAccum = Vector3(0.0f, 0.0f, 0.0f);
        Matrix3 m_inverseInertiaTensor = Matrix3();
        real m_angularDamping = 0.99f;
        real m_motion = 0.6f; // Twice the default sleep epsilon, so a new body doesn't sleep straight away
        bool m_isAwake = true;
        bool m_canSleep = true;

        // Clears both forceAccum and torqueAccum
        void clearAccumulators();
        void wakeIfAsleep();
    };
}
