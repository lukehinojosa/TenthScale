#pragma once
#include "core.h"

namespace cyclone
{
    class Particle
    {
    public:
        Vector3 getPosition() const { return m_position; }
        void setPosition(Vector3 p) { m_position = p; }
        Vector3 getVelocity() const { return m_velocity; }
        void setVelocity(Vector3 v) { m_velocity = v; }
        Vector3 getAcceleration() const { return m_acceleration; }
        void setAcceleration(Vector3 a) { m_acceleration = a; }
        Vector3 getDamping() const { return m_damping; }
        void setDamping(Vector3 d) { m_damping = d; }

        Particle() = default;
        Particle(Vector3 p, Vector3 v, Vector3 a, Vector3 d)
        {
            setPosition(p);
            setVelocity(v);
            setAcceleration(a);
            setDamping(d);
        }

        // Vector3 and real are 16 bytes or less, so they are passed by value
        static Vector3 rPowVector3(Vector3 v, real p) { return Vector3{ real_pow(v.x, p), real_pow(v.y, p), real_pow(v.z, p) }; }
        void integrate(real dt);

        // Sets the mass by storing its inverse. A mass of 0 is treated as
        // infinite (immovable): the inverse mass becomes 0 so no force moves it.
        void setMass(real mass);

        // The inverse of the mass; zero means infinite mass.
        real getInverseMass() const { return inverseMass; }

        // Mass in kg. Infinite mass (inverse mass 0) reports REAL_MAX.
        real getMass() const { return (inverseMass <= 0) ? REAL_MAX : ((real)1.0) / inverseMass; }

        // Adds the given force to the accumulator, to be applied on the next
        // integrate() and then cleared.
        void addForce(Vector3 force);

        // Resets the accumulated force to zero.
        void clearAccumulator();

    private:
        Vector3 m_position = Vector3(0.0f, 0.0f, 0.0f);
        Vector3 m_velocity = Vector3(0.0f, 0.0f, 0.0f);
        Vector3 m_acceleration = Vector3(0.0f, -9.81f, 0.0f);
        Vector3 m_damping = Vector3(0.99f, 0.99f, 0.99f);

        // The inverse of the mass, one by default; zero means infinite mass.
        real inverseMass = 1.0f;

        // Sum of all forces applied since the last integrate().
        Vector3 forceAccum = Vector3(0.0f, 0.0f, 0.0f);
    };
}