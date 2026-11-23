#pragma once

#include "core.h"

namespace cyclone2d
{
    // A point mass that moves but never turns (chapter 3)
    class Particle
    {
    public:
        Particle() = default;
        Particle(Vector2 position, Vector2 velocity, Vector2 acceleration = Vector2::GRAVITY, real damping = (real)0.99)
            : m_position(position), m_velocity(velocity), m_acceleration(acceleration), m_damping(damping)
        {
        }

        Vector2 getPosition() const { return m_position; }
        void setPosition(Vector2 p) { m_position = p; }
        Vector2 getVelocity() const { return m_velocity; }
        void setVelocity(Vector2 v) { m_velocity = v; }
        Vector2 getAcceleration() const { return m_acceleration; }
        void setAcceleration(Vector2 a) { m_acceleration = a; }

        // The share of velocity kept after one second, so 1 means no damping
        real getDamping() const { return m_damping; }
        void setDamping(real d) { m_damping = d; }

        void integrate(real dt);

        // A mass of 0 is infinite: the particle never moves
        void setMass(real mass);
        real getInverseMass() const { return m_inverseMass; }
        void setInverseMass(real inverseMass) { m_inverseMass = inverseMass; }
        real getMass() const { return (m_inverseMass <= 0) ? REAL_MAX : ((real)1.0) / m_inverseMass; }
        bool hasFiniteMass() const { return m_inverseMass > 0; }

        // Forces last until the next integrate()
        void addForce(Vector2 force) { m_forceAccum += force; }
        void clearAccumulator() { m_forceAccum.clear(); }

    private:
        Vector2 m_position;
        Vector2 m_velocity;
        Vector2 m_acceleration = Vector2::GRAVITY;
        real m_damping = (real)0.99;
        real m_inverseMass = 1;
        Vector2 m_forceAccum;
    };
}
