#include "particle.h"

using namespace cyclone2d;

void Particle::integrate(real dt)
{
    if (m_inverseMass <= 0 || dt <= 0)
    {
        clearAccumulator();
        return;
    }

    m_position.addScaledVector(m_velocity, dt);

    Vector2 resultingAcceleration = m_acceleration;
    resultingAcceleration.addScaledVector(m_forceAccum, m_inverseMass);
    m_velocity.addScaledVector(resultingAcceleration, dt);

    // Raised to the time step, so the damping doesn't depend on how often this runs
    m_velocity *= real_pow(m_damping, dt);

    clearAccumulator();
}

void Particle::setMass(real mass)
{
    m_inverseMass = mass <= 0 ? 0 : ((real)1.0) / mass;
}
