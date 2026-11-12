#include "particle.h"

using namespace cyclone;

void Particle::integrate(real dt)
{
    // Update position from the current velocity.
    setPosition(getPosition() + getVelocity() * dt);

    // Start from the constant acceleration (e.g. gravity), then add the
    // acceleration produced by all accumulated forces: a = F * (1/mass).
    Vector3 resultingAcceleration = getAcceleration();
    resultingAcceleration += forceAccum * inverseMass;

    // Update velocity from the resulting acceleration.
    Vector3 newVelocity = getVelocity() + resultingAcceleration * dt;

    // Apply drag/damping to the velocity, raised to the timestep so the
    // amount of drag is independent of the size of the timestep.
    newVelocity.componentProductUpdate(rPowVector3(getDamping(), dt));
    setVelocity(newVelocity);

    // Forces only last one frame; clear them for the next integration.
    clearAccumulator();
}

void Particle::setMass(real mass)
{
    // A mass of 0 represents an immovable object (infinite mass), so its
    // inverse mass is 0 and accumulated forces produce no acceleration.
    if (mass <= 0)
    {
        inverseMass = 0;
    }
    else
    {
        inverseMass = ((real)1.0) / mass;
    }
}

void Particle::addForce(Vector3 force)
{
    forceAccum += force;
}

void Particle::clearAccumulator()
{
    forceAccum.clear();
}
