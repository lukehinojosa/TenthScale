#include "particleForces.h"

#include <algorithm>

using namespace cyclone2d;

namespace
{
    Vector2 springForce(const Vector2& position, const Vector2& velocity, const Vector2& otherPosition, const Vector2& otherVelocity,
                        real springConstant, real restLength, real damping, bool isBungee)
    {
        const Vector2 delta = position - otherPosition;
        const real length = delta.magnitude();
        if (length <= 0 || (isBungee && length <= restLength))
        {
            return Vector2();
        }
        const Vector2 direction = delta * (1 / length);
        const real separatingSpeed = (velocity - otherVelocity).dot(direction);
        return direction * (-springConstant * (length - restLength) - damping * separatingSpeed);
    }
}

void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator* generator)
{
    registrations.push_back(Registration{ particle, generator });
}

void ParticleForceRegistry::remove(Particle* particle, ParticleForceGenerator* generator)
{
    auto found = std::find_if(registrations.begin(), registrations.end(),
                              [&](const Registration& r) { return r.particle == particle && r.generator == generator; });
    if (found != registrations.end())
    {
        registrations.erase(found);
    }
}

void ParticleForceRegistry::updateForces(real duration)
{
    for (const Registration& r : registrations)
    {
        r.generator->updateForce(r.particle, duration);
    }
}

void ParticleGravity::updateForce(Particle* particle, real)
{
    if (!particle->hasFiniteMass())
    {
        return;
    }
    particle->addForce(gravity * particle->getMass());
}

void ParticleDrag::updateForce(Particle* particle, real)
{
    const Vector2 velocity = particle->getVelocity();
    const real speed = velocity.magnitude();
    if (speed <= 0)
    {
        return;
    }
    const real drag = k1 * speed + k2 * speed * speed;
    particle->addForce(velocity * (-drag / speed));
}

void ParticleSpring::updateForce(Particle* particle, real)
{
    particle->addForce(springForce(particle->getPosition(), particle->getVelocity(), other->getPosition(), other->getVelocity(),
                                   springConstant, restLength, damping, isBungee));
}

void ParticleAnchoredSpring::updateForce(Particle* particle, real)
{
    particle->addForce(springForce(particle->getPosition(), particle->getVelocity(), *anchor, Vector2(), springConstant, restLength,
                                   damping, isBungee));
}

void ParticleBuoyancy::updateForce(Particle* particle, real)
{
    const real depth = particle->getPosition().y;
    if (depth >= waterHeight + maxDepth)
    {
        return;
    }

    // Rises in a straight line from nothing at maxDepth above the surface to fully under at maxDepth below it
    real submerged = (waterHeight + maxDepth - depth) / (2 * maxDepth);
    submerged = std::min(submerged, (real)1);
    particle->addForce(Vector2(0, liquidDensity * volume * gravity * submerged));
}
