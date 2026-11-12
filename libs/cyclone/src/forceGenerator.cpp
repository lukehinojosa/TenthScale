#include "forceGenerator.h"

#include <cmath>

using namespace cyclone;

Vector3 ForceGenerator::AddSpringForce(Vector3 position1, Vector3 position2, real restLength, real stiffness, real damping,
                                       Vector3 velocity1, Vector3 velocity2, bool isBungee)
{
    // Vector from particle 2 to particle 1.
    Vector3 delta = position1 - position2;
    real length = delta.magnitude();

    // A bungee only pulls when stretched past its rest length; it never pushes
    // when compressed. If it isn't extended, there is no force.
    if (isBungee && length <= restLength)
    {
        return Vector3(0, 0, 0);
    }

    // Unit vector along the spring, pointing toward particle 1.
    Vector3 direction = delta.unit();

    // Hooke's law: restoring force pulls particle 1 toward its rest position.
    Vector3 springForce = direction * (-stiffness * (length - restLength));

    // Damping opposes the relative velocity of particle 1 with respect to particle 2.
    Vector3 dampingForce = (velocity1 - velocity2) * -damping;

    return springForce + dampingForce;
}

Vector3 ForceGenerator::AddBuoyancyForce(Vector3 position, real fluidHeight, real liquidDensity, real volume, real objectHeight)
{
    // How far the object's reference point sits below the fluid surface.
    real depth = fluidHeight - position.y;

    // Above the surface: no part submerged, so no buoyancy.
    if (depth <= 0)
    {
        return Vector3(0, 0, 0);
    }

    // Fraction of the object submerged, moves from 0 to 1 across the object's height.
    real submersion = depth / objectHeight;
    if (submersion > 1)
    {
        submersion = 1;
    }

    // Buoyant force acts straight up, scaled by how much of the object is submerged.
    return Vector3(0, liquidDensity * volume * submersion, 0);
}

Vector3 ForceGenerator::AddDragForce(Vector3 position, Vector3 velocity, real linearCoefficient, real exponentialCoefficient,
                                     bool useMaxHeight, real maxHeight)
{
    // When gated by height (e.g. only drag while submerged), skip if above maxHeight.
    if (useMaxHeight && position.y > maxHeight)
    {
        return Vector3(0, 0, 0);
    }

    real speed = velocity.magnitude();

    // No motion means no drag (and avoids normalising a zero vector).
    if (speed <= 0)
    {
        return Vector3(0, 0, 0);
    }

    // Drag magnitude grows with a linear and a quadratic term in speed.
    real dragMagnitude = linearCoefficient * speed + exponentialCoefficient * speed * speed;

    // Drag opposes the direction of motion.
    return velocity.unit() * -dragMagnitude;
}

Vector3 ForceGenerator::AddAttractRepulseForce(Vector3 position1, Vector3 position2, real constant)
{
    // Vector from the object toward the attractor/repulsor.
    Vector3 delta = position2 - position1;
    real distance = delta.magnitude();

    // Coincident points have no defined direction and an infinite force; skip.
    if (distance <= 0)
    {
        return Vector3(0, 0, 0);
    }

    // Stronger the closer they are, toward the other point. A positive constant attracts and a negative one repels.
    real magnitude = constant / (distance * distance);
    return delta.unit() * magnitude;
}

/// ParticleForceRegistry

void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator* fg)
{
    ParticleForceRegistration registration;
    registration.particle = particle;
    registration.fg = fg;
    registrations.push_back(registration);
}

void ParticleForceRegistry::remove(Particle* particle, ParticleForceGenerator* fg)
{
    for (auto it = registrations.begin(); it != registrations.end(); ++it)
    {
        if (it->particle == particle && it->fg == fg)
        {
            registrations.erase(it);
            return;
        }
    }
}

void ParticleForceRegistry::clear()
{
    registrations.clear();
}

void ParticleForceRegistry::updateForces(real duration)
{
    for (auto& registration : registrations)
    {
        registration.fg->updateForce(registration.particle, duration);
    }
}

/// Concrete force generators

ParticleSpring::ParticleSpring(Particle* other, real restLength, real stiffness, real damping, bool isBungee)
    : other(other), restLength(restLength), stiffness(stiffness), damping(damping), isBungee(isBungee)
{
}

void ParticleSpring::updateForce(Particle* particle, real duration)
{
    Vector3 force = ForceGenerator::AddSpringForce(particle->getPosition(), other->getPosition(), restLength, stiffness, damping,
                                                   particle->getVelocity(), other->getVelocity(), isBungee);
    particle->addForce(force);
}

ParticleBuoyancy::ParticleBuoyancy(real fluidHeight, real liquidDensity, real volume, real objectHeight)
    : fluidHeight(fluidHeight), liquidDensity(liquidDensity), volume(volume), objectHeight(objectHeight)
{
}

void ParticleBuoyancy::updateForce(Particle* particle, real duration)
{
    Vector3 force = ForceGenerator::AddBuoyancyForce(particle->getPosition(), fluidHeight, liquidDensity, volume, objectHeight);
    particle->addForce(force);
}

ParticleDrag::ParticleDrag(real linearCoefficient, real exponentialCoefficient, bool useMaxHeight, real maxHeight)
    : linearCoefficient(linearCoefficient), exponentialCoefficient(exponentialCoefficient), useMaxHeight(useMaxHeight), maxHeight(maxHeight)
{
}

void ParticleDrag::updateForce(Particle* particle, real duration)
{
    Vector3 force = ForceGenerator::AddDragForce(particle->getPosition(), particle->getVelocity(), linearCoefficient,
                                                 exponentialCoefficient, useMaxHeight, maxHeight);

    // Limit the drag so it can only slow the motion, never flip it.
    if (duration > 0)
    {
        real maxMagnitude = particle->getMass() * particle->getVelocity().magnitude() / duration;
        real magnitude = force.magnitude();
        if (magnitude > maxMagnitude)
        {
            force = force * (maxMagnitude / magnitude);
        }
    }

    particle->addForce(force);
}

ParticleAttractRepulse::ParticleAttractRepulse(Particle* other, real constant) : other(other), constant(constant)
{
}

void ParticleAttractRepulse::updateForce(Particle* particle, real duration)
{
    Vector3 force = ForceGenerator::AddAttractRepulseForce(particle->getPosition(), other->getPosition(), constant);
    particle->addForce(force);
}

void ParticleGravity::updateForce(Particle* particle, real)
{
    if (particle->getInverseMass() <= 0)
    {
        return;
    }
    particle->addForce(gravity * particle->getMass());
}

ParticleAnchoredSpring::ParticleAnchoredSpring(const Vector3* anchor, real restLength, real stiffness, real damping, bool isBungee)
    : anchor(anchor), restLength(restLength), stiffness(stiffness), damping(damping), isBungee(isBungee)
{
}

void ParticleAnchoredSpring::updateForce(Particle* particle, real)
{
    Vector3 force = ForceGenerator::AddSpringForce(particle->getPosition(), *anchor, restLength, stiffness, damping,
                                                   particle->getVelocity(), Vector3(0, 0, 0), isBungee);
    particle->addForce(force);
}

ParticleFakeSpring::ParticleFakeSpring(const Vector3* anchor, real springConstant, real damping)
    : anchor(anchor), springConstant(springConstant), damping(damping)
{
}

void ParticleFakeSpring::updateForce(Particle* particle, real duration)
{
    if (particle->getInverseMass() <= 0 || duration <= 0)
    {
        return;
    }
    const real gammaSquared = 4 * springConstant - damping * damping;
    if (gammaSquared <= 0)
    {
        return;
    }
    const real gamma = (real)0.5 * real_sqrt(gammaSquared);

    // Where the exact spring motion puts the particle after this step
    const Vector3 position = particle->getPosition() - *anchor;
    const Vector3 c = position * (damping / (2 * gamma)) + particle->getVelocity() * (1 / gamma);
    Vector3 target = position * real_cos(gamma * duration) + c * real_sin(gamma * duration);
    target *= real_exp((real)-0.5 * duration * damping);

    // The acceleration that lands it there in one step
    const Vector3 acceleration = (target - position) * (1 / (duration * duration)) - particle->getVelocity() * (1 / duration);
    particle->addForce(acceleration * particle->getMass());
}
