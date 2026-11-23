#include "particleContacts.h"

using namespace cyclone2d;

namespace
{
    // A contact that pulls the particle toward the target past length, and pushes it away inside length when pushes is set
    unsigned linkContact(ParticleContact* contact, Particle* a, Particle* b, const Vector2& target, real length, bool pushes,
                         real restitution)
    {
        const Vector2 toTarget = target - a->getPosition();
        const real current = toTarget.magnitude();
        if (current == length || (!pushes && current < length) || current <= 0)
        {
            return 0;
        }
        const Vector2 normal = toTarget * (1 / current);
        contact->particle[0] = a;
        contact->particle[1] = b;
        if (current > length)
        {
            contact->contactNormal = normal;
            contact->penetration = current - length;
        }
        else
        {
            contact->contactNormal = -normal;
            contact->penetration = length - current;
        }
        contact->restitution = restitution;
        return 1;
    }
}

void ParticleContact::resolve(real duration)
{
    resolveVelocity(duration);
    resolveInterpenetration();
}

real ParticleContact::calculateSeparatingVelocity() const
{
    Vector2 relativeVelocity = particle[0]->getVelocity();
    if (particle[1] != nullptr)
    {
        relativeVelocity -= particle[1]->getVelocity();
    }
    return relativeVelocity.dot(contactNormal);
}

void ParticleContact::resolveVelocity(real duration)
{
    const real separatingVelocity = calculateSeparatingVelocity();
    if (separatingVelocity > 0)
    {
        return;
    }

    real newSepVelocity = -separatingVelocity * restitution;

    // Velocity built up by this frame's acceleration alone is a resting contact, not a bounce
    Vector2 accCausedVelocity = particle[0]->getAcceleration();
    if (particle[1] != nullptr)
    {
        accCausedVelocity -= particle[1]->getAcceleration();
    }
    const real accCausedSepVelocity = accCausedVelocity.dot(contactNormal) * duration;
    if (accCausedSepVelocity < 0)
    {
        newSepVelocity += restitution * accCausedSepVelocity;
        if (newSepVelocity < 0)
        {
            newSepVelocity = 0;
        }
    }

    real totalInverseMass = particle[0]->getInverseMass();
    if (particle[1] != nullptr)
    {
        totalInverseMass += particle[1]->getInverseMass();
    }
    if (totalInverseMass <= 0)
    {
        return;
    }

    const Vector2 impulsePerIMass = contactNormal * ((newSepVelocity - separatingVelocity) / totalInverseMass);
    particle[0]->setVelocity(particle[0]->getVelocity() + impulsePerIMass * particle[0]->getInverseMass());
    if (particle[1] != nullptr)
    {
        particle[1]->setVelocity(particle[1]->getVelocity() - impulsePerIMass * particle[1]->getInverseMass());
    }
}

void ParticleContact::resolveInterpenetration()
{
    particleMovement[0].clear();
    particleMovement[1].clear();
    if (penetration <= 0)
    {
        return;
    }

    real totalInverseMass = particle[0]->getInverseMass();
    if (particle[1] != nullptr)
    {
        totalInverseMass += particle[1]->getInverseMass();
    }
    if (totalInverseMass <= 0)
    {
        return;
    }

    // Each particle moves in proportion to its inverse mass
    const Vector2 movePerIMass = contactNormal * (penetration / totalInverseMass);
    particleMovement[0] = movePerIMass * particle[0]->getInverseMass();
    particle[0]->setPosition(particle[0]->getPosition() + particleMovement[0]);
    if (particle[1] != nullptr)
    {
        particleMovement[1] = movePerIMass * -particle[1]->getInverseMass();
        particle[1]->setPosition(particle[1]->getPosition() + particleMovement[1]);
    }
}

void ParticleContactResolver::resolveContacts(ParticleContact* contactArray, unsigned numContacts, real duration)
{
    const unsigned maxIterations = iterations > 0 ? iterations : numContacts * 2;
    iterationsUsed = 0;
    while (iterationsUsed < maxIterations)
    {
        // The contact closing fastest, or penetrating, goes first
        real max = REAL_MAX;
        unsigned maxIndex = numContacts;
        for (unsigned i = 0; i < numContacts; i++)
        {
            const real sepVel = contactArray[i].calculateSeparatingVelocity();
            if (sepVel < max && (sepVel < 0 || contactArray[i].penetration > 0))
            {
                max = sepVel;
                maxIndex = i;
            }
        }
        if (maxIndex == numContacts)
        {
            break;
        }

        ParticleContact& resolved = contactArray[maxIndex];
        resolved.resolve(duration);

        // Moving those particles changed how deep every contact involving them is, this one included
        const Vector2 move[2] = { resolved.particleMovement[0], resolved.particleMovement[1] };
        Particle* const moved[2] = { resolved.particle[0], resolved.particle[1] };
        for (unsigned i = 0; i < numContacts; i++)
        {
            ParticleContact& other = contactArray[i];
            for (int b = 0; b < 2; b++)
            {
                if (other.particle[b] == nullptr)
                {
                    continue;
                }
                const real sign = b == 0 ? (real)-1 : (real)1;
                for (int m = 0; m < 2; m++)
                {
                    if (other.particle[b] == moved[m])
                    {
                        other.penetration += sign * move[m].dot(other.contactNormal);
                    }
                }
            }
        }
        iterationsUsed++;
    }
}

unsigned ParticleCable::addContact(ParticleContact* contact, unsigned limit) const
{
    if (limit == 0)
    {
        return 0;
    }
    return linkContact(contact, particle[0], particle[1], particle[1]->getPosition(), maxLength, false, restitution);
}

unsigned ParticleRod::addContact(ParticleContact* contact, unsigned limit) const
{
    if (limit == 0)
    {
        return 0;
    }
    return linkContact(contact, particle[0], particle[1], particle[1]->getPosition(), length, true, 0);
}

unsigned ParticleCableConstraint::addContact(ParticleContact* contact, unsigned limit) const
{
    if (limit == 0)
    {
        return 0;
    }
    return linkContact(contact, particle, nullptr, anchor, maxLength, false, restitution);
}

unsigned ParticleRodConstraint::addContact(ParticleContact* contact, unsigned limit) const
{
    if (limit == 0)
    {
        return 0;
    }
    return linkContact(contact, particle, nullptr, anchor, length, true, 0);
}

unsigned ParticleLineContacts::addContact(ParticleContact* contact, unsigned limit) const
{
    if (pParticles == nullptr)
    {
        return 0;
    }
    unsigned count = 0;
    for (Particle* pParticle : *pParticles)
    {
        if (count >= limit)
        {
            break;
        }
        const real distance = pParticle->getPosition().dot(normal) - offset;
        if (distance < 0)
        {
            contact->contactNormal = normal;
            contact->particle[0] = pParticle;
            contact->particle[1] = nullptr;
            contact->penetration = -distance;
            contact->restitution = restitution;
            contact++;
            count++;
        }
    }
    return count;
}
