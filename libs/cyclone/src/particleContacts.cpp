#include "particleContacts.h"

using namespace cyclone;

void ParticleContact::resolve(real duration)
{
    resolveVelocity(duration);
    resolveInterpenetration(duration);
}

real ParticleContact::calculateSeparatingVelocity() const
{
    Vector3 relativeVelocity = particle[0]->getVelocity();
    if (particle[1] != nullptr)
    {
        relativeVelocity -= particle[1]->getVelocity();
    }
    return relativeVelocity * contactNormal;
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
    Vector3 accCausedVelocity = particle[0]->getAcceleration();
    if (particle[1] != nullptr)
    {
        accCausedVelocity -= particle[1]->getAcceleration();
    }
    const real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;
    if (accCausedSepVelocity < 0)
    {
        newSepVelocity += restitution * accCausedSepVelocity;
        if (newSepVelocity < 0)
        {
            newSepVelocity = 0;
        }
    }

    const real deltaVelocity = newSepVelocity - separatingVelocity;
    real totalInverseMass = particle[0]->getInverseMass();
    if (particle[1] != nullptr)
    {
        totalInverseMass += particle[1]->getInverseMass();
    }
    if (totalInverseMass <= 0)
    {
        return;
    }

    const Vector3 impulsePerIMass = contactNormal * (deltaVelocity / totalInverseMass);
    particle[0]->setVelocity(particle[0]->getVelocity() + impulsePerIMass * particle[0]->getInverseMass());
    if (particle[1] != nullptr)
    {
        particle[1]->setVelocity(particle[1]->getVelocity() + impulsePerIMass * -particle[1]->getInverseMass());
    }
}

void ParticleContact::resolveInterpenetration(real)
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
    const Vector3 movePerIMass = contactNormal * (penetration / totalInverseMass);
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

        // Moving those particles changed how deep every other contact involving them is
        const Vector3* move = resolved.particleMovement;
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
                if (other.particle[b] == resolved.particle[0])
                {
                    other.penetration += sign * (move[0] * other.contactNormal);
                }
                else if (other.particle[b] == resolved.particle[1])
                {
                    other.penetration += sign * (move[1] * other.contactNormal);
                }
            }
        }
        iterationsUsed++;
    }
}

real ParticleLink::currentLength() const
{
    return (particle[0]->getPosition() - particle[1]->getPosition()).magnitude();
}

unsigned ParticleCable::addContact(ParticleContact* contact, unsigned limit) const
{
    const real length = currentLength();
    if (limit == 0 || length < maxLength)
    {
        return 0;
    }
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];
    // Pulls the ends back together, so the normal points from the first toward the second
    contact->contactNormal = (particle[1]->getPosition() - particle[0]->getPosition()).unit();
    contact->penetration = length - maxLength;
    contact->restitution = restitution;
    return 1;
}

unsigned ParticleRod::addContact(ParticleContact* contact, unsigned limit) const
{
    const real current = currentLength();
    if (limit == 0 || current == length)
    {
        return 0;
    }
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];
    Vector3 normal = (particle[1]->getPosition() - particle[0]->getPosition()).unit();
    if (current > length)
    {
        contact->contactNormal = normal;
        contact->penetration = current - length;
    }
    else
    {
        contact->contactNormal = normal * -1;
        contact->penetration = length - current;
    }
    contact->restitution = 0; // A rod never bounces
    return 1;
}

real ParticleConstraint::currentLength() const
{
    return (particle->getPosition() - anchor).magnitude();
}

unsigned ParticleCableConstraint::addContact(ParticleContact* contact, unsigned limit) const
{
    const real length = currentLength();
    if (limit == 0 || length < maxLength)
    {
        return 0;
    }
    contact->particle[0] = particle;
    contact->particle[1] = nullptr;
    contact->contactNormal = (anchor - particle->getPosition()).unit();
    contact->penetration = length - maxLength;
    contact->restitution = restitution;
    return 1;
}

unsigned ParticleRodConstraint::addContact(ParticleContact* contact, unsigned limit) const
{
    const real current = currentLength();
    if (limit == 0 || current == length)
    {
        return 0;
    }
    contact->particle[0] = particle;
    contact->particle[1] = nullptr;
    Vector3 normal = (anchor - particle->getPosition()).unit();
    if (current > length)
    {
        contact->contactNormal = normal;
        contact->penetration = current - length;
    }
    else
    {
        contact->contactNormal = normal * -1;
        contact->penetration = length - current;
    }
    contact->restitution = 0;
    return 1;
}

unsigned ParticleGroundContacts::addContact(ParticleContact* contact, unsigned limit) const
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
        const real y = pParticle->getPosition().y;
        if (y < height)
        {
            contact->contactNormal = Vector3(0, 1, 0);
            contact->particle[0] = pParticle;
            contact->particle[1] = nullptr;
            contact->penetration = height - y;
            contact->restitution = restitution;
            contact++;
            count++;
        }
    }
    return count;
}
