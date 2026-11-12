#include "particleWorld.h"

using namespace cyclone;

ParticleWorld::ParticleWorld(unsigned maxContacts, unsigned iterations)
    : resolver(iterations), contacts(maxContacts > 0 ? maxContacts : 1), autoIterations(iterations == 0)
{
}

unsigned ParticleWorld::generateContacts()
{
    unsigned limit = (unsigned)contacts.size();
    ParticleContact* next = contacts.data();
    for (ParticleContactGenerator* pGenerator : contactGenerators)
    {
        const unsigned used = pGenerator->addContact(next, limit);
        limit -= used;
        next += used;
        if (limit == 0)
        {
            break;
        }
    }
    return (unsigned)contacts.size() - limit;
}

void ParticleWorld::integrate(real duration)
{
    for (Particle* pParticle : particles)
    {
        pParticle->integrate(duration);
    }
}

void ParticleWorld::runPhysics(real duration)
{
    forces.updateForces(duration);
    integrate(duration);
    contactCount = generateContacts();
    if (contactCount > 0)
    {
        if (autoIterations)
        {
            resolver.setIterations(contactCount * 2);
        }
        resolver.resolveContacts(contacts.data(), contactCount, duration);
    }
}
