#pragma once

#include <vector>

#include "particle.h"
#include "particleContacts.h"
#include "particleForces.h"

namespace cyclone2d
{
    // The mass-aggregate engine (chapter 8): forces, integration, contacts and resolution each frame. It doesn't own
    // its particles or generators, so keep them alive while it runs.
    class ParticleWorld
    {
    public:
        explicit ParticleWorld(unsigned maxContacts = 256, unsigned iterations = 0);

        std::vector<Particle*> particles;
        std::vector<ParticleContactGenerator*> contactGenerators;
        ParticleForceRegistry forces;
        ParticleContactResolver resolver;

        void runPhysics(real duration);
        void integrate(real duration);
        unsigned generateContacts();

        unsigned getContactCount() const { return contactCount; }
        const ParticleContact* getContacts() const { return contacts.data(); }

    private:
        std::vector<ParticleContact> contacts;
        unsigned contactCount = 0;
        bool autoIterations = true;
    };
}
