#pragma once

#include <vector>

#include "core.h"
#include "particle.h"

namespace cyclone2d
{
    // Two particles in contact, or one against immovable scenery when particle[1] is null (chapter 7)
    class ParticleContact
    {
    public:
        Particle* particle[2] = { nullptr, nullptr };
        real restitution = 0;
        Vector2 contactNormal; // From particle[1] toward particle[0]
        real penetration = 0;

        // Filled in by resolveInterpenetration(), so the resolver can update other contacts
        Vector2 particleMovement[2];

        void resolve(real duration);
        real calculateSeparatingVelocity() const;

    private:
        friend class ParticleContactResolver;

        void resolveVelocity(real duration);
        void resolveInterpenetration();
    };

    // Resolves the contact closing fastest first, then updates the others, until all are fixed or the iterations run out
    class ParticleContactResolver
    {
    public:
        explicit ParticleContactResolver(unsigned iterations = 0) : iterations(iterations) {}

        // 0 means twice the contact count
        void setIterations(unsigned value) { iterations = value; }
        unsigned getIterationsUsed() const { return iterationsUsed; }

        void resolveContacts(ParticleContact* contactArray, unsigned numContacts, real duration);

    private:
        unsigned iterations = 0;
        unsigned iterationsUsed = 0;
    };

    // Anything that creates particle contacts each frame: cables, rods, the ground
    class ParticleContactGenerator
    {
    public:
        virtual ~ParticleContactGenerator() = default;

        // Writes up to limit contacts and returns how many it wrote
        virtual unsigned addContact(ParticleContact* contact, unsigned limit) const = 0;
    };

    // Stops two particles separating further than maxLength
    class ParticleCable : public ParticleContactGenerator
    {
    public:
        Particle* particle[2] = { nullptr, nullptr };
        real maxLength = 1;
        real restitution = 0;

        unsigned addContact(ParticleContact* contact, unsigned limit) const override;
    };

    // Keeps two particles exactly length apart
    class ParticleRod : public ParticleContactGenerator
    {
    public:
        Particle* particle[2] = { nullptr, nullptr };
        real length = 1;

        unsigned addContact(ParticleContact* contact, unsigned limit) const override;
    };

    // A cable from a particle to a fixed point
    class ParticleCableConstraint : public ParticleContactGenerator
    {
    public:
        Particle* particle = nullptr;
        Vector2 anchor;
        real maxLength = 1;
        real restitution = 0;

        unsigned addContact(ParticleContact* contact, unsigned limit) const override;
    };

    // A rod from a particle to a fixed point
    class ParticleRodConstraint : public ParticleContactGenerator
    {
    public:
        Particle* particle = nullptr;
        Vector2 anchor;
        real length = 1;

        unsigned addContact(ParticleContact* contact, unsigned limit) const override;
    };

    // Keeps particles on the front of a line, the ground by default
    class ParticleLineContacts : public ParticleContactGenerator
    {
    public:
        std::vector<Particle*>* pParticles = nullptr;
        Vector2 normal = Vector2::UP;
        real offset = 0; // How far the line is from the origin along its normal
        real restitution = (real)0.2;

        unsigned addContact(ParticleContact* contact, unsigned limit) const override;
    };
}
