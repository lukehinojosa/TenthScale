#pragma once

#include <vector>

#include "core.h"
#include "particle.h"

namespace cyclone
{
    // Two particles in contact, or one against immovable scenery when particle[1] is null (chapter 7).
    class ParticleContact
    {
    public:
        Particle* particle[2] = { nullptr, nullptr };
        real restitution = 0;
        Vector3 contactNormal; // From particle[1] toward particle[0]
        real penetration = 0; // Negative when the particles are apart

        // Filled in by resolveInterpenetration(), so the resolver can update other contacts
        Vector3 particleMovement[2];

        void resolve(real duration);
        real calculateSeparatingVelocity() const;

    private:
        friend class ParticleContactResolver;

        void resolveVelocity(real duration);
        void resolveInterpenetration(real duration);
    };

    // Resolves the most severe contact first, then updates the others, until all are fixed or the iterations run out
    class ParticleContactResolver
    {
    public:
        explicit ParticleContactResolver(unsigned iterations = 0) : iterations(iterations) {}

        void setIterations(unsigned value) { iterations = value; }
        unsigned getIterationsUsed() const { return iterationsUsed; }

        // Iterations of 0 means twice the contact count
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

    // Two particles joined by a link
    class ParticleLink : public ParticleContactGenerator
    {
    public:
        Particle* particle[2] = { nullptr, nullptr };

    protected:
        real currentLength() const;
    };

    // Stops two particles separating further than maxLength, bouncing back with restitution
    class ParticleCable : public ParticleLink
    {
    public:
        real maxLength = 1;
        real restitution = 0;

        unsigned addContact(ParticleContact* contact, unsigned limit) const override;
    };

    // Keeps two particles exactly length apart
    class ParticleRod : public ParticleLink
    {
    public:
        real length = 1;

        unsigned addContact(ParticleContact* contact, unsigned limit) const override;
    };

    // A particle linked to a fixed point in space
    class ParticleConstraint : public ParticleContactGenerator
    {
    public:
        Particle* particle = nullptr;
        Vector3 anchor;

    protected:
        real currentLength() const;
    };

    class ParticleCableConstraint : public ParticleConstraint
    {
    public:
        real maxLength = 1;
        real restitution = 0;

        unsigned addContact(ParticleContact* contact, unsigned limit) const override;
    };

    class ParticleRodConstraint : public ParticleConstraint
    {
    public:
        real length = 1;

        unsigned addContact(ParticleContact* contact, unsigned limit) const override;
    };

    // Keeps a set of particles above a horizontal ground plane
    class ParticleGroundContacts : public ParticleContactGenerator
    {
    public:
        std::vector<Particle*>* pParticles = nullptr;
        real height = 0;
        real restitution = (real)0.2;

        unsigned addContact(ParticleContact* contact, unsigned limit) const override;
    };
}
