#pragma once

#include <vector>

#include "core.h"
#include "particle.h"

namespace cyclone2d
{
    // Anything that pushes on particles each frame (chapter 5)
    class ParticleForceGenerator
    {
    public:
        virtual ~ParticleForceGenerator() = default;
        virtual void updateForce(Particle* particle, real duration) = 0;
    };

    // Which generators act on which particles. Doesn't own either.
    class ParticleForceRegistry
    {
    public:
        void add(Particle* particle, ParticleForceGenerator* generator);
        void remove(Particle* particle, ParticleForceGenerator* generator);
        void clear() { registrations.clear(); }
        void updateForces(real duration);

    private:
        struct Registration
        {
            Particle* particle;
            ParticleForceGenerator* generator;
        };
        std::vector<Registration> registrations;
    };

    // Gravity as a force, scaled by mass so everything falls at the same rate
    class ParticleGravity : public ParticleForceGenerator
    {
    public:
        explicit ParticleGravity(const Vector2& gravity = Vector2::GRAVITY) : gravity(gravity) {}
        void updateForce(Particle* particle, real duration) override;

        Vector2 gravity;
    };

    // Drag that grows with speed (k1) and with speed squared (k2)
    class ParticleDrag : public ParticleForceGenerator
    {
    public:
        ParticleDrag(real k1, real k2) : k1(k1), k2(k2) {}
        void updateForce(Particle* particle, real duration) override;

        real k1;
        real k2;
    };

    // A spring to another particle; a bungee only pulls, never pushes
    class ParticleSpring : public ParticleForceGenerator
    {
    public:
        ParticleSpring(Particle* other, real springConstant, real restLength, real damping = 0, bool isBungee = false)
            : other(other), springConstant(springConstant), restLength(restLength), damping(damping), isBungee(isBungee)
        {
        }
        void updateForce(Particle* particle, real duration) override;

        Particle* other;
        real springConstant;
        real restLength;
        real damping;
        bool isBungee;
    };

    // A spring to a point that may move but doesn't feel the force
    class ParticleAnchoredSpring : public ParticleForceGenerator
    {
    public:
        ParticleAnchoredSpring(const Vector2* anchor, real springConstant, real restLength, real damping = 0, bool isBungee = false)
            : anchor(anchor), springConstant(springConstant), restLength(restLength), damping(damping), isBungee(isBungee)
        {
        }
        void updateForce(Particle* particle, real duration) override;

        const Vector2* anchor;
        real springConstant;
        real restLength;
        real damping;
        bool isBungee;
    };

    // Floats a particle in liquid whose surface is at waterHeight. It is fully under maxDepth below the surface.
    class ParticleBuoyancy : public ParticleForceGenerator
    {
    public:
        ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity = 1000, real gravity = (real)9.81)
            : maxDepth(maxDepth), volume(volume), waterHeight(waterHeight), liquidDensity(liquidDensity), gravity(gravity)
        {
        }
        void updateForce(Particle* particle, real duration) override;

        real maxDepth;
        real volume;
        real waterHeight;
        real liquidDensity;
        real gravity;
    };
}
