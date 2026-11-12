#pragma once
#include <vector>
#include "core.h"
#include "particle.h"

namespace cyclone
{
    class ForceGenerator
    {
    public:
        static Vector3 AddSpringForce(Vector3 position1, Vector3 position2, real restLength, real stiffness, real damping,
                                      Vector3 velocity1, Vector3 velocity2, bool isBungee = false);

        static Vector3 AddBuoyancyForce(Vector3 position, real fluidHeight, real liquidDensity, real volume, real objectHeight);

        static Vector3 AddDragForce(Vector3 position, Vector3 velocity, real linearCoefficient, real exponentialCoefficient,
                                    bool useMaxHeight = false, real maxHeight = 0);

        static Vector3 AddAttractRepulseForce(Vector3 position1, Vector3 position2, real constant);
    };

    // Base class for anything that pushes on particles; override updateForce().
    class ParticleForceGenerator
    {
    public:
        // Calculates and applies a force to the given particle.
        virtual void updateForce(Particle* particle, real duration) = 0;

        virtual ~ParticleForceGenerator() = default;
    };

    // The force generators and the particles each one applies to.
    class ParticleForceRegistry
    {
    public:
        // Registers the given force generator to apply to the given particle.
        void add(Particle* particle, ParticleForceGenerator* fg);

        // Removes the given registered pair. Does nothing if it is not registered.
        void remove(Particle* particle, ParticleForceGenerator* fg);

        // Clears all registrations. Does not delete the particles or generators.
        void clear();

        // Calls all the force generators to update the forces of their
        // corresponding particles.
        void updateForces(real duration);

    protected:
        // Keeps track of one force generator and the particle it applies to.
        struct ParticleForceRegistration
        {
            Particle* particle;
            ParticleForceGenerator* fg;
        };

        std::vector<ParticleForceRegistration> registrations;
    };

    // A spring, or a bungee, between the owning particle and another.
    class ParticleSpring : public ParticleForceGenerator
    {
    public:
        ParticleSpring(Particle* other, real restLength, real stiffness, real damping, bool isBungee = false);
        void updateForce(Particle* particle, real duration) override;

    private:
        Particle* other;
        real restLength;
        real stiffness;
        real damping;
        bool isBungee;
    };

    // Buoyancy from a flat body of liquid below a given height.
    class ParticleBuoyancy : public ParticleForceGenerator
    {
    public:
        ParticleBuoyancy(real fluidHeight, real liquidDensity, real volume, real objectHeight);
        void updateForce(Particle* particle, real duration) override;

    private:
        real fluidHeight;
        real liquidDensity;
        real volume;
        real objectHeight;
    };

    // Drag that grows with the particle's speed, optionally only below a given height.
    class ParticleDrag : public ParticleForceGenerator
    {
    public:
        ParticleDrag(real linearCoefficient, real exponentialCoefficient, bool useMaxHeight = false, real maxHeight = 0);
        void updateForce(Particle* particle, real duration) override;

    private:
        real linearCoefficient;
        real exponentialCoefficient;
        bool useMaxHeight;
        real maxHeight;
    };

    // Pulls the owning particle toward another, or pushes it away, more strongly the closer they are.
    class ParticleAttractRepulse : public ParticleForceGenerator
    {
    public:
        ParticleAttractRepulse(Particle* other, real constant);
        void updateForce(Particle* particle, real duration) override;

    private:
        Particle* other;
        real constant;
    };

    // A constant gravity force, scaled by mass so every particle falls at the same rate
    class ParticleGravity : public ParticleForceGenerator
    {
    public:
        explicit ParticleGravity(const Vector3& gravity) : gravity(gravity) {}
        void updateForce(Particle* particle, real duration) override;

        Vector3 gravity;
    };

    // A spring (or bungee) from the particle to a point that may move but doesn't feel the force
    class ParticleAnchoredSpring : public ParticleForceGenerator
    {
    public:
        ParticleAnchoredSpring(const Vector3* anchor, real restLength, real stiffness, real damping = 0, bool isBungee = false);
        void updateForce(Particle* particle, real duration) override;

        const Vector3* anchor;
        real restLength;
        real stiffness;
        real damping;
        bool isBungee;
    };

    // A stiff spring that stays stable at any time step by working out where the spring would carry the particle
    // this frame (chapter 6.3.2). Keep the damping low compared with the stiffness.
    class ParticleFakeSpring : public ParticleForceGenerator
    {
    public:
        ParticleFakeSpring(const Vector3* anchor, real springConstant, real damping);
        void updateForce(Particle* particle, real duration) override;

        const Vector3* anchor;
        real springConstant;
        real damping;
    };
}
