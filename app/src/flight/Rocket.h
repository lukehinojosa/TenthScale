#pragma once

#include "shared/SolarSystem.h"

namespace tenth
{
    // The rocket's flight state, for now a point mass under gravity. Position and velocity are kept relative to the
    // body whose gravity it's in, so they stay precise and the rocket moves along with that body.
    class Rocket
    {
    public:
        // Places the rocket relative to a body, such as on Earth's surface or in orbit around it
        void place(BodyId reference, const Vec3d& position, const Vec3d& velocity);

        // One fixed physics step: switches reference body on crossing a sphere of influence, then applies gravity
        void step(double dt, const SolarSystem& system, double time);

        BodyId reference() const { return mReference; }

        const Vec3d& position() const { return mPosition; }

        const Vec3d& velocity() const { return mVelocity; }

        Vec3d absolutePosition(const SolarSystem& system, double time) const;
        Vec3d absoluteVelocity(const SolarSystem& system, double time) const;

    private:
        void updateReference(const SolarSystem& system, double time);

        BodyId mReference = NO_BODY;
        Vec3d mPosition;
        Vec3d mVelocity;
    };
}
