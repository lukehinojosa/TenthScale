#include "flight/Rocket.h"

namespace tenth
{
    void Rocket::place(BodyId reference, const Vec3d& position, const Vec3d& velocity)
    {
        mReference = reference;
        mPosition = position;
        mVelocity = velocity;
    }

    Vec3d Rocket::absolutePosition(const SolarSystem& system, double time) const
    {
        return system.positionAt(mReference, time) + mPosition;
    }

    Vec3d Rocket::absoluteVelocity(const SolarSystem& system, double time) const
    {
        return system.velocityAt(mReference, time) + mVelocity;
    }

    void Rocket::updateReference(const SolarSystem& system, double time)
    {
        const Vec3d absolute = absolutePosition(system, time);
        const BodyId dominant = system.dominantBody(absolute, time);
        if (dominant == mReference)
        {
            return;
        }
        // Re-expressed against the new body, so the rocket's true position and velocity don't jump
        const Vec3d absoluteVel = absoluteVelocity(system, time);
        mReference = dominant;
        mPosition = absolute - system.positionAt(mReference, time);
        mVelocity = absoluteVel - system.velocityAt(mReference, time);
    }

    void Rocket::step(double dt, const SolarSystem& system, double time)
    {
        updateReference(system, time);

        const double distanceSquared = mPosition.lengthSquared();
        if (distanceSquared > 0.0)
        {
            const double mu = system.body(mReference).mu;
            const Vec3d gravity = mPosition.normalized() * (-mu / distanceSquared);
            mVelocity += gravity * dt; // velocity first, then position (semi-implicit Euler)
        }
        mPosition += mVelocity * dt;
    }
}
