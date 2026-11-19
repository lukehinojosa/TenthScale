#pragma once

#include <cyclone/rigidBody.h>
#include <cyclone/rigidContactResolver.h>

#include "flight/Vehicle.h"
#include "shared/SolarSystem.h"

namespace tenth
{
    // The rocket in flight as one rigid body: it moves, turns, and spins under gravity and rests on the ground.
    // Position and velocity are kept relative to the body whose gravity it's in, so they stay precise and the rocket
    // moves along with that body. Its own axes are the design's, +Y up the rocket.
    class Rocket
    {
    public:
        Rocket();

        // Places the rocket relative to a body, standing straight up from it and not spinning
        void place(BodyId reference, const Vec3d& position, const Vec3d& velocity);

        // The built rocket: its parts, staging, mass, and inertia, plus its outline for touching the ground
        void setVehicle(const Vehicle& vehicle);
        const Vehicle& vehicle() const { return mVehicle; }

        // Whether any of it touched the reference body's surface in the last step
        bool onGround() const { return mOnGround; }

        // One fixed physics step: switches reference body on crossing a sphere of influence, applies gravity, moves
        // and turns, then pushes it back out of the ground
        void step(double dt, const SolarSystem& system, double time);

        BodyId reference() const { return mReference; }

        Vec3d position() const; // of the center of mass
        Vec3d velocity() const;

        // How high its lowest point is above the reference body's surface, however it's lying
        double altitude(const SolarSystem& system) const;

        // Turns a direction in the rocket's own axes into the reference body's axes
        Vec3d toWorld(const Vec3d& direction) const;

        // How fast it turns, in radians per second about each of the reference body's axes
        Vec3d spin() const;
        void setSpin(const Vec3d& spin);

        // Leans it over by angle radians about an axis given in the reference body's axes
        void turn(const Vec3d& axis, double angle);

        Vec3d absolutePosition(const SolarSystem& system, double time) const;
        Vec3d absoluteVelocity(const SolarSystem& system, double time) const;

    private:
        void updateReference(const SolarSystem& system, double time);
        void touchGround(double dt, const SolarSystem& system);

        BodyId mReference = NO_BODY;
        cyclone::RigidBody mBody;
        cyclone::RigidContactResolver mResolver;
        std::vector<cyclone::Contact> mContacts;
        Vehicle mVehicle;
        bool mOnGround = false;
    };
}
