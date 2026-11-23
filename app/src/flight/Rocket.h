#pragma once

#include <vector>

#include <cyclone/rigidBody.h>
#include <cyclone/rigidContactResolver.h>

#include "flight/LaunchPad.h"
#include "flight/Vehicle.h"
#include "shared/SolarSystem.h"

namespace tenth
{
    // A charge going off where two pieces came apart, for the flight view to show. Positions and velocities are
    // relative to the rocket's reference body.
    struct Separation
    {
        Vec3d site;
        Vec3d velocity; // of the joint just before the charge went off
        Vec3d axis; // the way the pieces were pushed apart
        double size = 1.0; // 1 for a decoupler; nose cones pop with a smaller puff
        Vec3d shift; // how far the rocket's center of mass jumped as the piece left, so a view following it can stay put
    };

    // What the SAS keeps the nose pointed at. Stability holds wherever it was pointing; the rest follow the orbit:
    // along or against the velocity, square to the orbit's plane, or toward or away from the body below.
    enum class SasMode
    {
        Off,
        Stability,
        Prograde,
        Retrograde,
        Normal,
        AntiNormal,
        RadialOut,
        RadialIn,
    };

    // The rocket in flight as one rigid body: it moves, turns, and spins under gravity and its engines, and rests on
    // the ground.
    // Position and velocity are kept relative to the body whose gravity it's in, so they stay precise and the rocket
    // moves along with that body. Its own axes are the design's, +Y up the rocket.
    class Rocket
    {
    public:
        static constexpr double DECOUPLER_SPEED = 5.0; // m/s a decoupler's charge parts the two halves at
        static constexpr double NOSE_CONE_SPEED = 8.0; // the parachute shoves the cone off this fast
        static constexpr double GIMBAL_RATE = 0.2; // radians per second an engine bell swivels

        Rocket();

        // Places the rocket relative to a body, standing straight up from it and not spinning
        void place(BodyId reference, const Vec3d& position, const Vec3d& velocity);

        // A pad it can stand on and land on as well as the ground; pieces that come away know it too
        void setPad(const LaunchPad& pad) { mPad = pad; }

        // The built rocket: its parts, staging, mass, and inertia, plus its outline for touching the ground
        void setVehicle(const Vehicle& vehicle);
        const Vehicle& vehicle() const { return mVehicle; }

        // Shared by every lit engine, 0 to 1
        void setThrottle(double throttle);
        double throttle() const { return mVehicle.throttle; }

        // How hard to turn about each of the rocket's own axes, -1 to 1: pitch about x, roll about y, and yaw about z.
        // The reaction wheels turn it with that share of their torque, so a heavier rocket turns slower. Burning
        // engines also swivel to help pitch and yaw.
        void setSteering(const Vec3d& steering);
        const Vec3d& steering() const { return mPilot; }

        // The SAS steers with the same wheels and engines whenever the pilot isn't, turning no faster than it can
        // stop again. Turning on Stability holds wherever the nose ends up once any spin has died away.
        void setSas(SasMode mode);
        SasMode sas() const { return mSas; }

        // Where a mode would point the nose, in the reference body's axes; false only with the SAS off. Standing
        // still, prograde is up; climbing straight up, normal is square to the way the rocket faces across.
        bool holdDirection(SasMode mode, Vec3d& direction) const;

        // Fires the next stage: lights its engines, opens its parachutes, and lets go of whatever its decouplers
        // hold. Opening a parachute pops off any nose cone still covering it. Returns the parts it fired.
        std::vector<PartIndex> fireNextStage();

        // The pieces that have come away since the last call, each flying on its own now
        std::vector<Rocket> takeDropped();

        // Where pieces have come apart since the last call
        std::vector<Separation> takeSeparations();

        // The engines' total push in the last step, in newtons, and the air pressure they pushed against
        double thrust() const { return mThrust; }
        double pressure() const { return mPressure; }

        // The air's total push on the rocket in the last step, in newtons
        double drag() const { return mDrag; }

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

        // Turns a direction in the rocket's own axes into the reference body's axes, and back
        Vec3d toWorld(const Vec3d& direction) const;
        Vec3d toLocal(const Vec3d& direction) const;

        // How fast it turns, in radians per second about each of the reference body's axes
        Vec3d spin() const;
        void setSpin(const Vec3d& spin);

        // Leans it over by angle radians about an axis given in the reference body's axes
        void turn(const Vec3d& axis, double angle);

        // Pushes two separate rockets apart where any part of one has entered a part of the other, such as a dropped
        // stage catching up with the rocket it came off
        friend void collide(Rocket& a, Rocket& b, double dt);

        Vec3d absolutePosition(const SolarSystem& system, double time) const;
        Vec3d absoluteVelocity(const SolarSystem& system, double time) const;

    private:
        void updateReference(const SolarSystem& system, double time);

        // Follows the path and the orbit's plane for the SAS, easing round where the motion leaves them unclear
        void updateCourse(double dt);
        void pushWithEngines();
        void steerWithSas();

        // Burning engines lean toward the pitch and yaw being asked for, so their thrust turns the rocket the same
        // way; the rest ease back to straight
        void swivelEngines(double dt);
        void turnWithWheels();

        // Every part, and any open parachute, feels the air from where it is, so fins low down steady the rocket.
        // Released parachutes fill a little more each step while there's air, and fold up again on the ground.
        void pushWithAir(double density, double dt);

        // Lets the part and everything attached through it go as a rocket of its own, with its engines shut off.
        // A push from site, in the design's axes, kicks the two apart at this speed; size is how big it looks.
        void splitOff(PartIndex index, const Vec3d& site, double speed, double size);

        // A sudden push at a point, which turns the rocket too unless it's in line with the center of mass
        void kick(const Vec3d& impulse, const Vec3d& site);

        // Nose cones come away at the top of the climb, uncovering the parachute
        void popNoseCones();

        // Hands Cyclone the vehicle's mass and inertia; the body's position stays the center of mass as fuel burns
        void applyMass();
        void touchGround(double dt, const SolarSystem& system);

        // Contacts for each of this rocket's outline points inside one of other's parts
        void addContactsInside(Rocket& other, std::vector<cyclone::Contact>& contacts);

        // How far the furthest outline point reaches from the center of mass
        double reach() const;

        BodyId mReference = NO_BODY;
        cyclone::RigidBody mBody;
        cyclone::RigidContactResolver mResolver;
        std::vector<cyclone::Contact> mContacts;
        Vehicle mVehicle;
        bool mOnGround = false;
        LaunchPad mPad;
        double mThrust = 0.0;
        double mPressure = 0.0;
        double mDrag = 0.0;
        double mClimbRate = 0.0; // last step's, for spotting the top of the climb
        Vec3d mPilot; // what the keys ask for
        SasMode mSas = SasMode::Off;
        Vec3d mHold{ 0.0, 1.0, 0.0 }; // where Stability keeps the nose
        bool mSettling = false; // Stability follows the nose until the pilot's turn has died away
        Vec3d mPath{ 0.0, 1.0, 0.0 }; // prograde
        Vec3d mPlane{ 0.0, 0.0, 1.0 }; // normal: square to the path, along the orbit's turn
        std::vector<Rocket> mDropped;
        std::vector<Separation> mSeparations;
    };

    void collide(Rocket& a, Rocket& b, double dt);
}
