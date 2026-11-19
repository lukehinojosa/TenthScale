#pragma once

#include <vector>

#include "flight/RocketDesign.h"
#include "shared/Vec3d.h"

namespace tenth
{
    // How hard a body is to spin about each axis, in kg m^2. Symmetric, so six numbers cover it.
    struct Inertia
    {
        double xx = 0.0;
        double yy = 0.0; // about the rocket's long axis, its roll
        double zz = 0.0;
        double xy = 0.0;
        double xz = 0.0;
        double yz = 0.0;

        Inertia& operator+=(const Inertia& other);
    };

    // The built rocket as flight sees it: its parts and staging, the fuel left in each part, and the mass numbers
    // rigid-body physics needs. Measured in the design's axes, +Y up the rocket.
    struct Vehicle
    {
        RocketDesign design; // kept whole, so stages can drop parts in flight
        std::vector<double> fuel; // kg left in each part
        std::vector<bool> lit; // engines the staging key has lit
        std::vector<bool> deployed; // parachutes the staging key has opened
        std::vector<PartIndex> sections; // the part starting each part's section, whose tanks its engines draw from
        double throttle = 1.0; // 0 to 1, shared by every lit engine
        int nextStage = 0; // the stage the next press of the staging key fires

        double mass = 0.0; // kg
        Vec3d centerOfMass; // from the root part's centre
        Inertia inertia; // about the center of mass
        double groundClearance = 0.0; // how far the center of mass sits above the rocket's lowest point, tanks full
        std::vector<Vec3d> outline; // points around each part's edges, from the root, for touching the ground
    };

    // One engine's push at the current air pressure, where 1 is sea level and 0 is vacuum
    struct EngineOutput
    {
        double thrust = 0.0; // newtons at full throttle
        double isp = 0.0; // seconds
    };

    // Each part counts as a solid block of its shape: a cylinder its average width for round parts
    Inertia partInertia(const PlacedPart& placed, double mass);

    // Points on the part's rims or corners, in the design's axes: enough for it to rest and pivot on the ground
    std::vector<Vec3d> partOutline(const RocketDesign& design, PartIndex index);

    // Whether a point, in the design's axes, lies inside the part. If it does, depth is how far it is from the
    // nearest face and outward is the way out through that face.
    bool insidePart(const RocketDesign& design, PartIndex index, const Vec3d& point, double& depth, Vec3d& outward);

    // With every tank full and nothing lit
    Vehicle makeVehicle(const RocketDesign& design);

    // Works out mass, center of mass, and inertia again from the fuel left
    void updateMass(Vehicle& vehicle);

    // Lights the engines and opens the parachutes in the next stage, and moves on to the one after. Decouplers are
    // left for the caller, since splitting the rocket moves it. Returns the parts it fired.
    std::vector<PartIndex> fireNextStage(Vehicle& vehicle);

    // Takes the part and everything attached through it off as a vehicle of its own, each part keeping its fuel and
    // lit engines. rootOffset is where the new vehicle's root sat in this one's design, before the split.
    Vehicle detachBranch(Vehicle& vehicle, PartIndex index, Vec3d& rootOffset);

    double sectionFuel(const Vehicle& vehicle, PartIndex section);
    double sectionCapacity(const Vehicle& vehicle, PartIndex section);

    // Lit and with fuel left in its section
    bool isBurning(const Vehicle& vehicle, PartIndex engine);

    // Thinner air lets the exhaust expand more, so the same fuel flow pushes harder and lasts longer
    EngineOutput engineOutput(const PartDefinition& engine, double pressure);

    // Every burning engine draws its fuel flow from the tanks in its section, each emptying at the same pace.
    // Leaves the mass numbers for updateMass.
    void burnFuel(Vehicle& vehicle, double pressure, double dt);
}
