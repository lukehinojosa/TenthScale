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

    // The built rocket as flight sees it: its parts and staging, plus the mass numbers rigid-body physics needs.
    // Measured in the design's axes, +Y up the rocket, with full tanks.
    struct Vehicle
    {
        RocketDesign design; // kept whole, so stages can drop parts and fuel can burn down in flight
        double mass = 0.0; // kg
        Vec3d centerOfMass; // from the root part's centre
        Inertia inertia; // about the center of mass
        double groundClearance = 0.0; // how far the center of mass sits above the rocket's lowest point
        std::vector<Vec3d> outline; // points around each part's edges, from the center of mass, for touching the ground
        int nextStage = 0; // the stage the next press of the staging key fires
    };

    // Each part counts as a solid block of its shape: a cylinder its average width for round parts
    Inertia partInertia(const PlacedPart& placed);

    // Points on the part's rims or corners, in the design's axes: enough for it to rest and pivot on the ground
    std::vector<Vec3d> partOutline(const RocketDesign& design, PartIndex index);

    Vehicle makeVehicle(const RocketDesign& design);
}
