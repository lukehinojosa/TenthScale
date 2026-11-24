#pragma once

#include <vector>

#include "shared/Vec3d.h"

namespace tenth
{
    // The curve gravity alone carries something along: an ellipse, or past escape speed an open curve that never
    // comes back. Points on it are named by their angle round from periapsis, growing in the direction of travel.
    // Distances are from the body's centre.
    struct Conic
    {
        double mu = 0.0;
        double eccentricity = 0.0;
        double semiLatus = 0.0; // how far out it is a quarter turn from periapsis
        Vec3d towardPeriapsis;
        Vec3d along; // a quarter turn further round, the way it's going
        double angleNow = 0.0; // where it is now, between -pi and pi

        bool closed() const { return eccentricity < 1.0; }
        double radiusAt(double angle) const;
        Vec3d positionAt(double angle) const;

        // Seconds from now until it gets to an angle, which on an ellipse may be laps ahead
        double timeUntil(double angle) const;
    };

    // False when there's no sideways motion at all, so the path is a straight line with no curve to draw
    bool conicFrom(const Vec3d& position, const Vec3d& velocity, double mu, Conic& conic);

    enum class PathEnd
    {
        Lap, // round and back to where it is now
        Impact, // into the ground
        Escape, // out of the body's sphere of influence
    };

    // Where something is headed from where it is now: one whole lap, or as far as the ground or the edge of the
    // sphere of influence, whichever it reaches first
    struct PredictedPath
    {
        Conic conic;
        std::vector<Vec3d> points; // from now on, starting where it is; empty with no curve or when underground
        PathEnd end = PathEnd::Lap;
        double endAngle = 0.0;
        double endTime = 0.0; // until the impact or escape, or one lap's period
        bool reachesApoapsis = false;
        bool reachesPeriapsis = false;
        double apoapsisAngle = 0.0; // the next time round, so always ahead of angleNow
        double periapsisAngle = 0.0;
    };

    PredictedPath predictPath(const Vec3d& position, const Vec3d& velocity, double mu, double surfaceRadius, double sphereOfInfluence,
                              int segments);
}
