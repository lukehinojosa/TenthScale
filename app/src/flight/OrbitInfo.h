#pragma once

#include "shared/Vec3d.h"

namespace tenth
{
    // The shape of the path a body's gravity alone would carry something along, read from where it is and how fast
    // it's going. Distances are from the body's centre.
    struct OrbitInfo
    {
        double eccentricity = 0.0;
        double periapsis = 0.0; // the closest it comes
        double apoapsis = 0.0; // the furthest it gets; meaningless while escaping
        double period = 0.0; // seconds for one lap; 0 while escaping
        double timeToApoapsis = 0.0; // 0 while escaping
        double timeToPeriapsis = 0.0; // 0 while escaping and already past it
        double inclination = 0.0; // radians the orbit tilts from the equator
        bool escaping = false; // fast enough that it never comes back
    };

    // Position and velocity relative to a body with gravitational parameter mu
    OrbitInfo orbitFrom(const Vec3d& position, const Vec3d& velocity, double mu);
}
