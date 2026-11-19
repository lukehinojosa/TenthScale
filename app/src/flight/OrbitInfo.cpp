#include "flight/OrbitInfo.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace tenth
{
    namespace
    {
        constexpr double CIRCULAR = 1e-9; // below this eccentricity there's no point on the orbit to time toward
    }

    OrbitInfo orbitFrom(const Vec3d& position, const Vec3d& velocity, double mu)
    {
        OrbitInfo orbit;
        const double r = position.length();
        if (r <= 0.0 || mu <= 0.0)
        {
            return orbit;
        }
        const double speedSquared = velocity.lengthSquared();
        const double radialSpeed = position.dot(velocity);
        const Vec3d momentum = position.cross(velocity);
        const double h = momentum.length();
        const Vec3d toPeriapsis = (position * (speedSquared - mu / r) - velocity * radialSpeed) / mu;
        const double e = toPeriapsis.length();
        const double energy = speedSquared * 0.5 - mu / r;

        orbit.eccentricity = e;
        orbit.inclination = h > 0.0 ? std::acos(std::clamp(momentum.y / h, -1.0, 1.0)) : 0.0;
        orbit.escaping = energy >= 0.0;
        if (orbit.escaping)
        {
            orbit.periapsis = h * h / (mu * (1.0 + e));
            orbit.apoapsis = std::numeric_limits<double>::infinity();
            if (energy > 0.0 && e > 1.0)
            {
                // Only on the way in is there a periapsis still ahead
                const double a = -mu / (2.0 * energy);
                const double sinhF = radialSpeed / (e * std::sqrt(-mu * a));
                const double F = std::asinh(sinhF);
                const double M = e * sinhF - F;
                orbit.timeToPeriapsis = M < 0.0 ? -M / std::sqrt(mu / (-a * a * a)) : 0.0;
            }
            return orbit;
        }

        const double a = -mu / (2.0 * energy);
        orbit.periapsis = a * (1.0 - e);
        orbit.apoapsis = a * (1.0 + e);
        const double motion = std::sqrt(mu / (a * a * a));
        orbit.period = 2.0 * std::numbers::pi / motion;
        if (e < CIRCULAR)
        {
            return orbit;
        }

        // How far round the orbit it is, measured from periapsis as a share of the lap's time
        const double E = std::atan2(radialSpeed / std::sqrt(mu * a), 1.0 - r / a);
        double M = E - e * std::sin(E);
        if (M < 0.0)
        {
            M += 2.0 * std::numbers::pi;
        }
        const double twoPi = 2.0 * std::numbers::pi;
        orbit.timeToPeriapsis = std::fmod(twoPi - M, twoPi) / motion;
        orbit.timeToApoapsis = std::fmod(std::numbers::pi - M + twoPi, twoPi) / motion;
        return orbit;
    }
}
