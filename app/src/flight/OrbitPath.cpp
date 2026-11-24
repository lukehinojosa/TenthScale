#include "flight/OrbitPath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace tenth
{
    namespace
    {
        constexpr double TWO_PI = 2.0 * std::numbers::pi;
        constexpr double PARABOLIC = 1e-9; // this close to escape speed, the path is treated as a parabola
        constexpr double ROUND = 1e-9; // below this eccentricity there's no periapsis or apoapsis worth marking
        constexpr double OPEN_REACH = 0.99; // an open path with nothing to stop it is drawn this share of the way out
    }

    double Conic::radiusAt(double angle) const
    {
        return semiLatus / (1.0 + eccentricity * std::cos(angle));
    }

    Vec3d Conic::positionAt(double angle) const
    {
        return (towardPeriapsis * std::cos(angle) + along * std::sin(angle)) * radiusAt(angle);
    }

    double Conic::timeUntil(double angle) const
    {
        const double e = eccentricity;
        if (std::abs(e - 1.0) < PARABOLIC)
        {
            const auto at = [](double nu)
            {
                const double d = std::tan(nu * 0.5);
                return d + d * d * d / 3.0;
            };
            return std::sqrt(semiLatus * semiLatus * semiLatus / mu) * 0.5 * (at(angle) - at(angleNow));
        }
        const double a = semiLatus / (1.0 - e * e);
        if (e < 1.0)
        {
            // How far round as a share of the lap's time, counting whole laps so later passes come out later
            const auto mean = [&](double nu)
            {
                const double laps = std::floor((nu + std::numbers::pi) / TWO_PI);
                const double within = nu - laps * TWO_PI;
                const double E = 2.0 * std::atan2(std::sqrt(1.0 - e) * std::sin(within * 0.5), std::sqrt(1.0 + e) * std::cos(within * 0.5));
                return E - e * std::sin(E) + laps * TWO_PI;
            };
            return (mean(angle) - mean(angleNow)) / std::sqrt(mu / (a * a * a));
        }
        const auto mean = [&](double nu)
        {
            const double F = 2.0 * std::atanh(std::sqrt((e - 1.0) / (e + 1.0)) * std::tan(nu * 0.5));
            return e * std::sinh(F) - F;
        };
        const double span = -a;
        return (mean(angle) - mean(angleNow)) / std::sqrt(mu / (span * span * span));
    }

    bool conicFrom(const Vec3d& position, const Vec3d& velocity, double mu, Conic& conic)
    {
        const double r = position.length();
        const Vec3d momentum = position.cross(velocity);
        const double h = momentum.length();
        if (r <= 0.0 || mu <= 0.0 || h <= 1e-9 * r * std::max(velocity.length(), 1.0))
        {
            return false;
        }
        const Vec3d toPeriapsis = (position * (velocity.lengthSquared() - mu / r) - velocity * position.dot(velocity)) / mu;
        conic.mu = mu;
        conic.eccentricity = toPeriapsis.length();
        conic.semiLatus = h * h / mu;

        // A circle has no periapsis, so the angle is counted from where it is now
        conic.towardPeriapsis = conic.eccentricity > ROUND ? toPeriapsis / conic.eccentricity : position / r;
        conic.along = (momentum / h).cross(conic.towardPeriapsis);
        conic.angleNow = std::atan2(position.dot(conic.along), position.dot(conic.towardPeriapsis));
        return true;
    }

    PredictedPath predictPath(const Vec3d& position, const Vec3d& velocity, double mu, double surfaceRadius, double sphereOfInfluence,
                              int segments)
    {
        PredictedPath path;
        Conic& conic = path.conic;
        if (!conicFrom(position, velocity, mu, conic))
        {
            return path;
        }
        const double e = conic.eccentricity;
        const double p = conic.semiLatus;
        const double now = conic.angleNow;
        const bool closed = conic.closed();
        double end = closed ? now + TWO_PI : std::numeric_limits<double>::infinity();
        path.end = closed ? PathEnd::Lap : PathEnd::Escape;

        // The ground is crossed going down at minus the angle where the curve meets it; between the two crossings
        // the curve is underground
        if (p / (1.0 + e) < surfaceRadius)
        {
            const double meets = e > ROUND ? (p / surfaceRadius - 1.0) / e : -2.0;
            if (meets <= -1.0)
            {
                return path;
            }
            const double ground = std::acos(meets);
            double impact = std::numeric_limits<double>::infinity();
            if (now <= -ground)
            {
                impact = -ground;
            }
            else if (now >= ground)
            {
                impact = closed ? TWO_PI - ground : impact;
            }
            else
            {
                return path;
            }
            if (impact < end)
            {
                end = impact;
                path.end = PathEnd::Impact;
            }
        }

        // The edge of the sphere of influence is crossed going out at the angle where the curve meets it
        if (std::isfinite(sphereOfInfluence) && (!closed || p / (1.0 - e) > sphereOfInfluence))
        {
            const double edge = std::acos(std::clamp((p / sphereOfInfluence - 1.0) / e, -1.0, 1.0));
            const double escape = std::max(edge, now);
            if (escape < end)
            {
                end = escape;
                path.end = PathEnd::Escape;
            }
        }

        if (!std::isfinite(end))
        {
            end = std::acos(-1.0 / e) * OPEN_REACH;
        }
        path.endAngle = end;
        path.endTime = path.end == PathEnd::Lap ? conic.timeUntil(now + TWO_PI) : conic.timeUntil(end);
        segments = std::max(segments, 2);
        path.points.reserve((size_t)segments + 1);
        for (int i = 0; i <= segments; i++)
        {
            path.points.push_back(conic.positionAt(now + (end - now) * i / segments));
        }

        if (e > ROUND)
        {
            double periapsis = 0.0;
            while (periapsis <= now)
            {
                periapsis += TWO_PI;
            }
            path.periapsisAngle = periapsis;
            path.reachesPeriapsis = periapsis <= end;
            if (closed)
            {
                double apoapsis = std::numbers::pi;
                while (apoapsis <= now)
                {
                    apoapsis += TWO_PI;
                }
                path.apoapsisAngle = apoapsis;
                path.reachesApoapsis = apoapsis <= end;
            }
        }
        return path;
    }
}
