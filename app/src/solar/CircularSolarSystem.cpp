#include "solar/CircularSolarSystem.h"

#include <cmath>
#include <limits>
#include <numbers>

namespace tenth
{
    CircularSolarSystem::CircularSolarSystem()
    {
        const BodyId sun = addBody("Sun", NO_BODY, 6.9634e8, 1.32712440018e20, 0.0, 0.0, 0.0);
        const BodyId earth = addBody("Earth", sun, 6.371e6, 3.986004418e14, 1.495978707e11, 0.0, 1.0e5);
        addBody("Moon", earth, 1.7374e6, 4.9048695e12, 3.844e8, 0.0, 0.0);
    }

    BodyId CircularSolarSystem::addBody(const char* name, BodyId parent, double realRadius, double realMu, double realOrbitRadius,
                                        double startAngle, double realAtmosphereHeight)
    {
        BodyInfo info;
        info.name = name;
        info.parent = parent;
        info.radius = realRadius * WORLD_SCALE;
        info.mu = realMu * WORLD_SCALE * WORLD_SCALE; // keeps the real surface gravity at the smaller radius
        info.atmosphereHeight = realAtmosphereHeight * WORLD_SCALE;

        Orbit orbit;
        orbit.startAngle = startAngle;
        if (parent == NO_BODY)
        {
            info.sphereOfInfluence = std::numeric_limits<double>::infinity();
        }
        else
        {
            const double parentMu = mBodies[parent].mu;
            orbit.radius = realOrbitRadius * WORLD_SCALE;
            orbit.period = 2.0 * std::numbers::pi * std::sqrt(orbit.radius * orbit.radius * orbit.radius / parentMu);
            info.sphereOfInfluence = orbit.radius * std::pow(info.mu / parentMu, 0.4);
        }

        mBodies.push_back(info);
        mOrbits.push_back(orbit);
        return (BodyId)mBodies.size() - 1;
    }

    int CircularSolarSystem::bodyCount() const
    {
        return (int)mBodies.size();
    }

    const BodyInfo& CircularSolarSystem::body(BodyId id) const
    {
        return mBodies.at(id);
    }

    double CircularSolarSystem::angleAt(BodyId id, double time) const
    {
        const Orbit& orbit = mOrbits[id];
        return orbit.startAngle + 2.0 * std::numbers::pi * time / orbit.period;
    }

    // Orbits run anticlockwise seen from above (+Y), in the XZ plane
    Vec3d CircularSolarSystem::positionAt(BodyId id, double time) const
    {
        const BodyInfo& info = body(id);
        if (info.parent == NO_BODY)
        {
            return {};
        }
        const double angle = angleAt(id, time);
        const double radius = mOrbits[id].radius;
        return positionAt(info.parent, time) + Vec3d{ radius * std::cos(angle), 0.0, -radius * std::sin(angle) };
    }

    Vec3d CircularSolarSystem::velocityAt(BodyId id, double time) const
    {
        const BodyInfo& info = body(id);
        if (info.parent == NO_BODY)
        {
            return {};
        }
        const double angle = angleAt(id, time);
        const double speed = 2.0 * std::numbers::pi * mOrbits[id].radius / mOrbits[id].period;
        return velocityAt(info.parent, time) + Vec3d{ -speed * std::sin(angle), 0.0, -speed * std::cos(angle) };
    }

    // Starts at the Sun and steps down into whichever child's sphere of influence holds the position
    BodyId CircularSolarSystem::dominantBody(const Vec3d& position, double time) const
    {
        BodyId current = 0;
        bool descended = true;
        while (descended)
        {
            descended = false;
            for (BodyId id = 0; id < bodyCount(); id++)
            {
                if (mBodies[id].parent != current)
                {
                    continue;
                }
                if ((position - positionAt(id, time)).length() < mBodies[id].sphereOfInfluence)
                {
                    current = id;
                    descended = true;
                    break;
                }
            }
        }
        return current;
    }
}
