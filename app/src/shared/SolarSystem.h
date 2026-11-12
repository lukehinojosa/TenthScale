#pragma once

#include <string>
#include <string_view>

#include "shared/Vec3d.h"

namespace tenth
{
    // Everything is one tenth of its real size and distance, with real surface gravity
    constexpr double WORLD_SCALE = 0.1;

    using BodyId = int;
    constexpr BodyId NO_BODY = -1;

    // Fixed facts about a planet, moon, or star. Lengths in metres, already at world scale.
    struct BodyInfo
    {
        std::string name;
        BodyId parent = NO_BODY; // what it orbits; NO_BODY for the Sun
        double radius = 0.0;
        double mu = 0.0; // gravitational parameter, how strongly it pulls
        double sphereOfInfluence = 0.0; // inside this distance its gravity is the one that counts
        double atmosphereHeight = 0.0; // 0 for no atmosphere
    };

    // The shared interface between the Solar System and Rocket and Flight modules. Positions and velocities are
    // relative to the Sun, in metres and metres per second, and times are seconds since the mission started.
    class SolarSystem
    {
    public:
        virtual ~SolarSystem() = default;

        virtual int bodyCount() const = 0;
        virtual const BodyInfo& body(BodyId id) const = 0;

        // Where is this planet at time t?
        virtual Vec3d positionAt(BodyId id, double time) const = 0;
        virtual Vec3d velocityAt(BodyId id, double time) const = 0;

        // Whose gravity is this rocket in? The smallest sphere of influence containing the position.
        virtual BodyId dominantBody(const Vec3d& position, double time) const = 0;

        // NO_BODY if no body has that name
        BodyId findBody(std::string_view name) const;

        double surfaceGravity(BodyId id) const;
    };
}
