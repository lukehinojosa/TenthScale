#pragma once

#include <vector>

#include "shared/SolarSystem.h"

namespace tenth
{
    // The placeholder Solar System: the Sun, Earth, and the Moon on flat circular orbits. It lets rocket work go
    // ahead until the real Kepler orbits replace it behind the same interface.
    class CircularSolarSystem : public SolarSystem
    {
    public:
        CircularSolarSystem();

        int bodyCount() const override;
        const BodyInfo& body(BodyId id) const override;
        Vec3d positionAt(BodyId id, double time) const override;
        Vec3d velocityAt(BodyId id, double time) const override;
        BodyId dominantBody(const Vec3d& position, double time) const override;

    private:
        struct Orbit
        {
            double radius = 0.0;
            double period = 0.0;
            double startAngle = 0.0;
        };

        // Real-world sizes go in; they're scaled down here. The first body added is the Sun.
        BodyId addBody(const char* name, BodyId parent, double realRadius, double realMu, double realOrbitRadius, double startAngle,
                       double realAtmosphereHeight);

        double angleAt(BodyId id, double time) const;

        std::vector<BodyInfo> mBodies;
        std::vector<Orbit> mOrbits;
    };
}
