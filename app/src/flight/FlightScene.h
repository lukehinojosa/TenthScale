#pragma once

#include "flight/Rocket.h"
#include "shared/Scene.h"
#include "shared/SolarSystem.h"

namespace tenth
{
    // Flight, for now a rocket coasting in low Earth orbit with its numbers on screen until the flight view exists
    class FlightScene : public Scene
    {
    public:
        explicit FlightScene(const SolarSystem& solarSystem);

        void onEnter() override;
        void onFixedUpdate(double dt) override;
        void onGui() override;

    private:
        const SolarSystem& mSolarSystem;
        Rocket mRocket;
        double mTime = 0.0;
    };
}
