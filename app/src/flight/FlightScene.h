#pragma once

#include "flight/DesignStats.h"
#include "flight/Rocket.h"
#include "flight/RocketDesign.h"
#include "shared/Scene.h"
#include "shared/SolarSystem.h"

namespace tenth
{
    // Flight, for now the launched rocket standing on Earth with its numbers on screen until the flight view exists
    class FlightScene : public Scene
    {
    public:
        FlightScene(const RocketDesign& design, const SolarSystem& solarSystem);

        void onEnter() override;
        void onFixedUpdate(double dt) override;
        void onGui() override;

    private:
        const RocketDesign& mDesign;
        const SolarSystem& mSolarSystem;
        DesignStats mStats;
        Rocket mRocket;
        double mTime = 0.0;
    };
}
