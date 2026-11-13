#pragma once

#include <raylib.h>

#include <engine/Sky.h>

#include "flight/RocketDesign.h"
#include "shared/OrbitCamera.h"
#include "shared/Scene.h"
#include "shared/SolarSystem.h"

namespace tenth
{
    // The rocket standing on the launch pad after rollout, ready to launch or go back to the assembly building
    class LaunchPadScene : public Scene
    {
    public:
        LaunchPadScene(Camera3D& camera, const RocketDesign& design, const SolarSystem& solarSystem);

        void onEnter() override;
        void onUpdate(double frameDt, double alpha) override;
        void onDraw3D() override;
        void onGui() override;
        void onShutdown() override;

    private:
        Camera3D& mCamera;
        const RocketDesign& mDesign;
        OrbitCamera mOrbit;
        double mSurfaceGravity = 9.81;
        engine::Sky mSky; // made on first entry, since it needs the window's graphics context
        bool mSkyReady = false;
    };
}
