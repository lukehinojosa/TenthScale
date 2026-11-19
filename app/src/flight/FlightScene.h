#pragma once

#include <raylib.h>

#include <engine/Sky.h>

#include "flight/DesignStats.h"
#include "flight/Rocket.h"
#include "flight/RocketDesign.h"
#include "flight/SurfaceFrame.h"
#include "shared/OrbitCamera.h"
#include "shared/Scene.h"
#include "shared/SolarSystem.h"

namespace tenth
{
    // The rocket from rollout onward: held on the launch pad until launch, then flying, with the camera following it
    class FlightScene : public Scene
    {
    public:
        FlightScene(Camera3D& camera, const RocketDesign& design, const SolarSystem& solarSystem);

        void onEnter() override;
        void onExit() override;
        void onFixedUpdate(double dt) override;
        void onUpdate(double frameDt, double alpha) override;
        void onDraw3D() override;
        void onGui() override;
        void onShutdown() override;

    private:
        // Where a point near the reference body shows up on screen: measured from the rocket, in the pad's axes,
        // so the numbers stay small enough for floats even hundreds of kilometres from the planet's centre
        Vector3 toScene(const Vec3d& position) const;

        void drawGround() const;
        void drawFlames() const;

        // Fires the next stage, or launches if the rocket is still on the pad
        void stage();

        void drawStatusBar();
        void drawGauges();
        void drawStageStack();

        Camera3D& mCamera;
        const RocketDesign& mDesign;
        const SolarSystem& mSolarSystem;
        DesignStats mStats; // of what's left to burn, worked out each frame
        Rocket mRocket;
        SurfaceFrame mPad;
        OrbitCamera mOrbit;
        double mTime = 0.0;
        bool mLaunched = false;

        double mPreviousNear = 0.0; // the other scenes' clip planes, put back on leaving
        double mPreviousFar = 0.0;

        engine::Sky mSky; // these need the window's graphics context, so they're made on first entry
        Model mEarthModel{};
        Model mGroundModel{};
        bool mSkyReady = false;
        bool mGraphicsReady = false;
    };
}
