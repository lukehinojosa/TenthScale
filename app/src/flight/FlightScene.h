#pragma once

#include <random>
#include <vector>

#include <raylib.h>

#include <engine/Scene.h>

#include "flight/DesignStats.h"
#include "flight/Globe.h"
#include "flight/LaunchPad.h"
#include "flight/MapView.h"
#include "flight/OrbitInfo.h"
#include "flight/PlanetSurface.h"
#include "flight/Rocket.h"
#include "flight/RocketDesign.h"
#include "flight/SpaceSky.h"
#include "flight/SurfaceFrame.h"
#include "flight/TimeWarp.h"
#include "shared/OrbitCamera.h"
#include "shared/SolarSystem.h"

namespace tenth
{
    // The rocket from rollout onward: held on the launch pad until launch, then flying, with the camera following it
    class FlightScene : public engine::Scene
    {
    public:
        FlightScene(const RocketDesign& design, const SolarSystem& solarSystem);

        void onLoad() override;
        void onUnload() override;
        void onEnter() override;
        void onExit() override;
        void onFixedUpdate(double dt) override;
        void onUpdate(double frameDt, double alpha) override;
        void onDraw3D() override;
        void onGui() override;

    private:
        // How high the camera is above the reference body's surface
        double cameraHeight() const;

        // Where a point near the reference body shows up on screen: measured from the rocket, in the pad's axes,
        // so the numbers stay small enough for floats even hundreds of kilometres from the planet's centre
        Vector3 toScene(const Vec3d& position) const;

        void drawGround() const;

        // The way to the Sun in the scene's axes, from the body the rocket is around
        Vector3 sunDirection() const;
        Vector3 sunFrom(BodyId id) const;

        // M swaps between following the rocket and the map
        void toggleMap();
        MapView::Flight mapFlight() const;

        // Keeps the rocket's idea of where the target is up to date, and drops a target the rocket has arrived at
        void updateTarget();

        // The raised hardstand with its sloped sides, the flame trench, and the service tower
        void drawPad() const;

        // Draws a rocket or dropped piece where it is, turned however it's facing
        void drawRocket(const Rocket& rocket) const;
        void drawFlames(const Rocket& rocket) const;
        void drawParachutes(const Rocket& rocket) const;

        // One fixed step of everything that moves; time warp runs several of these per tick
        void stepPhysics(double dt);

        // Steps the dropped pieces and clears away those that have landed and settled or fallen far behind
        void updateDebris(double dt);
        void collectDropped();

        // A flash, sparks, and smoke where pieces came apart
        void burst(const Separation& separation);
        void updateEffects(double dt);
        void drawEffects() const;

        // Fires the next stage, or launches if the rocket is still on the pad
        void stage();

        void drawStatusBar();
        void drawClock();
        void drawInstruments();
        void drawSas();
        void drawSasSwitch();
        void drawSasControl();
        void drawOrbit();
        void drawGForce();
        void drawStageStack();
        void drawGameView();

        const RocketDesign& mDesign;
        const SolarSystem& mSolarSystem;
        DesignStats mStats; // of what's left to burn, worked out each frame
        Rocket mRocket;

        // A dropped stage or nose cone, falling on its own
        struct Debris
        {
            Rocket rocket;
            double settledTime = 0.0; // how long it has lain still on the ground
        };
        std::vector<Debris> mDebris;

        // One blob of a separation's flash, sparks, or smoke. It moves with what it came off, and its own outward
        // speed fades as it spreads.
        struct Puff
        {
            Vec3d position;
            Vec3d velocity;
            Vec3d drift; // the joint's velocity, which the puff settles back to
            double age = 0.0;
            double life = 1.0;
            double fade = 0.0; // how quickly its outward speed dies away, per second
            float radius = 0.1f;
            float growth = 0.0f; // metres per second
            Color color = WHITE;
        };
        std::vector<Puff> mPuffs;
        std::mt19937 mRandom{ 7 };
        SurfaceFrame mPad;
        LaunchPad mLaunchPad;
        OrbitCamera mOrbit;

        // The camera eases to a new framing when the rocket splits, rather than jumping with its center of mass
        Vector3 mFocus{};
        float mZoomTarget = 0.0f; // 0 while not easing the zoom
        float mFramedHeight = 0.0f; // the rocket's height the zoom was last set for
        bool mSnapCamera = true;
        double mTime = 0.0;
        double mGForce = 1.0; // the push felt along the rocket, in multiples of Earth's gravity, smoothed
        bool mLaunched = false;
        TimeWarp mWarp;
        float mSasPointer = -1.5707963f; // where the dial's rocket points, in screen radians, easing toward the mode

        double mPreviousNear = 0.0; // the other scenes' clip planes, put back on leaving
        double mPreviousFar = 0.0;

        SpaceSky mSky;
        PlanetSurface mPlanet;
        BodyGlobes mGlobes; // every body but Earth, shared with the map
        Vector3 mSun{ 0.0f, 1.0f, 0.0f };

        MapView mMap;
        bool mMapOpen = false;
        bool mMapFresh = true; // not yet opened this flight, so it starts framed on the rocket's orbit
        BodyId mTarget = NO_BODY; // picked on the map
    };
}
