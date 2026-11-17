#pragma once

#include <array>
#include <vector>

#include <raylib.h>

#include <cyclone/collision.h>
#include <engine/FlyCamera.h>

#include "shared/Scene.h"

namespace tenth
{
    struct Body;
    // Calvin's rotating planets: ten planets pulled around a fixed sun by each other's gravity, bouncing off on contact
    class PlanetsScene : public Scene
    {
    public:
        explicit PlanetsScene(Camera3D& camera);

        void onEnter() override;
        void onFixedUpdate(double dt) override;
        void onUpdate(double frameDt, double alpha) override;
        void onDraw3D() override;
        void onGui() override;
        void onShutdown() override;
        

    private:
        static constexpr int PLANET_COUNT = 10;
        static constexpr int SUN = PLANET_COUNT; // the sun is the last body

        struct Body
        {
            cyclone::RigidBody rigidBody;
            cyclone::SphereCollider collider;
        };

        void addBody(int index, cyclone::real radius, cyclone::real mass, cyclone::Vector3 position);
        void applyGravity();
        void pushClickedPlanet();
        void launch();
        void resetPositions();
        cyclone::Vector3 randomPosition();

        Camera3D& mCamera;
        engine::FlyCamera mFlyCamera;
        std::array<Body, PLANET_COUNT + 1> mBodies;
        std::vector<cyclone::Contact> mContacts;
        cyclone::CollisionData mCollisionData;
        cyclone::ContactResolver mResolver;
        float mRestitution = 0.5f;

        Model mPlanetModel{};
        Model mSunModel{};
        bool mModelsReady = false; // made on first entry, since they need the window's graphics context

    public:
        std::array<Body, PLANET_COUNT + 1> getBodies() { return mBodies; };
    };
}
