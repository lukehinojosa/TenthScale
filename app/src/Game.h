#pragma once

#include <engine/Game.h>

#include "flight/BuildScene.h"
#include "flight/FlightScene.h"
#include "solar/CircularSolarSystem.h"

// Owns the Solar System and the scenes, and forwards each engine hook to whichever scene is current
class Game : public engine::Game
{
public:
    explicit Game(engine::GameConfig config);

private:
    void onInit() override;
    void onFixedUpdate(double dt) override;
    void onUpdate(double frameDt, double alpha) override;
    void onDraw3D() override;
    void onDraw2D() override;
    void onGui() override;

    void switchTo(tenth::SceneId id);

    tenth::CircularSolarSystem mSolarSystem;
    tenth::BuildScene mBuildScene;
    tenth::FlightScene mFlightScene;
    tenth::Scene* mpScene = nullptr;
};
