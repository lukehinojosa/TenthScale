#pragma once

#include <engine/Game.h>

#include "flight/BuildScene.h"
#include "flight/FlightScene.h"
#include "flight/LaunchPadScene.h"
#include "flight/PartCatalog.h"
#include "flight/RocketDesign.h"
#include "solar/CircularSolarSystem.h"

// Owns the Solar System, the parts, the rocket being built, and the scenes; forwards engine hooks to the current scene
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
    void onShutdown() override;

    void switchTo(tenth::SceneId id);

    tenth::CircularSolarSystem mSolarSystem;
    tenth::PartCatalog mCatalog = tenth::PartCatalog::standard();
    tenth::RocketDesign mDesign; // built on the build screen, flown on launch
    tenth::BuildScene mBuildScene;
    tenth::LaunchPadScene mLaunchPadScene;
    tenth::FlightScene mFlightScene;
    tenth::Scene* mpScene = nullptr;
};
