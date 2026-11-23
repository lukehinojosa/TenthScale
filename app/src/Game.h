#pragma once

#include <engine/Game.h>

#include "flight/PartCatalog.h"
#include "flight/RocketDesign.h"
#include "solar/CircularSolarSystem.h"

// Owns the Solar System, the parts, and the rocket being built, and adds the scenes that share them to the engine's
// SceneManager, which runs whichever is current
class Game : public engine::Game
{
public:
    explicit Game(engine::GameConfig config);

private:
    void onInit() override;
    void onUpdate(double frameDt, double alpha) override;

    tenth::CircularSolarSystem mSolarSystem;
    tenth::PartCatalog mCatalog = tenth::PartCatalog::standard();
    tenth::RocketDesign mDesign; // built on the build screen, flown on launch
};
