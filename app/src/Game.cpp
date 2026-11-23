#include "Game.h"

#include <utility>

#include "flight/BuildScene.h"
#include "flight/DesignFile.h"
#include "flight/FlightScene.h"
#include "shared/SceneNames.h"
#include "solar/PlanetsScene.h"

Game::Game(engine::GameConfig config) : engine::Game(std::move(config))
{
}

void Game::onInit()
{
    tenth::initRocketStorage();
    mDesign = tenth::defaultRocket(mCatalog);
    engine::SceneManager& scenes = getScenes();
    scenes.add<tenth::BuildScene>(tenth::scenes::BUILD, mCatalog, mDesign, mSolarSystem);
    scenes.add<tenth::FlightScene>(tenth::scenes::FLIGHT, mDesign, mSolarSystem);
    scenes.add<tenth::PlanetsScene>(tenth::scenes::PLANETS);
    scenes.switchTo(tenth::scenes::BUILD);
}

void Game::onUpdate(double /*frameDt*/, double /*alpha*/)
{
#ifndef __EMSCRIPTEN__
    if (IsKeyPressed(KEY_ESCAPE))
    {
        requestQuit();
    }
#endif
}
