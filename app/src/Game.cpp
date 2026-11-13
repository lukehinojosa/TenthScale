#include "Game.h"

#include <optional>
#include <utility>

Game::Game(engine::GameConfig config)
    : engine::Game(std::move(config)), mBuildScene(getCamera(), mCatalog, mDesign, mSolarSystem),
      mLaunchPadScene(getCamera(), mDesign, mSolarSystem), mFlightScene(mSolarSystem)
{
}

void Game::onInit()
{
    switchTo(tenth::SceneId::Build);
}

void Game::switchTo(tenth::SceneId id)
{
    if (mpScene != nullptr)
    {
        mpScene->onExit();
    }
    switch (id)
    {
    case tenth::SceneId::Build:
        mpScene = &mBuildScene;
        break;
    case tenth::SceneId::LaunchPad:
        mpScene = &mLaunchPadScene;
        break;
    case tenth::SceneId::Flight:
        mpScene = &mFlightScene;
        break;
    }
    mpScene->onEnter();
}

void Game::onFixedUpdate(double dt)
{
    mpScene->onFixedUpdate(dt);
}

void Game::onUpdate(double frameDt, double alpha)
{
    // A button pressed in last frame's GUI takes effect here, before anything else runs
    if (const std::optional<tenth::SceneId> request = mpScene->takeRequestedScene())
    {
        switchTo(*request);
    }
    mpScene->onUpdate(frameDt, alpha);

#ifndef __EMSCRIPTEN__
    if (IsKeyPressed(KEY_ESCAPE))
    {
        requestQuit();
    }
#endif
}

void Game::onDraw3D()
{
    mpScene->onDraw3D();
}

void Game::onDraw2D()
{
    mpScene->onDraw2D();
}

void Game::onGui()
{
    mpScene->onGui();
}

void Game::onShutdown()
{
    mBuildScene.onShutdown();
    mLaunchPadScene.onShutdown();
    mFlightScene.onShutdown();
}
