#include "engine/Scene.h"

#include <utility>

#include "engine/Game.h"
#include "engine/SceneManager.h"

namespace engine
{
    Camera3D& Scene::getCamera() const
    {
        return mpGame->getCamera();
    }

    InputSystem& Scene::getInput() const
    {
        return mpGame->getInput();
    }

    AudioManager& Scene::getAudio() const
    {
        return mpGame->getAudio();
    }

    core::CallbackManager& Scene::getCallbacks() const
    {
        return mpGame->getCallbacks();
    }

    const GameConfig& Scene::getConfig() const
    {
        return mpGame->getConfig();
    }

    double Scene::getTime() const
    {
        return mpGame->getTime();
    }

    double Scene::getFixedDt() const
    {
        return mpGame->getFixedDt();
    }

    void Scene::requestQuit() const
    {
        mpGame->requestQuit();
    }

    void Scene::switchTo(const std::string& name, SceneTransition transition) const
    {
        mpManager->switchTo(name, transition);
    }

    void Scene::switchTo(const std::string& name, std::any value, SceneTransition transition) const
    {
        mpManager->switchTo(name, std::move(value), transition);
    }

    void Scene::push(const std::string& name, std::any value) const
    {
        mpManager->push(name, std::move(value));
    }

    void Scene::pop() const
    {
        mpManager->pop();
    }
}
