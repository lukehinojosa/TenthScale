#include "engine/SceneManager.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace engine
{
    Scene& SceneManager::add(const std::string& name, std::unique_ptr<Scene> pScene)
    {
        if (pScene == nullptr)
        {
            throw std::logic_error("SceneManager::add was given no scene for '" + name + "'");
        }
        if (has(name))
        {
            throw std::logic_error("A scene named '" + name + "' was already added");
        }
        pScene->mpGame = &mGame;
        pScene->mpManager = this;
        pScene->mName = name;
        Scene& scene = *pScene;
        mScenes.emplace(name, std::move(pScene));
        return scene;
    }

    Scene* SceneManager::find(const std::string& name) const
    {
        const auto found = mScenes.find(name);
        return found == mScenes.end() ? nullptr : found->second.get();
    }

    bool SceneManager::known(const std::string& name, const char* action) const
    {
        if (has(name))
        {
            return true;
        }
        TraceLog(LOG_WARNING, "SCENE: can't %s '%s': no scene by that name was added", action, name.c_str());
        return false;
    }

    void SceneManager::switchTo(const std::string& name, SceneTransition transition)
    {
        switchTo(name, std::any(), transition);
    }

    void SceneManager::switchTo(const std::string& name, std::any value, SceneTransition transition)
    {
        if (known(name, "switch to"))
        {
            mRequests.push_back(Request{ RequestType::Switch, name, transition, std::move(value) });
        }
    }

    void SceneManager::push(const std::string& name, std::any value)
    {
        if (known(name, "push"))
        {
            mRequests.push_back(Request{ RequestType::Push, name, {}, std::move(value) });
        }
    }

    void SceneManager::pop()
    {
        mRequests.push_back(Request{ RequestType::Pop, {}, {}, {} });
    }

    float SceneManager::getFadeAmount() const
    {
        if (mFade == FadeState::None || mFadeDuration <= 0.0f)
        {
            return 0.0f;
        }
        const float progress = std::clamp(mFadeTime / mFadeDuration, 0.0f, 1.0f);
        return mFade == FadeState::Out ? progress : 1.0f - progress;
    }

    void SceneManager::startFadeIn(const SceneTransition& transition)
    {
        mFadeTime = 0.0f;
        mFadeDuration = transition.fadeInSeconds;
        mFadeColor = transition.color;
        mFade = transition.fadeInSeconds > 0.0f ? FadeState::In : FadeState::None;
    }

    void SceneManager::applyRequests()
    {
        // A fading switch holds back everything asked for after it until the screen is covered
        while (!mRequests.empty() && mFade != FadeState::Out)
        {
            const Request request = mRequests.front();
            mRequests.pop_front();
            if (request.type == RequestType::Switch && request.transition.fadeOutSeconds > 0.0f && !mStack.empty())
            {
                // Picks up from however dark a fade-in still running has left the screen
                const float covered = getFadeAmount();
                mFadingSwitch = request;
                mFade = FadeState::Out;
                mFadeDuration = request.transition.fadeOutSeconds;
                mFadeTime = covered * mFadeDuration;
                mFadeColor = request.transition.color;
                return;
            }
            apply(request);
            if (request.type == RequestType::Switch)
            {
                startFadeIn(request.transition);
            }
        }
    }

    void SceneManager::enter(Scene& scene, const std::any& value)
    {
        scene.mParameter = value;
        if (!scene.mLoaded)
        {
            scene.onLoad();
            scene.mLoaded = true;
        }
        mStack.push_back(&scene);
        scene.onEnter();
    }

    void SceneManager::apply(const Request& request)
    {
        switch (request.type)
        {
        case RequestType::Switch:
        {
            while (!mStack.empty())
            {
                Scene* pTop = mStack.back();
                mStack.pop_back();
                pTop->onExit();
            }
            enter(*find(request.name), request.value);
            break;
        }
        case RequestType::Push:
        {
            Scene* pScene = find(request.name);
            if (std::find(mStack.begin(), mStack.end(), pScene) != mStack.end())
            {
                TraceLog(LOG_WARNING, "SCENE: can't push '%s': it's already on the stack", request.name.c_str());
                break;
            }
            if (!mStack.empty())
            {
                mStack.back()->onPause();
            }
            enter(*pScene, request.value);
            break;
        }
        case RequestType::Pop:
        {
            if (mStack.empty())
            {
                TraceLog(LOG_WARNING, "SCENE: can't pop: the stack is empty");
                break;
            }
            Scene* pTop = mStack.back();
            mStack.pop_back();
            pTop->onExit();
            if (!mStack.empty())
            {
                mStack.back()->onResume();
            }
            break;
        }
        }
    }

    std::size_t SceneManager::firstVisible() const
    {
        std::size_t index = mStack.size() - 1;
        while (index > 0 && mStack[index]->isOverlay())
        {
            index--;
        }
        return index;
    }

    void SceneManager::fixedUpdate(double dt)
    {
        if (Scene* pTop = getCurrent())
        {
            pTop->onFixedUpdate(dt);
        }
    }

    void SceneManager::beginFrame(double frameDt)
    {
        // Requests first, so a fade starts moving on the frame it was asked for
        applyRequests();
        if (mFade == FadeState::None)
        {
            return;
        }
        mFadeTime += (float)frameDt;
        if (mFadeTime < mFadeDuration)
        {
            return;
        }
        if (mFade == FadeState::Out)
        {
            // Swapped while the screen is fully covered, which this frame then draws
            apply(mFadingSwitch);
            startFadeIn(mFadingSwitch.transition);
            applyRequests();
        }
        else
        {
            mFade = FadeState::None;
        }
    }

    void SceneManager::update(double frameDt, double alpha)
    {
        if (Scene* pTop = getCurrent())
        {
            pTop->onUpdate(frameDt, alpha);
        }
    }

    void SceneManager::renderOffscreen()
    {
        if (mStack.empty())
        {
            return;
        }
        for (std::size_t i = firstVisible(); i < mStack.size(); i++)
        {
            mStack[i]->onRenderOffscreen();
        }
    }

    void SceneManager::draw3D()
    {
        if (mStack.empty())
        {
            return;
        }
        for (std::size_t i = firstVisible(); i < mStack.size(); i++)
        {
            mStack[i]->onDraw3D();
        }
    }

    void SceneManager::draw2D()
    {
        if (mStack.empty())
        {
            return;
        }
        for (std::size_t i = firstVisible(); i < mStack.size(); i++)
        {
            mStack[i]->onDraw2D();
        }
    }

    void SceneManager::gui()
    {
        if (Scene* pTop = getCurrent())
        {
            pTop->onGui();
        }
    }

    void SceneManager::drawTransition() const
    {
        const float amount = getFadeAmount();
        if (amount > 0.0f)
        {
            DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), ColorAlpha(mFadeColor, amount));
        }
    }

    void SceneManager::unloadAll()
    {
        while (!mStack.empty())
        {
            Scene* pTop = mStack.back();
            mStack.pop_back();
            pTop->onExit();
        }
        mRequests.clear();
        mFade = FadeState::None;
        for (auto& [name, pScene] : mScenes)
        {
            if (pScene->mLoaded)
            {
                pScene->onUnload();
                pScene->mLoaded = false;
            }
        }
    }
}
