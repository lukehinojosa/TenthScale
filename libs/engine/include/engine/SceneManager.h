#pragma once

#include <any>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <raylib.h>

#include "Scene.h"

namespace engine
{
    // Owns a game's scenes by name and runs a stack of them: the top one updates, and overlays let the ones under
    // them show. Changes wait for the start of the next frame, so a scene is never swapped out halfway through its
    // own hooks. Game calls the per-frame functions; games only add scenes and ask for changes.
    class SceneManager
    {
    public:
        explicit SceneManager(Game& game) : mGame(game) {}
        SceneManager(const SceneManager&) = delete;
        SceneManager& operator=(const SceneManager&) = delete;

        // Names must be unique; adding one twice throws std::logic_error
        Scene& add(const std::string& name, std::unique_ptr<Scene> pScene);

        template <typename T, typename... Args>
        T& add(const std::string& name, Args&&... args)
        {
            auto pScene = std::make_unique<T>(std::forward<Args>(args)...);
            T& scene = *pScene;
            add(name, std::move(pScene));
            return scene;
        }

        bool has(const std::string& name) const { return mScenes.count(name) > 0; }
        Scene* find(const std::string& name) const;

        // Replaces the whole stack with this scene. The value, of any type, reaches it through Scene::getParameter().
        void switchTo(const std::string& name, SceneTransition transition = {});
        void switchTo(const std::string& name, std::any value, SceneTransition transition = {});

        // Puts a scene on top, pausing the one below, and takes it off again
        void push(const std::string& name, std::any value = {});
        void pop();

        Scene* getCurrent() const { return mStack.empty() ? nullptr : mStack.back(); }
        std::size_t getDepth() const { return mStack.size(); }
        bool isTransitioning() const { return mFade != FadeState::None || !mRequests.empty(); }

        // How much of the screen the fade covers, from 0 to 1
        float getFadeAmount() const;

        // Advances any fade and applies the changes asked for since last frame, before any scene hook runs
        void beginFrame(double frameDt);
        void fixedUpdate(double dt);
        void update(double frameDt, double alpha);
        void renderOffscreen();
        void draw3D();
        void draw2D();
        void gui();
        void drawTransition() const;

        // Exits every scene on the stack and unloads every loaded one; Game calls this before closing the window
        void unloadAll();

    private:
        enum class RequestType
        {
            Switch,
            Push,
            Pop
        };

        struct Request
        {
            RequestType type = RequestType::Switch;
            std::string name;
            SceneTransition transition;
            std::any value;
        };

        enum class FadeState
        {
            None,
            Out,
            In
        };

        void applyRequests();
        void apply(const Request& request);
        void enter(Scene& scene, const std::any& value);
        void startFadeIn(const SceneTransition& transition);
        bool known(const std::string& name, const char* action) const;

        // The lowest scene that's drawn: the top one, or further down while the ones above are overlays
        std::size_t firstVisible() const;

        Game& mGame;
        std::map<std::string, std::unique_ptr<Scene>> mScenes;
        std::vector<Scene*> mStack;
        std::deque<Request> mRequests;

        FadeState mFade = FadeState::None;
        float mFadeTime = 0.0f;
        float mFadeDuration = 0.0f;
        Color mFadeColor = BLACK;
        Request mFadingSwitch;
    };
}
