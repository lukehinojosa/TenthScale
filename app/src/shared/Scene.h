#pragma once

#include <optional>

namespace tenth
{
    enum class SceneId
    {
        Build,
        LaunchPad,
        Flight,
    };

    // One screen of the game, such as the build screen or flight. The game forwards its hooks to the current scene.
    class Scene
    {
    public:
        virtual ~Scene() = default;

        virtual void onEnter() {}
        virtual void onExit() {}
        virtual void onFixedUpdate(double /*dt*/) {}
        virtual void onUpdate(double /*frameDt*/, double /*alpha*/) {}
        virtual void onDraw3D() {}
        virtual void onDraw2D() {}
        virtual void onGui() {}

        // Before the window closes, for freeing anything that lives on the GPU
        virtual void onShutdown() {}

        // The game switches between frames, so a scene is never swapped out halfway through its own update
        std::optional<SceneId> takeRequestedScene()
        {
            const std::optional<SceneId> request = mRequestedScene;
            mRequestedScene.reset();
            return request;
        }

    protected:
        void requestScene(SceneId id) { mRequestedScene = id; }

    private:
        std::optional<SceneId> mRequestedScene;
    };
}
