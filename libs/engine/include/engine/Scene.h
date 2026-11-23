#pragma once

#include <any>
#include <string>
#include <type_traits>

#include <raylib.h>

namespace core
{
    class CallbackManager;
}

namespace engine
{
    class AudioManager;
    class Game;
    class InputSystem;
    class SceneManager;
    struct GameConfig;

    // How a switch looks: fade to a colour, swap scenes while the screen is covered, then fade back in
    struct SceneTransition
    {
        float fadeOutSeconds = 0.0f;
        float fadeInSeconds = 0.0f;
        Color color = BLACK;

        static SceneTransition fade(float seconds, Color color = BLACK) { return SceneTransition{ seconds / 2, seconds / 2, color }; }
    };

    // One screen of a game, such as a title menu, a level, or a pause menu. Add it to the game's SceneManager, which
    // calls these hooks while the scene is on its stack.
    class Scene
    {
    public:
        virtual ~Scene() = default;

        // Once, the first time the scene is entered: load textures and models here rather than in the constructor
        virtual void onLoad() {}

        // Once, before the window closes, if the scene was loaded: free anything on the GPU here
        virtual void onUnload() {}

        // Each time the scene joins or leaves the stack
        virtual void onEnter() {}
        virtual void onExit() {}

        // When another scene is pushed on top of this one, and when that scene is popped
        virtual void onPause() {}
        virtual void onResume() {}

        // Only the scene on top of the stack updates
        virtual void onFixedUpdate(double /*dt*/) {}
        virtual void onUpdate(double /*frameDt*/, double /*alpha*/) {}

        // Called for the top scene, and for the scenes under it while the ones above are overlays
        virtual void onRenderOffscreen() {}
        virtual void onDraw3D() {}
        virtual void onDraw2D() {}

        // Only the top scene draws GUI, so windows from a paused scene can't be clicked
        virtual void onGui() {}

        // An overlay, such as a pause menu, leaves the scenes under it drawn though not updated
        virtual bool isOverlay() const { return false; }

        const std::string& getName() const { return mName; }
        bool isLoaded() const { return mLoaded; }

        // The value given to switchTo() or push() the last time this scene was entered, set before onLoad() and
        // onEnter(). Empty when none was given; a new entry replaces it, while resuming after a pop keeps it.
        const std::any& getParameter() const { return mParameter; }
        bool hasParameter() const { return mParameter.has_value(); }

        // The value as a T, or fallback when there is none or it's another type. A string literal reads as std::string.
        template <typename T>
        T getParameter(T fallback) const
        {
            if (const T* pValue = std::any_cast<T>(&mParameter))
            {
                return *pValue;
            }
            if constexpr (std::is_same_v<T, std::string>)
            {
                if (const char* const* pText = std::any_cast<const char*>(&mParameter))
                {
                    return std::string(*pText);
                }
            }
            return fallback;
        }

        // Null when there is no value of that type; for values too large or awkward to copy
        template <typename T>
        const T* findParameter() const
        {
            return std::any_cast<T>(&mParameter);
        }

    protected:
        // Valid once the scene has been added to a SceneManager
        Game& getGame() const { return *mpGame; }
        SceneManager& getScenes() const { return *mpManager; }

        // Shortcuts for the same calls on getScenes(), so a scene can change screens without including SceneManager.h
        void switchTo(const std::string& name, SceneTransition transition = {}) const;
        void switchTo(const std::string& name, std::any value, SceneTransition transition = {}) const;
        void push(const std::string& name, std::any value = {}) const;
        void pop() const;
        Camera3D& getCamera() const;
        InputSystem& getInput() const;
        AudioManager& getAudio() const;
        core::CallbackManager& getCallbacks() const;
        const GameConfig& getConfig() const;
        double getTime() const;
        double getFixedDt() const;
        void requestQuit() const;

    private:
        friend class SceneManager;

        Game* mpGame = nullptr;
        SceneManager* mpManager = nullptr;
        std::string mName;
        std::any mParameter;
        bool mLoaded = false;
    };
}
