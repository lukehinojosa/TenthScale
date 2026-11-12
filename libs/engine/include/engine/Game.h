#pragma once

#include <string>

#include <raylib.h>

#include <core/CallbackManager.h>
#include <core/EventListener.h>

#include "Audio.h"
#include "InputSystem.h"

namespace engine
{
    struct GameConfig
    {
        std::string title = "CppTemplate";
        int width = 1280;
        int height = 720;
        double fixedHz = 50.0; // Physics rate, independent of the frame rate
        int targetFps = 0; // 0 leaves it to vsync
        bool resizable = true;
        bool msaa = true;
        bool audio = true;
        float masterVolume = 1.0f;
        Color clearColor = { 30, 30, 36, 255 };
    };

    // Subclass and override the on* hooks; run() owns the window, ImGui, and the frame loop
    class Game : public core::EventListener
    {
    public:
        explicit Game(GameConfig config = {});
        virtual ~Game() override;

        // Blocks until quit on desktop; on the web it hands the loop to the browser and returns
        void run();
        void requestQuit() { mShouldQuit = true; }

    protected:
        virtual void onInit() {}
        virtual void onShutdown() {}

        // Called zero or more times per frame with a constant dt
        virtual void onFixedUpdate(double /*dt*/) {}

        // Called once per frame; alpha is how far between the last two fixed steps we are (0 to 1)
        virtual void onUpdate(double /*frameDt*/, double /*alpha*/) {}

        // Before the frame starts drawing: render-to-texture passes such as shadow maps go here
        virtual void onRenderOffscreen() {}

        virtual void onDraw3D() {}
        virtual void onDraw2D() {}
        virtual void onGui() {}

        void handleEvent(const core::Event& /*theEvent*/) override {}

        const GameConfig& getConfig() const { return mConfig; }
        Camera3D& getCamera() { return mCamera; }
        InputSystem& getInput() { return mInput; }
        AudioManager& getAudio() { return mAudio; }
        core::CallbackManager& getCallbacks() { return mCallbacks; }
        double getFixedDt() const { return 1.0 / mConfig.fixedHz; }
        double getTime() const { return mTime; }

    private:
        void init();
        void frame();
        void shutdown();
        static void webFrame(void* pGame);

        GameConfig mConfig;
        Camera3D mCamera{};
        InputSystem mInput;
        AudioManager mAudio;
        core::CallbackManager mCallbacks;
        double mAccumulator = 0.0;
        double mTime = 0.0;
        bool mShouldQuit = false;
        bool mOwnsEventSystem = false;
    };
}
