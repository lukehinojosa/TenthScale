#include "engine/Game.h"

#include <algorithm>

#include <rlImGui.h>

#include <core/EventSystem.h>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>

EM_JS(int, jsViewportWidth, (), {
    return window.innerWidth;
});

EM_JS(int, jsViewportHeight, (), {
    return window.innerHeight;
});
#endif

namespace engine
{
    Game::Game(GameConfig config) : mConfig(std::move(config))
    {
        mCamera.position = { 0.0f, 6.0f, 14.0f };
        mCamera.target = { 0.0f, 1.0f, 0.0f };
        mCamera.up = { 0.0f, 1.0f, 0.0f };
        mCamera.fovy = 45.0f;
        mCamera.projection = CAMERA_PERSPECTIVE;
    }

    Game::~Game() = default;

    void Game::run()
    {
        init();

#ifdef __EMSCRIPTEN__
        // The browser drives frames; this call never returns
        emscripten_set_main_loop_arg(&Game::webFrame, this, 0, true);
#else
        while (!mShouldQuit && !WindowShouldClose())
        {
            frame();
        }
        shutdown();
#endif
    }

    void Game::init()
    {
        if (core::EventSystem::getInstance() == nullptr)
        {
            core::EventSystem::createInstance();
            core::EventSystem::getInstance()->init();
            mOwnsEventSystem = true;
        }

        unsigned int flags = 0;
        if (mConfig.resizable)
        {
            flags |= FLAG_WINDOW_RESIZABLE;
        }
        if (mConfig.msaa)
        {
            flags |= FLAG_MSAA_4X_HINT;
        }
        if (mConfig.targetFps <= 0)
        {
            flags |= FLAG_VSYNC_HINT;
        }
        SetConfigFlags(flags);
#ifdef __EMSCRIPTEN__
        // In the browser the canvas fills the page, whatever size the config asks for
        if (mConfig.resizable)
        {
            mConfig.width = jsViewportWidth();
            mConfig.height = jsViewportHeight();
        }
#endif
        InitWindow(mConfig.width, mConfig.height, mConfig.title.c_str());
        SetExitKey(KEY_NULL); // Quitting is the game's decision, not a hardcoded Escape
#ifndef __EMSCRIPTEN__
        // The browser paces frames itself, and raylib's frame wait would need ASYNCIFY there
        if (mConfig.targetFps > 0)
        {
            SetTargetFPS(mConfig.targetFps);
        }
#endif

        if (mConfig.audio)
        {
            mAudio.setMasterVolume(mConfig.masterVolume);
            mAudio.init();
        }

        rlImGuiSetup(true);
        onInit();
    }

    void Game::frame()
    {
        // Clamped so a breakpoint or a dragged window doesn't trigger hundreds of catch-up steps
        double frameDt = std::min((double)GetFrameTime(), 0.25);
        const double fixedDt = getFixedDt();

#ifdef __EMSCRIPTEN__
        // raylib doesn't follow the browser window, so keep the canvas matched to it (and mouse coords 1:1)
        if (mConfig.resizable)
        {
            int width = jsViewportWidth();
            int height = jsViewportHeight();
            if (width != GetScreenWidth() || height != GetScreenHeight())
            {
                SetWindowSize(width, height);
            }
        }
#endif

        mInput.update();
        mAudio.update();
        mCallbacks.update(frameDt);

        mAccumulator += frameDt;
        while (mAccumulator >= fixedDt)
        {
            onFixedUpdate(fixedDt);
            mAccumulator -= fixedDt;
            mTime += fixedDt;
        }
        onUpdate(frameDt, mAccumulator / fixedDt);
        onRenderOffscreen();

        BeginDrawing();
        ClearBackground(mConfig.clearColor);

        BeginMode3D(mCamera);
        onDraw3D();
        EndMode3D();

        onDraw2D();

        rlImGuiBegin();
        onGui();
        rlImGuiEnd();

        EndDrawing();
    }

    void Game::shutdown()
    {
        onShutdown();
        mCallbacks.clear();
        mAudio.shutdown();
        rlImGuiShutdown();
        CloseWindow();

        if (mOwnsEventSystem)
        {
            core::EventSystem::getInstance()->removeListenerFromAllEvents(this);
            core::EventSystem::destroyInstance();
            mOwnsEventSystem = false;
        }
    }

    void Game::webFrame(void* pGame)
    {
        static_cast<Game*>(pGame)->frame();
    }
}
