#pragma once

#include <vector>

#include <raylib.h>

#include <core/Trackable.h>

namespace engine
{
    // Polls raylib once per frame and fires KeyboardEvent and MouseEvent through the EventSystem
    class InputSystem : public core::Trackable
    {
    public:
        void update();

        // Input ImGui is using (typing in a text box, clicking a window) is not turned into events
        void setRespectImGui(bool respect) { mRespectImGui = respect; }

    private:
        std::vector<KeyboardKey> mHeldKeys;
        bool mRespectImGui = true;
    };
}
