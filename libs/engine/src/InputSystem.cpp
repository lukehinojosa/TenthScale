#include "engine/InputSystem.h"
#include "engine/InterfaceEvent.h"

#include <algorithm>

#include <imgui.h>

#include <core/EventSystem.h>

namespace engine
{
    void InputSystem::update()
    {
        core::EventSystem* pEventSystem = core::EventSystem::getInstance();
        if (pEventSystem == nullptr)
        {
            return;
        }

        bool imguiOwnsKeyboard = false;
        bool imguiOwnsMouse = false;
        if (mRespectImGui && ImGui::GetCurrentContext() != nullptr)
        {
            imguiOwnsKeyboard = ImGui::GetIO().WantCaptureKeyboard;
            imguiOwnsMouse = ImGui::GetIO().WantCaptureMouse;
        }

        // Releases first, so a key tapped within one frame still reports both
        for (auto it = mHeldKeys.begin(); it != mHeldKeys.end();)
        {
            if (IsKeyReleased(*it) || !IsKeyDown(*it))
            {
                pEventSystem->fireEvent(KeyboardEvent(*it, InputAction::RELEASED));
                it = mHeldKeys.erase(it);
            }
            else
            {
                ++it;
            }
        }

        for (int key = GetKeyPressed(); key != 0; key = GetKeyPressed())
        {
            if (imguiOwnsKeyboard)
            {
                continue;
            }

            KeyboardKey pressed = static_cast<KeyboardKey>(key);
            if (std::find(mHeldKeys.begin(), mHeldKeys.end(), pressed) == mHeldKeys.end())
            {
                mHeldKeys.push_back(pressed);
            }
            pEventSystem->fireEvent(KeyboardEvent(pressed, InputAction::PRESSED));
        }

        if (imguiOwnsMouse)
        {
            return;
        }

        const MouseButton buttons[] = { MOUSE_BUTTON_LEFT, MOUSE_BUTTON_RIGHT, MOUSE_BUTTON_MIDDLE };
        Vector2 position = GetMousePosition();
        for (MouseButton button : buttons)
        {
            if (IsMouseButtonPressed(button))
            {
                pEventSystem->fireEvent(MouseEvent(position, button, InputAction::PRESSED));
            }
            if (IsMouseButtonReleased(button))
            {
                pEventSystem->fireEvent(MouseEvent(position, button, InputAction::RELEASED));
            }
        }
    }
}
