#include "engine/FlyCamera.h"

#include <imgui.h>

namespace engine
{
    void FlyCamera::update(Camera3D& camera, float dt)
    {
        bool imguiOwnsMouse = ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureMouse;
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && !imguiOwnsMouse)
        {
            DisableCursor();
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT))
        {
            EnableCursor();
        }
        if (!IsCursorHidden())
        {
            return;
        }

        float forward = (float)IsKeyDown(KEY_W) - (float)IsKeyDown(KEY_S);
        float right = (float)IsKeyDown(KEY_D) - (float)IsKeyDown(KEY_A);
        float up = (float)IsKeyDown(KEY_E) - (float)IsKeyDown(KEY_Q);

        float step = moveSpeed * dt;
        if (IsKeyDown(KEY_LEFT_SHIFT))
        {
            step *= sprintMultiplier;
        }

        Vector2 mouseDelta = GetMouseDelta();
        Vector3 movement = { forward * step, right * step, up * step };
        Vector3 rotation = { mouseDelta.x * lookSensitivity, mouseDelta.y * lookSensitivity, 0.0f };

        // UpdateCameraPro wants movement as (forward, right, up) and rotation as (yaw, pitch, roll)
        UpdateCameraPro(&camera, movement, rotation, -GetMouseWheelMove());
    }
}
