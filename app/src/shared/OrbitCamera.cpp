#include "shared/OrbitCamera.h"

#include <algorithm>
#include <cmath>

#include <imgui.h>

namespace tenth
{
    void OrbitCamera::update(Camera3D& camera, float dt)
    {
        const bool hasImGui = ImGui::GetCurrentContext() != nullptr;
        const bool imguiOwnsMouse = hasImGui && ImGui::GetIO().WantCaptureMouse;
        const bool imguiOwnsKeyboard = hasImGui && ImGui::GetIO().WantCaptureKeyboard;

        // A drag only starts outside the panels, but keeps going if the mouse then passes over one
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && !imguiOwnsMouse)
        {
            mOrbiting = true;
        }
        if (IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE) && !imguiOwnsMouse)
        {
            mLifting = true;
        }
        if (!IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            mOrbiting = false;
        }
        if (!IsMouseButtonDown(MOUSE_BUTTON_MIDDLE))
        {
            mLifting = false;
        }

        const Vector2 mouseDelta = GetMouseDelta();
        if (mOrbiting)
        {
            yaw -= mouseDelta.x * orbitSensitivity;
            pitch += mouseDelta.y * orbitSensitivity;
        }
        if (mLifting)
        {
            focus.y += mouseDelta.y * distance * 0.002f;
        }
        if (!imguiOwnsMouse)
        {
            distance *= std::pow(1.0f - zoomStep, GetMouseWheelMove());
        }
        if (!imguiOwnsKeyboard)
        {
            const float lift = (float)IsKeyDown(KEY_W) - (float)IsKeyDown(KEY_S);
            focus.y += lift * liftSpeed * distance * dt;
        }

        pitch = std::clamp(pitch, minPitch, maxPitch);
        distance = std::clamp(distance, minDistance, maxDistance);
        focus.y = std::clamp(focus.y, minFocusHeight, maxFocusHeight);
        apply(camera);
    }

    void OrbitCamera::apply(Camera3D& camera) const
    {
        const float flat = distance * std::cos(pitch);
        const Vector3 offset = { flat * std::sin(yaw), distance * std::sin(pitch), flat * std::cos(yaw) };

        // Sliding the camera sideways, rather than turning it, moves the focus on screen without tilting the view
        const float halfHeight = std::max(1.0f, (float)GetScreenHeight() * 0.5f);
        const float viewHalfHeight = distance * std::tan(camera.fovy * DEG2RAD * 0.5f);
        const float slide = leftInset * 0.5f / halfHeight * viewHalfHeight;
        const Vector3 right = { std::cos(yaw), 0.0f, -std::sin(yaw) };
        const Vector3 target = { focus.x - right.x * slide, focus.y, focus.z - right.z * slide };

        camera.target = target;
        camera.position = { target.x + offset.x, target.y + offset.y, target.z + offset.z };
        camera.up = { 0.0f, 1.0f, 0.0f };
        camera.projection = CAMERA_PERSPECTIVE;
    }
}
