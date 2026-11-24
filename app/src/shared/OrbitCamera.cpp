#include "shared/OrbitCamera.h"

#include <algorithm>
#include <cmath>

#include <imgui.h>
#include <raymath.h>

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

        // Left upside down by free tilt, it's the same spot seen from the far side, so it's turned the right way up.
        // This comes before the drag, so a fast drag past the limit clamps instead of flipping round.
        if (!freeTilt && std::cos(pitch) < 0.0f)
        {
            pitch = std::remainder(PI - pitch, 2.0f * PI);
            yaw += PI;
        }

        const Vector2 mouseDelta = GetMouseDelta();
        if (mOrbiting)
        {
            // Upside down, the screen's left and right are swapped, so turning is too
            const float turn = std::cos(pitch) < 0.0f ? -1.0f : 1.0f;
            yaw -= mouseDelta.x * orbitSensitivity * turn;
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
        if (liftWithKeys && !imguiOwnsKeyboard)
        {
            const float lift = (float)IsKeyDown(KEY_W) - (float)IsKeyDown(KEY_S);
            focus.y += lift * liftSpeed * distance * dt;
        }

        distance = std::clamp(distance, minDistance, maxDistance);
        focus.y = std::clamp(focus.y, minFocusHeight, maxFocusHeight);

        if (freeTilt)
        {
            pitch = std::remainder(pitch, 2.0f * PI);
            apply(camera);
            return;
        }

        // The tilt that would put the camera at the floor or ceiling height becomes the limit
        const float lowest = std::asin(std::clamp((minCameraHeight - focus.y) / distance, -1.0f, 1.0f));
        const float highest = std::asin(std::clamp((maxCameraHeight - focus.y) / distance, -1.0f, 1.0f));
        // A floor so high that even looking straight down can't clear it leaves the steepest tilt allowed
        const float ceiling = std::min(maxPitch, highest);
        pitch = std::clamp(pitch, std::min(std::max(minPitch, lowest), ceiling), ceiling);
        apply(camera);
    }

    void OrbitCamera::apply(Camera3D& camera) const
    {
        const float flat = distance * std::cos(pitch);
        const Vector3 offset = { flat * std::sin(yaw), distance * std::sin(pitch), flat * std::cos(yaw) };

        // Sliding the camera sideways, rather than turning it, moves the focus on screen without tilting the view
        const float halfHeight = std::max(1.0f, (float)GetScreenHeight() * 0.5f);
        const float viewHalfHeight = distance * std::tan(camera.fovy * DEG2RAD * 0.5f);
        const float slide = (leftInset - rightInset) * 0.5f / halfHeight * viewHalfHeight;
        const Vector3 right = { std::cos(yaw), 0.0f, -std::sin(yaw) };
        const Vector3 target = { focus.x - right.x * slide, focus.y, focus.z - right.z * slide };

        camera.target = Vector3RotateByQuaternion(target, frame);
        camera.position = Vector3RotateByQuaternion({ target.x + offset.x, target.y + offset.y, target.z + offset.z }, frame);
        // Up tips over with the tilt, so the view rolls smoothly through straight up and down
        const Vector3 up = { -std::sin(pitch) * std::sin(yaw), std::cos(pitch), -std::sin(pitch) * std::cos(yaw) };
        camera.up = Vector3RotateByQuaternion(up, frame);
        camera.projection = CAMERA_PERSPECTIVE;
    }
}
