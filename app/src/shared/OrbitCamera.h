#pragma once

#include <raylib.h>

namespace tenth
{
    // Circles a focus point: right-drag to orbit, wheel to zoom, middle-drag or W/S to move the focus up and down
    struct OrbitCamera
    {
        Vector3 focus = { 0.0f, 2.0f, 0.0f };
        float distance = 12.0f;
        float yaw = 0.8f; // radians around the vertical axis
        float pitch = 0.35f; // radians above the horizon

        float minDistance = 2.0f;
        float maxDistance = 80.0f;
        float minPitch = -0.05f;
        float maxPitch = 1.5f;
        float minFocusHeight = 0.0f;
        float maxFocusHeight = 60.0f;
        float minCameraHeight = -1.0e9f; // the camera tilts no lower or higher than this, to stay inside a room
        float maxCameraHeight = 1.0e9f;

        float orbitSensitivity = 0.006f; // radians per pixel
        float zoomStep = 0.12f; // share of the distance per wheel notch
        float liftSpeed = 0.6f; // share of the distance per second with W/S

        // Pixels panels cover on each side, so the focus shows in the middle of what's left of the screen
        float leftInset = 0.0f;
        float rightInset = 0.0f;

        void update(Camera3D& camera, float dt);

        // Points the camera without reading input, such as when a scene starts
        void apply(Camera3D& camera) const;

    private:
        bool mOrbiting = false;
        bool mLifting = false;
    };
}
