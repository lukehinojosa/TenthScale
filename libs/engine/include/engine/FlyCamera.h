#pragma once

#include <raylib.h>

namespace engine
{
    // Hold right mouse to look; WASD to move, Q/E down and up, shift to go faster, wheel to zoom
    struct FlyCamera
    {
        float moveSpeed = 10.0f;
        float lookSensitivity = 0.2f;
        float sprintMultiplier = 3.0f;

        void update(Camera3D& camera, float dt);
    };
}
