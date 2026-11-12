#pragma once

#include <raylib.h>

namespace engine
{
    // Procedural sky dome: zenith-to-horizon gradient that shifts with the sun's height, plus a sun disc and glow.
    // Draw it first inside BeginMode3D; it writes no depth, so everything else draws over it.
    class Sky
    {
    public:
        Color dayZenith = { 70, 130, 215, 255 };
        Color dayHorizon = { 190, 215, 240, 255 };
        Color sunsetHorizon = { 245, 150, 90, 255 };
        Color nightZenith = { 5, 8, 20, 255 };
        Color nightHorizon = { 20, 28, 50, 255 };
        Color ground = { 45, 45, 50, 255 };

        bool init();
        void unload();

        // sunDirection points toward the sun
        void draw(Vector3 sunDirection);

    private:
        Shader mShader{};
        Model mCube{};
        int mLocSun = -1;
        int mLocColors = -1;
    };
}
