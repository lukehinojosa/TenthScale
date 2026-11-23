#pragma once

#include <raylib.h>

#include "flight/SurfaceFrame.h"

namespace tenth
{
    // Everything beyond the planet: the Milky Way panorama on the far plane, with the planet's air drawn over it.
    // The air is a shell round the planet that every view ray is traced through, so from the ground it's the blue
    // sky, it thins to black as the camera climbs out of it, and from space it's a glow round the planet's edge.
    class SpaceSky
    {
    public:
        // The scene as the sky needs it, all in the scene's axes
        struct View
        {
            SurfaceFrame axes; // the body's axes as the scene sees them
            Vector3 centre; // the planet's
            Vector3 camera;
            Vector3 sun;
            double radius = 0.0;
            double atmosphereHeight = 0.0;
            double cameraHeight = 0.0; // above the surface, worked out in doubles where floats would lose it
        };

        void init();
        void unload();
        void draw(const View& view) const;

    private:
        Model mCube{};
        Shader mStarShader{};
        Shader mAirShader{};
        Texture2D mStars{};
        bool mReady = false;
    };
}
