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
        static constexpr int MAX_DISCS = 4;

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
            bool sunDisc = true; // off where the Sun is drawn as a body of its own, as on the map

            // Bodies drawn between the stars and the air, seen through it: the way to each, and the cosine of
            // half its size across. The air leaves them showing where it would hide the stars.
            Vector4 discs[MAX_DISCS]{};
            int discCount = 0;
        };

        void init();
        void unload();

        // The stars then the air; drawStars and drawAir split it for bodies to go between
        void draw(const View& view) const;
        void drawStars(const View& view) const;
        void drawAir(const View& view) const;

    private:
        Model mCube{};
        Shader mStarShader{};
        Shader mAirShader{};
        Texture2D mStars{};
        bool mReady = false;
    };
}
