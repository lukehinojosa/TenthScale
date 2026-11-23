#pragma once

#include <raylib.h>

#include "flight/SurfaceFrame.h"

namespace tenth
{
    // Earth drawn from its day, night, cloud, relief, and water maps, seen through the air between it and the eye: a
    // whole-planet sphere for seeing it from afar, and a finer patch of ground around the launch site for standing on.
    class PlanetSurface
    {
    public:
        // What the planet needs to know about the scene each frame, all in the scene's axes
        struct View
        {
            SurfaceFrame axes; // the body's axes as the scene sees them: its centre at the origin, its x along east
            Vector3 centre;
            Vector3 camera;
            Vector3 sun;
            double cameraHeight = 0.0; // above the surface, worked out in doubles where floats would lose it
        };

        // Loads the maps and builds the meshes for a body of this radius with the ground patch centred on site.
        // Any missing map is replaced by a flat colour, so the planet still draws.
        void init(const SurfaceFrame& site, double radius, double atmosphereHeight, Color localGround);
        void unload();

        // The sphere draws without depth so the patch, and everything after, cover it
        void draw(const View& view) const;

    private:
        Mesh mSphere{};
        Mesh mGround{};
        Material mSurface{};
        Texture2D mDay{};
        Texture2D mNight{};
        Texture2D mClouds{};
        Texture2D mRelief{};
        Texture2D mWater{};
        Shader mSurfaceShader{};
        SurfaceFrame mSite;
        double mRadius = 0.0;
        double mAtmosphereHeight = 0.0;
        Color mLocalGround{};
        bool mReady = false;
    };
}
