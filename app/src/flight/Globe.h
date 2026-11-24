#pragma once

#include <vector>

#include <raylib.h>

#include "flight/SurfaceFrame.h"
#include "shared/SolarSystem.h"

namespace tenth
{
    // A unit sphere in a body's axes, its maps' left edge on the far side from +X. The seam is doubled so the maps
    // wrap round without a stitch.
    Mesh makeGlobeMesh(int rings, int segments);

    // A patch of the curved surface around a site, in the site's axes, with rings spaced closer near the middle where
    // the camera is. A whole-body sphere is too coarse to stand on, so this covers everything in sight there.
    Mesh makeGroundPatch(const SurfaceFrame& site, double radius);

    // A map of a body's surface, or a flat colour when the file is missing so the body still draws. Browsers get
    // smaller maps to stay within their memory.
    Texture2D loadSurfaceMap(const char* path, Color fallback, bool grey);

    // How a body's surface map is turned in the Solar System's axes. A moon keeps the middle of its map toward its
    // planet; anything else keeps the axes as they are.
    struct BodyAxes
    {
        Vec3d x = { 1.0, 0.0, 0.0 };
        Vec3d y = { 0.0, 1.0, 0.0 };
        Vec3d z = { 0.0, 0.0, 1.0 };

        Vec3d toBody(const Vec3d& v) const { return { v.dot(x), v.dot(y), v.dot(z) }; }
        Vec3d fromBody(const Vec3d& v) const { return x * v.x + y * v.y + z * v.z; }
    };
    BodyAxes bodyAxes(const SolarSystem& system, BodyId id, double time);

    // A model matrix carrying unit x, y, and z onto these three directions and the origin to a point
    Matrix placedMatrix(const Vec3d& x, const Vec3d& y, const Vec3d& z, Vector3 at);

    // Every body but Earth drawn from its surface map, named after it as in moon.jpg, lit from the Sun or glowing if
    // it is a star. Earth has its own PlanetSurface with clouds, lights, and air. One fine patch of ground follows
    // whatever spot it's put under, for standing on a body without air.
    class BodyGlobes
    {
    public:
        void init(const SolarSystem& system);
        void unload();

        // Earth, drawn by PlanetSurface instead
        bool usesPlanet(BodyId id) const;

        // placed carries the unit sphere onto the body; sun is the way to the Sun in the same axes
        void drawGlobe(BodyId id, const Matrix& placed, Vector3 sun) const;

        // Builds the patch under position, given in the body's own axes, unless its middle is within slack of there
        void placeGround(BodyId id, const Vec3d& position, double radius, double slack);
        BodyId groundBody() const { return mGroundBody; }
        const SurfaceFrame& groundSite() const { return mGroundSite; }

        // placed carries the patch's site axes onto the scene
        void drawGround(const Matrix& placed, Vector3 sun) const;

    private:
        struct Map
        {
            Texture2D texture{};
            bool loaded = false;
            bool star = false;
        };

        void useMap(BodyId id, Vector3 sun) const;

        std::vector<Map> mMaps;
        Mesh mSphere{};
        Material mMaterial{};
        Shader mShader{};
        Mesh mGround{};
        BodyId mGroundBody = NO_BODY;
        SurfaceFrame mGroundSite;
        bool mReady = false;
    };
}
