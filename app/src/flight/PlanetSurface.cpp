#include "flight/PlanetSurface.h"

#include <string>

#include <engine/ShaderUtils.h>
#include <raymath.h>
#include <rlgl.h>

#include "flight/Globe.h"
#include "shaders/atmosphere.frag.h"
#include "shaders/planet.frag.h"
#include "shaders/planet.vert.h"

namespace tenth
{
    namespace
    {
        constexpr int SPHERE_RINGS = 128;
        constexpr int SPHERE_SEGMENTS = 256;
        constexpr Color FLAT = { 128, 128, 255, 255 }; // a relief map's colour for level ground

        Shader loadShader()
        {
            const std::string fragment = std::string(shaders::ATMOSPHERE_FRAG) + shaders::PLANET_FRAG;
            Shader shader = engine::loadPortableShader(shaders::PLANET_VERT, fragment.c_str());
            shader.locs[SHADER_LOC_MATRIX_MVP] = GetShaderLocation(shader, "mvp");
            shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");
            shader.locs[SHADER_LOC_MAP_ROUGHNESS] = GetShaderLocation(shader, "texture3");
            shader.locs[SHADER_LOC_MAP_OCCLUSION] = GetShaderLocation(shader, "texture4");
            return shader;
        }

        void setVector(Shader shader, const char* name, Vector3 value)
        {
            SetShaderValue(shader, GetShaderLocation(shader, name), &value, SHADER_UNIFORM_VEC3);
        }

        Vector3 toVector(const Vec3d& v)
        {
            return { (float)v.x, (float)v.y, (float)v.z };
        }

        Vector3 toVector(Color color)
        {
            return { color.r / 255.0f, color.g / 255.0f, color.b / 255.0f };
        }
    }

    void PlanetSurface::init(const SurfaceFrame& site, double radius, double atmosphereHeight, Color localGround)
    {
        mSite = site;
        mRadius = radius;
        mAtmosphereHeight = atmosphereHeight;
        mLocalGround = localGround;
        mSphere = makeGlobeMesh(SPHERE_RINGS, SPHERE_SEGMENTS);
        mGround = makeGroundPatch(site, radius);
        mDay = loadSurfaceMap("assets/textures/earth_daymap.jpg", localGround, false);
        mNight = loadSurfaceMap("assets/textures/earth_nightmap.jpg", BLACK, false);
        mClouds = loadSurfaceMap("assets/textures/earth_clouds.jpg", BLACK, true);
        mRelief = loadSurfaceMap("assets/textures/earth_normal_map.jpg", FLAT, false);
        mWater = loadSurfaceMap("assets/textures/earth_specular_map.png", BLACK, true);

        mSurfaceShader = loadShader();
        mSurface = LoadMaterialDefault();
        mSurface.shader = mSurfaceShader;
        mSurface.maps[MATERIAL_MAP_ALBEDO].texture = mDay;
        mSurface.maps[MATERIAL_MAP_METALNESS].texture = mNight;
        mSurface.maps[MATERIAL_MAP_NORMAL].texture = mRelief;
        mSurface.maps[MATERIAL_MAP_ROUGHNESS].texture = mClouds;
        mSurface.maps[MATERIAL_MAP_OCCLUSION].texture = mWater;
        mReady = true;
    }

    void PlanetSurface::unload()
    {
        if (!mReady)
        {
            return;
        }
        UnloadMaterial(mSurface); // the shader and maps go with it
        UnloadMesh(mSphere);
        UnloadMesh(mGround);
        mReady = false;
    }

    void PlanetSurface::draw(const View& view) const
    {
        if (!mReady)
        {
            return;
        }
        const Vector3 sitePosition = toVector(view.axes.rotate(mSite.origin)) + view.centre;
        setVector(mSurfaceShader, "planetCentre", view.centre);
        setVector(mSurfaceShader, "sunDirection", Vector3Normalize(view.sun));
        setVector(mSurfaceShader, "cameraPosition", view.camera);
        setVector(mSurfaceShader, "sitePosition", sitePosition);
        setVector(mSurfaceShader, "localColor", toVector(mLocalGround));
        setVector(mSurfaceShader, "pole", toVector(view.axes.rotate({ 0.0, 1.0, 0.0 })));
        const float world[4] = { (float)mRadius, (float)mAtmosphereHeight, (float)view.cameraHeight, 0.0f };
        SetShaderValue(mSurfaceShader, GetShaderLocation(mSurfaceShader, "world"), world, SHADER_UNIFORM_VEC4);

        // Turns the body's axes into the scene's, scaled out to a size and moved to the body's centre
        const auto placed = [&](double scale)
        {
            const Vec3d e = view.axes.east * scale;
            const Vec3d u = view.axes.up * scale;
            const Vec3d s = view.axes.south * scale;
            return Matrix{ (float)e.x, (float)e.y, (float)e.z, view.centre.x, (float)u.x, (float)u.y, (float)u.z, view.centre.y,
                           (float)s.x, (float)s.y, (float)s.z, view.centre.z, 0.0f, 0.0f, 0.0f, 1.0f };
        };

        if (!view.ground)
        {
            DrawMesh(mSphere, mSurface, placed(mRadius));
            return;
        }
        rlDrawRenderBatchActive();
        rlDisableDepthTest();
        DrawMesh(mSphere, mSurface, placed(mRadius));
        rlEnableDepthTest();
        DrawMesh(mGround, mSurface, MatrixTranslate(sitePosition.x, sitePosition.y, sitePosition.z));
    }
}
