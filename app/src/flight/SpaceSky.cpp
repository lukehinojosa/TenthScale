#include "flight/SpaceSky.h"

#include <string>

#include <engine/ShaderUtils.h>
#include <raymath.h>
#include <rlgl.h>

#include "shaders/air.frag.h"
#include "shaders/atmosphere.frag.h"
#include "shaders/sky.vert.h"
#include "shaders/stars.frag.h"

namespace tenth
{
    namespace
    {
        constexpr int WEB_PANORAMA_WIDTH = 4096;

        void setVector(Shader shader, const char* name, Vector3 value)
        {
            SetShaderValue(shader, GetShaderLocation(shader, name), &value, SHADER_UNIFORM_VEC3);
        }

        Vector3 toVector(const Vec3d& v)
        {
            return { (float)v.x, (float)v.y, (float)v.z };
        }

        Shader loadCubeShader(const std::string& fragment)
        {
            Shader shader = engine::loadPortableShader(shaders::SKY_VERT, fragment.c_str());
            shader.locs[SHADER_LOC_MATRIX_VIEW] = GetShaderLocation(shader, "matView");
            shader.locs[SHADER_LOC_MATRIX_PROJECTION] = GetShaderLocation(shader, "matProjection");
            return shader;
        }
    }

    void SpaceSky::init()
    {
        const char* path = "assets/textures/eso0932a.jpg";
        Image image = FileExists(path) ? LoadImage(path) : Image{};
        if (image.data == nullptr)
        {
            image = GenImageColor(1, 1, BLACK);
        }
#ifdef __EMSCRIPTEN__
        if (image.width > WEB_PANORAMA_WIDTH)
        {
            ImageResize(&image, WEB_PANORAMA_WIDTH, WEB_PANORAMA_WIDTH * image.height / image.width);
        }
#endif
        mStars = LoadTextureFromImage(image);
        UnloadImage(image);
        SetTextureFilter(mStars, TEXTURE_FILTER_BILINEAR);

        mStarShader = loadCubeShader(shaders::STARS_FRAG);
        mAirShader = loadCubeShader(std::string(shaders::ATMOSPHERE_FRAG) + shaders::AIR_FRAG);
        mCube = LoadModelFromMesh(GenMeshCube(2.0f, 2.0f, 2.0f));
        mCube.materials[0].maps[MATERIAL_MAP_ALBEDO].texture = mStars;
        mReady = true;
    }

    void SpaceSky::unload()
    {
        if (!mReady)
        {
            return;
        }
        UnloadModel(mCube); // leaves the shaders and texture, which go below
        UnloadShader(mStarShader);
        UnloadShader(mAirShader);
        UnloadTexture(mStars);
        mReady = false;
    }

    void SpaceSky::draw(const View& view) const
    {
        if (!mReady)
        {
            return;
        }
        setVector(mStarShader, "bodyEast", toVector(view.axes.east));
        setVector(mStarShader, "bodyUp", toVector(view.axes.up));
        setVector(mStarShader, "bodySouth", toVector(view.axes.south));
        setVector(mAirShader, "planetCentre", view.centre);
        setVector(mAirShader, "sunDirection", Vector3Normalize(view.sun));
        setVector(mAirShader, "cameraPosition", view.camera);
        const float world[4] = { (float)view.radius, (float)view.atmosphereHeight, (float)view.cameraHeight, 0.0f };
        SetShaderValue(mAirShader, GetShaderLocation(mAirShader, "world"), world, SHADER_UNIFORM_VEC4);

        // Seen from inside the cube, so its faces are back faces; nothing writes depth, so the scene draws over it
        rlDrawRenderBatchActive();
        rlDisableBackfaceCulling();
        rlDisableDepthMask();
        mCube.materials[0].shader = mStarShader;
        DrawModel(mCube, Vector3Zero(), 1.0f, WHITE);
        rlSetBlendFactors(RL_ONE, RL_SRC_ALPHA, RL_FUNC_ADD);
        BeginBlendMode(BLEND_CUSTOM);
        mCube.materials[0].shader = mAirShader;
        DrawModel(mCube, Vector3Zero(), 1.0f, WHITE);
        EndBlendMode();
        rlEnableDepthMask();
        rlEnableBackfaceCulling();
    }
}
