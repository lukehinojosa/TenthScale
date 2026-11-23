#include "engine/Sky.h"
#include "engine/ShaderUtils.h"

#include "shaders/sky.frag.h"
#include "shaders/sky.vert.h"

#include <raymath.h>
#include <rlgl.h>

namespace engine
{
    namespace
    {
        Vector3 toVector(Color color)
        {
            return { color.r / 255.0f, color.g / 255.0f, color.b / 255.0f };
        }
    }

    bool Sky::init()
    {
        mShader = loadPortableShader(shaders::SKY_VERT, shaders::SKY_FRAG);
        if (mShader.id == rlGetShaderIdDefault())
        {
            return false;
        }
        mShader.locs[SHADER_LOC_MATRIX_VIEW] = GetShaderLocation(mShader, "matView");
        mShader.locs[SHADER_LOC_MATRIX_PROJECTION] = GetShaderLocation(mShader, "matProjection");
        mLocSun = GetShaderLocation(mShader, "sunDirection");
        mLocColors = GetShaderLocation(mShader, "skyColors");

        mCube = LoadModelFromMesh(GenMeshCube(2.0f, 2.0f, 2.0f));
        mCube.materials[0].shader = mShader;
        return true;
    }

    void Sky::unload()
    {
        if (mCube.meshCount > 0)
        {
            mCube.materials[0].shader = Shader{}; // The shader is unloaded separately below
            UnloadModel(mCube);
            mCube = Model{};
        }
        if (mShader.id != 0)
        {
            UnloadShader(mShader);
            mShader = Shader{};
        }
    }

    void Sky::draw(Vector3 sunDirection)
    {
        Vector3 sun = Vector3Normalize(sunDirection);
        Vector3 colors[6] = { toVector(dayZenith), toVector(dayHorizon), toVector(sunsetHorizon), toVector(nightZenith), toVector(nightHorizon), toVector(ground) };
        SetShaderValue(mShader, mLocSun, &sun, SHADER_UNIFORM_VEC3);
        SetShaderValueV(mShader, mLocColors, colors, SHADER_UNIFORM_VEC3, 6);

        // Seen from inside the cube, so its faces are back faces; no depth write so the scene draws over it
        rlDisableBackfaceCulling();
        rlDisableDepthMask();
        DrawModel(mCube, Vector3Zero(), 1.0f, WHITE);
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        rlEnableBackfaceCulling();
    }
}
