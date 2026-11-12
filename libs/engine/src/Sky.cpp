#include "engine/Sky.h"
#include "engine/ShaderUtils.h"

#include <raymath.h>
#include <rlgl.h>

namespace engine
{
    namespace
    {
        const char* SKY_VS = R"(
in vec3 vertexPosition;

uniform mat4 matView;
uniform mat4 matProjection;

out vec3 viewDirection;

void main()
{
    viewDirection = vertexPosition;
    // Rotation only, so the sky stays centered on the camera; z = w puts it on the far plane
    mat4 rotationOnly = mat4(mat3(matView));
    vec4 clip = matProjection * rotationOnly * vec4(vertexPosition, 1.0);
    gl_Position = clip.xyww;
}
)";

        const char* SKY_FS = R"(
in vec3 viewDirection;

uniform vec3 sunDirection;
uniform vec3 skyColors[6]; // dayZenith, dayHorizon, sunsetHorizon, nightZenith, nightHorizon, ground

out vec4 finalColor;

void main()
{
    vec3 dir = normalize(viewDirection);
    vec3 sun = normalize(sunDirection);

    // 1 at noon, 0 once the sun is a little below the horizon
    float day = smoothstep(-0.15, 0.35, sun.y);
    // Peaks while the sun is near the horizon
    float sunset = (1.0 - abs(sun.y) * 3.0);
    sunset = clamp(sunset, 0.0, 1.0) * smoothstep(-0.2, 0.05, sun.y);

    vec3 zenith = mix(skyColors[3], skyColors[0], day);
    vec3 horizon = mix(skyColors[4], skyColors[1], day);
    // Sunset tint is strongest on the side of the sky facing the sun
    float facingSun = max(dot(normalize(vec3(dir.x, 0.0, dir.z)), normalize(vec3(sun.x, 0.0, sun.z))), 0.0);
    horizon = mix(horizon, skyColors[2], sunset * (0.35 + 0.65 * facingSun));

    float height = max(dir.y, 0.0);
    vec3 color = mix(horizon, zenith, pow(height, 0.45));
    if (dir.y < 0.0)
    {
        color = mix(horizon, skyColors[5] * (0.3 + 0.7 * day), smoothstep(0.0, 0.08, -dir.y));
    }

    float angle = acos(clamp(dot(dir, sun), -1.0, 1.0));
    float disc = 1.0 - smoothstep(0.02, 0.03, angle);
    float glow = pow(max(dot(dir, sun), 0.0), 64.0) * 0.6;
    float sunVisible = smoothstep(-0.05, 0.02, sun.y);
    color += vec3(1.0, 0.92, 0.75) * (disc * 2.0 + glow) * sunVisible;

    finalColor = vec4(color, 1.0);
}
)";

        Vector3 toVector(Color color)
        {
            return { color.r / 255.0f, color.g / 255.0f, color.b / 255.0f };
        }
    }

    bool Sky::init()
    {
        mShader = loadPortableShader(SKY_VS, SKY_FS);
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
