#include "engine/Lighting.h"
#include "engine/ShaderUtils.h"

#include <algorithm>
#include <cmath>

#include <raymath.h>
#include <rlgl.h>

#ifdef __EMSCRIPTEN__
#include <GLES3/gl3.h>
#endif

namespace engine
{
    namespace
    {
        unsigned int loadShadowDepthTexture(int size)
        {
#ifdef __EMSCRIPTEN__
            // raylib 5.5 asks WebGL2 for an unsized depth format, which WebGL2 rejects, so make it here
            GLuint id = 0;
            glGenTextures(1, &id);
            glBindTexture(GL_TEXTURE_2D, id);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, size, size, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glBindTexture(GL_TEXTURE_2D, 0);
            return id;
#else
            return rlLoadTextureDepth(size, size, false);
#endif
        }

        // Above raylib's own texture slots, so materials never overwrite it
        constexpr int SHADOW_MAP_SLOT = 10;

        const char* LIT_VS = R"(
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;
in vec4 vertexColor;

uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
uniform mat4 lightViewProj;

out vec3 fragPosition;
out vec2 fragTexCoord;
out vec4 fragColor;
out vec3 fragNormal;
out vec4 fragPosLightSpace;

void main()
{
    vec4 world = matModel * vec4(vertexPosition, 1.0);
    fragPosition = world.xyz;
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;
    fragNormal = normalize(vec3(matNormal * vec4(vertexNormal, 0.0)));
    fragPosLightSpace = lightViewProj * world;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)";

        const char* LIT_FS = R"(
#define MAX_POINT_LIGHTS 8

in vec3 fragPosition;
in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;
in vec4 fragPosLightSpace;

uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform vec3 viewPos;

uniform vec3 sunDirection;
uniform vec3 sunColor;
uniform float ambient;
uniform float specularStrength;
uniform float shininess;

uniform int pointLightCount;
uniform vec3 pointLightPosition[MAX_POINT_LIGHTS];
uniform vec3 pointLightColor[MAX_POINT_LIGHTS];
uniform float pointLightRange[MAX_POINT_LIGHTS];

uniform sampler2D shadowMap;
uniform int shadowsEnabled;
uniform float shadowTexel;

out vec4 finalColor;

float sunShadow(vec3 normal, vec3 lightDir)
{
    if (shadowsEnabled == 0)
    {
        return 0.0;
    }
    vec3 coords = fragPosLightSpace.xyz / fragPosLightSpace.w * 0.5 + 0.5;
    if (coords.z > 1.0 || coords.x < 0.0 || coords.x > 1.0 || coords.y < 0.0 || coords.y > 1.0)
    {
        return 0.0;
    }

    // Steeper light angles need more bias to avoid acne
    float bias = max(0.004 * (1.0 - dot(normal, lightDir)), 0.0008);
    float shadow = 0.0;
    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            float closest = texture(shadowMap, coords.xy + vec2(float(x), float(y)) * shadowTexel).r;
            shadow += coords.z - bias > closest ? 1.0 : 0.0;
        }
    }
    return shadow / 9.0;
}

vec3 blinnPhong(vec3 normal, vec3 lightDir, vec3 viewDir, vec3 color)
{
    float diffuse = max(dot(normal, lightDir), 0.0);
    vec3 halfway = normalize(lightDir + viewDir);
    float specular = pow(max(dot(normal, halfway), 0.0), shininess) * specularStrength;
    return color * (diffuse + specular);
}

void main()
{
    vec4 base = texture(texture0, fragTexCoord) * colDiffuse * fragColor;
    vec3 normal = normalize(fragNormal);
    vec3 viewDir = normalize(viewPos - fragPosition);

    vec3 sunDir = normalize(sunDirection);
    vec3 light = ambient * sunColor;
    light += (1.0 - sunShadow(normal, sunDir)) * blinnPhong(normal, sunDir, viewDir, sunColor);

    for (int i = 0; i < MAX_POINT_LIGHTS; i++)
    {
        if (i >= pointLightCount)
        {
            break;
        }
        vec3 toLight = pointLightPosition[i] - fragPosition;
        float distance = length(toLight);
        // Smooth falloff that reaches zero at the light's range
        float falloff = clamp(1.0 - distance / pointLightRange[i], 0.0, 1.0);
        falloff *= falloff;
        light += falloff * blinnPhong(normal, toLight / distance, viewDir, pointLightColor[i]);
    }

    finalColor = vec4(base.rgb * light, base.a);
}
)";

        Vector3 colorToVector(Color color, float intensity)
        {
            return { color.r / 255.0f * intensity, color.g / 255.0f * intensity, color.b / 255.0f * intensity };
        }
    }

    bool Lighting::init(int shadowMapSize)
    {
        mShader = loadPortableShader(LIT_VS, LIT_FS);
        if (mShader.id == rlGetShaderIdDefault())
        {
            return false;
        }

        mShader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(mShader, "matModel");
        mShader.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(mShader, "matNormal");
        mLocViewPos = GetShaderLocation(mShader, "viewPos");
        mLocSunDirection = GetShaderLocation(mShader, "sunDirection");
        mLocSunColor = GetShaderLocation(mShader, "sunColor");
        mLocAmbient = GetShaderLocation(mShader, "ambient");
        mLocSpecular = GetShaderLocation(mShader, "specularStrength");
        mLocShininess = GetShaderLocation(mShader, "shininess");
        mLocPointCount = GetShaderLocation(mShader, "pointLightCount");
        mLocPointPosition = GetShaderLocation(mShader, "pointLightPosition");
        mLocPointColor = GetShaderLocation(mShader, "pointLightColor");
        mLocPointRange = GetShaderLocation(mShader, "pointLightRange");
        mLocLightViewProj = GetShaderLocation(mShader, "lightViewProj");
        mLocShadowMap = GetShaderLocation(mShader, "shadowMap");
        mLocShadowsEnabled = GetShaderLocation(mShader, "shadowsEnabled");
        mLocShadowTexel = GetShaderLocation(mShader, "shadowTexel");

        mLightViewProj = MatrixIdentity();
        if (shadowMapSize <= 0)
        {
            return true;
        }

        // Depth-only framebuffer: the light's view needs depth and nothing else
        mShadowMapSize = shadowMapSize;
        mShadowMap.id = rlLoadFramebuffer();
        mShadowMap.texture.width = shadowMapSize;
        mShadowMap.texture.height = shadowMapSize;
        mShadowMap.depth.id = loadShadowDepthTexture(shadowMapSize);
        mShadowMap.depth.width = shadowMapSize;
        mShadowMap.depth.height = shadowMapSize;
        mShadowMap.depth.mipmaps = 1;
        rlFramebufferAttach(mShadowMap.id, mShadowMap.depth.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
        mHasShadowMap = rlFramebufferComplete(mShadowMap.id);
        if (!mHasShadowMap)
        {
            TraceLog(LOG_WARNING, "LIGHTING: shadow framebuffer incomplete, shadows disabled");
        }
        return true;
    }

    void Lighting::unload()
    {
        if (mShadowMap.id != 0)
        {
            rlUnloadFramebuffer(mShadowMap.id);
            rlUnloadTexture(mShadowMap.depth.id);
            mShadowMap = RenderTexture2D{};
        }
        if (mShader.id != 0)
        {
            UnloadShader(mShader);
            mShader = Shader{};
        }
        mHasShadowMap = false;
    }

    bool Lighting::beginShadowPass(Vector3 focus)
    {
        if (!mHasShadowMap || !shadowsEnabled)
        {
            return false;
        }

        // Unbind the shadow map first: sampling a texture while rendering into it is a feedback loop (an error on WebGL)
        rlActiveTextureSlot(SHADOW_MAP_SLOT);
        rlDisableTexture();
        rlActiveTextureSlot(0);

        Vector3 toSun = Vector3Normalize(sun.direction);
        Camera3D lightCamera{};
        lightCamera.target = focus;
        lightCamera.position = Vector3Add(focus, Vector3Scale(toSun, shadowExtent * 2.0f));
        lightCamera.up = (std::fabs(toSun.y) > 0.99f) ? Vector3{ 0.0f, 0.0f, 1.0f } : Vector3{ 0.0f, 1.0f, 0.0f };
        lightCamera.fovy = shadowExtent * 2.0f;
        lightCamera.projection = CAMERA_ORTHOGRAPHIC;

        BeginTextureMode(mShadowMap);
        ClearBackground(WHITE);
        BeginMode3D(lightCamera);
        mLightViewProj = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
        return true;
    }

    void Lighting::endShadowPass()
    {
        EndMode3D();
        EndTextureMode();
    }

    void Lighting::apply(const Camera3D& camera)
    {
        SetShaderValue(mShader, mLocViewPos, &camera.position, SHADER_UNIFORM_VEC3);

        Vector3 sunDirection = Vector3Normalize(sun.direction);
        Vector3 sunColor = colorToVector(sun.color, sun.intensity);
        SetShaderValue(mShader, mLocSunDirection, &sunDirection, SHADER_UNIFORM_VEC3);
        SetShaderValue(mShader, mLocSunColor, &sunColor, SHADER_UNIFORM_VEC3);
        SetShaderValue(mShader, mLocAmbient, &ambient, SHADER_UNIFORM_FLOAT);
        SetShaderValue(mShader, mLocSpecular, &specularStrength, SHADER_UNIFORM_FLOAT);
        SetShaderValue(mShader, mLocShininess, &shininess, SHADER_UNIFORM_FLOAT);

        int count = std::min((int)pointLights.size(), MAX_POINT_LIGHTS);
        Vector3 positions[MAX_POINT_LIGHTS]{};
        Vector3 colors[MAX_POINT_LIGHTS]{};
        float ranges[MAX_POINT_LIGHTS]{};
        for (int i = 0; i < count; i++)
        {
            positions[i] = pointLights[i].position;
            colors[i] = colorToVector(pointLights[i].color, pointLights[i].intensity);
            ranges[i] = std::max(pointLights[i].range, 0.01f);
        }
        SetShaderValue(mShader, mLocPointCount, &count, SHADER_UNIFORM_INT);
        SetShaderValueV(mShader, mLocPointPosition, positions, SHADER_UNIFORM_VEC3, MAX_POINT_LIGHTS);
        SetShaderValueV(mShader, mLocPointColor, colors, SHADER_UNIFORM_VEC3, MAX_POINT_LIGHTS);
        SetShaderValueV(mShader, mLocPointRange, ranges, SHADER_UNIFORM_FLOAT, MAX_POINT_LIGHTS);

        int useShadows = (mHasShadowMap && shadowsEnabled) ? 1 : 0;
        SetShaderValue(mShader, mLocShadowsEnabled, &useShadows, SHADER_UNIFORM_INT);
        SetShaderValueMatrix(mShader, mLocLightViewProj, mLightViewProj);
        if (useShadows)
        {
            float texel = 1.0f / (float)mShadowMapSize;
            int slot = SHADOW_MAP_SLOT;
            SetShaderValue(mShader, mLocShadowTexel, &texel, SHADER_UNIFORM_FLOAT);
            SetShaderValue(mShader, mLocShadowMap, &slot, SHADER_UNIFORM_INT);
            rlActiveTextureSlot(SHADOW_MAP_SLOT);
            rlEnableTexture(mShadowMap.depth.id);
            rlActiveTextureSlot(0);
        }
    }
}
