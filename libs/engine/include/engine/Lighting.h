#pragma once

#include <vector>

#include <raylib.h>

namespace engine
{
    struct PointLight
    {
        Vector3 position = { 0.0f, 3.0f, 0.0f };
        Color color = WHITE;
        float intensity = 1.0f;
        float range = 15.0f; // Distance where the light has faded to about 1%
    };

    struct SunLight
    {
        Vector3 direction = { 0.4f, 1.0f, 0.3f }; // Points toward the sun
        Color color = { 255, 244, 214, 255 };
        float intensity = 1.0f;
    };

    // Blinn-Phong lighting with one sun, up to MAX_POINT_LIGHTS point lights, and a PCF shadow map from the sun.
    // Each frame: renderShadows(...) with a function that draws the shadow casters, then apply(camera), then draw.
    class Lighting
    {
    public:
        static constexpr int MAX_POINT_LIGHTS = 8;

        SunLight sun;
        std::vector<PointLight> pointLights;
        float ambient = 0.25f;
        float specularStrength = 0.35f;
        float shininess = 32.0f;
        bool shadowsEnabled = true;
        float shadowExtent = 40.0f; // Half-width of the square area the shadow map covers, centered on the focus point

        // shadowMapSize is in texels; 0 disables shadows entirely (no depth framebuffer is made)
        bool init(int shadowMapSize = 2048);
        void unload();

        // Assign this to a model's material: model.materials[0].shader = lighting.getShader()
        const Shader& getShader() const { return mShader; }

        // drawCasters is called inside the light's view; it should draw everything that casts shadows
        template<class DrawFn>
        void renderShadows(Vector3 focus, DrawFn drawCasters)
        {
            if (!beginShadowPass(focus))
            {
                return;
            }
            drawCasters();
            endShadowPass();
        }

        // Uploads lights, the camera position, and the shadow map; call before drawing lit models
        void apply(const Camera3D& camera);

    private:
        bool beginShadowPass(Vector3 focus);
        void endShadowPass();

        Shader mShader{};
        RenderTexture2D mShadowMap{};
        int mShadowMapSize = 0;
        Matrix mLightViewProj{};
        bool mHasShadowMap = false;

        int mLocViewPos = -1;
        int mLocSunDirection = -1;
        int mLocSunColor = -1;
        int mLocAmbient = -1;
        int mLocSpecular = -1;
        int mLocShininess = -1;
        int mLocPointCount = -1;
        int mLocPointPosition = -1;
        int mLocPointColor = -1;
        int mLocPointRange = -1;
        int mLocLightViewProj = -1;
        int mLocShadowMap = -1;
        int mLocShadowsEnabled = -1;
        int mLocShadowTexel = -1;
    };
}
