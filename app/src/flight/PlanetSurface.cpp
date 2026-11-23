#include "flight/PlanetSurface.h"

#include <cmath>
#include <string>
#include <numbers>

#include <engine/ShaderUtils.h>
#include <raymath.h>
#include <rlgl.h>

#include "shaders/atmosphere.frag.h"
#include "shaders/planet.frag.h"
#include "shaders/planet.vert.h"

namespace tenth
{
    namespace
    {
        constexpr int SPHERE_RINGS = 128;
        constexpr int SPHERE_SEGMENTS = 256;
        constexpr int GROUND_RINGS = 36;
        constexpr int GROUND_SEGMENTS = 96;
        constexpr double GROUND_FIRST_RING = 2.0; // metres
        constexpr double GROUND_RING_GROWTH = 1.35;
        constexpr int WEB_MAP_WIDTH = 4096; // browsers get smaller maps to stay within their memory
        constexpr Color FLAT = { 128, 128, 255, 255 }; // a relief map's colour for level ground

        Vector2 mapSpot(const Vec3d& p)
        {
            const double latitude = std::asin(p.y / p.length());
            const double longitude = std::atan2(-p.z, p.x);
            return { (float)(0.5 + longitude / (2.0 * std::numbers::pi)), (float)(0.5 - latitude / std::numbers::pi) };
        }

        // A unit sphere in the body's axes, with a doubled seam so the maps wrap without a stitch
        Mesh makeSphere()
        {
            Mesh mesh{};
            mesh.vertexCount = (SPHERE_RINGS + 1) * (SPHERE_SEGMENTS + 1);
            mesh.triangleCount = SPHERE_RINGS * SPHERE_SEGMENTS * 2;
            mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
            mesh.normals = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
            mesh.texcoords = (float*)MemAlloc(mesh.vertexCount * 2 * sizeof(float));
            mesh.indices = (unsigned short*)MemAlloc(mesh.triangleCount * 3 * sizeof(unsigned short));
            for (int ring = 0; ring <= SPHERE_RINGS; ring++)
            {
                const double latitude = std::numbers::pi * (0.5 - (double)ring / SPHERE_RINGS);
                for (int segment = 0; segment <= SPHERE_SEGMENTS; segment++)
                {
                    const double longitude = std::numbers::pi * (2.0 * segment / SPHERE_SEGMENTS - 1.0);
                    const Vec3d up = SurfaceFrame::direction(latitude, longitude);
                    const int vertex = ring * (SPHERE_SEGMENTS + 1) + segment;
                    for (int axis = 0; axis < 3; axis++)
                    {
                        const double value = axis == 0 ? up.x : (axis == 1 ? up.y : up.z);
                        mesh.vertices[vertex * 3 + axis] = (float)value;
                        mesh.normals[vertex * 3 + axis] = (float)value;
                    }
                    mesh.texcoords[vertex * 2 + 0] = (float)segment / SPHERE_SEGMENTS;
                    mesh.texcoords[vertex * 2 + 1] = (float)ring / SPHERE_RINGS;
                }
            }

            // Wound to face outward
            int index = 0;
            for (int ring = 0; ring < SPHERE_RINGS; ring++)
            {
                for (int segment = 0; segment < SPHERE_SEGMENTS; segment++)
                {
                    const unsigned short a = (unsigned short)(ring * (SPHERE_SEGMENTS + 1) + segment);
                    const unsigned short b = (unsigned short)(a + 1);
                    const unsigned short c = (unsigned short)(a + SPHERE_SEGMENTS + 1);
                    const unsigned short d = (unsigned short)(c + 1);
                    mesh.indices[index++] = a;
                    mesh.indices[index++] = c;
                    mesh.indices[index++] = b;
                    mesh.indices[index++] = b;
                    mesh.indices[index++] = c;
                    mesh.indices[index++] = d;
                }
            }
            UploadMesh(&mesh, false);
            return mesh;
        }

        // A patch of the curved surface around the site, in its axes, with rings spaced closer near the middle where
        // the camera is. The whole-planet sphere is too coarse to stand on, so this covers everything in sight there.
        Mesh makeGround(const SurfaceFrame& site, double radius)
        {
            Mesh mesh{};
            mesh.vertexCount = 1 + GROUND_RINGS * GROUND_SEGMENTS;
            mesh.triangleCount = GROUND_SEGMENTS + (GROUND_RINGS - 1) * GROUND_SEGMENTS * 2;
            mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
            mesh.normals = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
            mesh.texcoords = (float*)MemAlloc(mesh.vertexCount * 2 * sizeof(float));
            mesh.indices = (unsigned short*)MemAlloc(mesh.triangleCount * 3 * sizeof(unsigned short));

            mesh.normals[1] = 1.0f;
            const Vector2 middle = mapSpot(site.origin);
            mesh.texcoords[0] = middle.x;
            mesh.texcoords[1] = middle.y;
            double distance = GROUND_FIRST_RING;
            for (int ring = 0; ring < GROUND_RINGS; ring++)
            {
                const double angle = distance / radius;
                const double across = radius * std::sin(angle);
                const double drop = radius * (std::cos(angle) - 1.0);
                for (int segment = 0; segment < GROUND_SEGMENTS; segment++)
                {
                    const double heading = 2.0 * std::numbers::pi * segment / GROUND_SEGMENTS;
                    const int vertex = 1 + ring * GROUND_SEGMENTS + segment;
                    const Vec3d local = { across * std::cos(heading), drop, across * std::sin(heading) };
                    mesh.vertices[vertex * 3 + 0] = (float)local.x;
                    mesh.vertices[vertex * 3 + 1] = (float)local.y;
                    mesh.vertices[vertex * 3 + 2] = (float)local.z;
                    mesh.normals[vertex * 3 + 0] = (float)(std::sin(angle) * std::cos(heading));
                    mesh.normals[vertex * 3 + 1] = (float)std::cos(angle);
                    mesh.normals[vertex * 3 + 2] = (float)(std::sin(angle) * std::sin(heading));
                    const Vector2 spot = mapSpot(site.origin + site.toBody(local));
                    mesh.texcoords[vertex * 2 + 0] = spot.x;
                    mesh.texcoords[vertex * 2 + 1] = spot.y;
                }
                distance *= GROUND_RING_GROWTH;
            }

            // Wound so each triangle faces up, the way the camera sees it
            int index = 0;
            for (int segment = 0; segment < GROUND_SEGMENTS; segment++)
            {
                const int next = (segment + 1) % GROUND_SEGMENTS;
                mesh.indices[index++] = 0;
                mesh.indices[index++] = (unsigned short)(1 + next);
                mesh.indices[index++] = (unsigned short)(1 + segment);
            }
            for (int ring = 0; ring + 1 < GROUND_RINGS; ring++)
            {
                for (int segment = 0; segment < GROUND_SEGMENTS; segment++)
                {
                    const int next = (segment + 1) % GROUND_SEGMENTS;
                    const unsigned short inner = (unsigned short)(1 + ring * GROUND_SEGMENTS + segment);
                    const unsigned short innerNext = (unsigned short)(1 + ring * GROUND_SEGMENTS + next);
                    const unsigned short outer = (unsigned short)(inner + GROUND_SEGMENTS);
                    const unsigned short outerNext = (unsigned short)(innerNext + GROUND_SEGMENTS);
                    mesh.indices[index++] = inner;
                    mesh.indices[index++] = innerNext;
                    mesh.indices[index++] = outer;
                    mesh.indices[index++] = innerNext;
                    mesh.indices[index++] = outerNext;
                    mesh.indices[index++] = outer;
                }
            }
            UploadMesh(&mesh, false);
            return mesh;
        }

        Texture2D loadMap(const char* path, Color fallback, bool grey)
        {
            Image image = FileExists(path) ? LoadImage(path) : Image{};
            if (image.data == nullptr)
            {
                image = GenImageColor(1, 1, fallback);
            }
#ifdef __EMSCRIPTEN__
            if (image.width > WEB_MAP_WIDTH)
            {
                ImageResize(&image, WEB_MAP_WIDTH, WEB_MAP_WIDTH * image.height / image.width);
            }
#endif
            if (grey)
            {
                ImageFormat(&image, PIXELFORMAT_UNCOMPRESSED_GRAYSCALE);
            }
            Texture2D texture = LoadTextureFromImage(image);
            UnloadImage(image);
            GenTextureMipmaps(&texture);
            SetTextureFilter(texture, TEXTURE_FILTER_TRILINEAR);
            SetTextureFilter(texture, TEXTURE_FILTER_ANISOTROPIC_8X);
            return texture;
        }

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
        mSphere = makeSphere();
        mGround = makeGround(site, radius);
        mDay = loadMap("assets/textures/earth_daymap.jpg", localGround, false);
        mNight = loadMap("assets/textures/earth_nightmap.jpg", BLACK, false);
        mClouds = loadMap("assets/textures/earth_clouds.jpg", BLACK, true);
        mRelief = loadMap("assets/textures/earth_normal_map.jpg", FLAT, false);
        mWater = loadMap("assets/textures/earth_specular_map.png", BLACK, true);

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

        rlDrawRenderBatchActive();
        rlDisableDepthTest();
        DrawMesh(mSphere, mSurface, placed(mRadius));
        rlEnableDepthTest();
        DrawMesh(mGround, mSurface, MatrixTranslate(sitePosition.x, sitePosition.y, sitePosition.z));
    }
}
