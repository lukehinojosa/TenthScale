#include "flight/Globe.h"

#include <cctype>
#include <cmath>
#include <numbers>
#include <string>

#include <engine/ShaderUtils.h>
#include <rlgl.h>

#include "shaders/globe.frag.h"
#include "shaders/planet.vert.h"

namespace tenth
{
    namespace
    {
        constexpr int WEB_MAP_WIDTH = 4096;
        constexpr int SPHERE_RINGS = 64;
        constexpr int SPHERE_SEGMENTS = 128;
        constexpr int GROUND_RINGS = 36;
        constexpr int GROUND_SEGMENTS = 96;
        constexpr double GROUND_FIRST_RING = 2.0; // metres
        constexpr double GROUND_RING_GROWTH = 1.35;

        Vector2 mapSpot(const Vec3d& p)
        {
            const double latitude = std::asin(p.y / p.length());
            const double longitude = std::atan2(-p.z, p.x);
            return { (float)(0.5 + longitude / (2.0 * std::numbers::pi)), (float)(0.5 - latitude / std::numbers::pi) };
        }
    }

    Mesh makeGlobeMesh(int rings, int segments)
    {
        Mesh mesh{};
        mesh.vertexCount = (rings + 1) * (segments + 1);
        mesh.triangleCount = rings * segments * 2;
        mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
        mesh.normals = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
        mesh.texcoords = (float*)MemAlloc(mesh.vertexCount * 2 * sizeof(float));
        mesh.indices = (unsigned short*)MemAlloc(mesh.triangleCount * 3 * sizeof(unsigned short));
        for (int ring = 0; ring <= rings; ring++)
        {
            const double latitude = std::numbers::pi * (0.5 - (double)ring / rings);
            for (int segment = 0; segment <= segments; segment++)
            {
                const double longitude = std::numbers::pi * (2.0 * segment / segments - 1.0);
                const Vec3d up = SurfaceFrame::direction(latitude, longitude);
                const int vertex = ring * (segments + 1) + segment;
                for (int axis = 0; axis < 3; axis++)
                {
                    const double value = axis == 0 ? up.x : (axis == 1 ? up.y : up.z);
                    mesh.vertices[vertex * 3 + axis] = (float)value;
                    mesh.normals[vertex * 3 + axis] = (float)value;
                }
                mesh.texcoords[vertex * 2 + 0] = (float)segment / segments;
                mesh.texcoords[vertex * 2 + 1] = (float)ring / rings;
            }
        }

        // Wound to face outward
        int index = 0;
        for (int ring = 0; ring < rings; ring++)
        {
            for (int segment = 0; segment < segments; segment++)
            {
                const unsigned short a = (unsigned short)(ring * (segments + 1) + segment);
                const unsigned short b = (unsigned short)(a + 1);
                const unsigned short c = (unsigned short)(a + segments + 1);
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

    Mesh makeGroundPatch(const SurfaceFrame& site, double radius)
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

    Texture2D loadSurfaceMap(const char* path, Color fallback, bool grey)
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

    BodyAxes bodyAxes(const SolarSystem& system, BodyId id, double time)
    {
        BodyAxes axes;
        const BodyId parent = system.body(id).parent;
        if (parent == NO_BODY || system.body(parent).parent == NO_BODY)
        {
            return axes;
        }
        const Vec3d toParent = (system.positionAt(parent, time) - system.positionAt(id, time)).normalized();
        const Vec3d north = Vec3d{ 0.0, 1.0, 0.0 } - toParent * toParent.y;
        if (north.lengthSquared() <= 1e-12)
        {
            return axes;
        }
        axes.x = toParent;
        axes.y = north.normalized();
        axes.z = axes.x.cross(axes.y);
        return axes;
    }

    Matrix placedMatrix(const Vec3d& x, const Vec3d& y, const Vec3d& z, Vector3 at)
    {
        return { (float)x.x, (float)y.x, (float)z.x, at.x, (float)x.y, (float)y.y, (float)z.y, at.y,
                 (float)x.z, (float)y.z, (float)z.z, at.z, 0.0f, 0.0f, 0.0f, 1.0f };
    }

    void BodyGlobes::init(const SolarSystem& system)
    {
        mSphere = makeGlobeMesh(SPHERE_RINGS, SPHERE_SEGMENTS);
        mShader = engine::loadPortableShader(shaders::PLANET_VERT, shaders::GLOBE_FRAG);
        mShader.locs[SHADER_LOC_MATRIX_MVP] = GetShaderLocation(mShader, "mvp");
        mShader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(mShader, "matModel");
        mMaterial = LoadMaterialDefault();
        mMaterial.shader = mShader;

        mMaps.assign((size_t)system.bodyCount(), Map{});
        for (BodyId id = 0; id < system.bodyCount(); id++)
        {
            Map& map = mMaps[(size_t)id];
            const std::string& name = system.body(id).name;
            map.star = system.body(id).parent == NO_BODY;
            if (name == "Earth")
            {
                continue;
            }
            std::string file = name;
            for (char& c : file)
            {
                c = (char)std::tolower((unsigned char)c);
            }
            map.texture = loadSurfaceMap(("assets/textures/" + file + ".jpg").c_str(), GRAY, false);
            map.loaded = true;
        }
        mGroundBody = NO_BODY;
        mReady = true;
    }

    void BodyGlobes::unload()
    {
        if (!mReady)
        {
            return;
        }
        // The material only borrows the maps, so it's handed back the default before it lets go of its own
        mMaterial.maps[MATERIAL_MAP_ALBEDO].texture = Texture2D{ rlGetTextureIdDefault(), 1, 1, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
        UnloadMaterial(mMaterial); // the shader goes with it
        UnloadMesh(mSphere);
        if (mGroundBody != NO_BODY)
        {
            UnloadMesh(mGround);
            mGroundBody = NO_BODY;
        }
        for (Map& map : mMaps)
        {
            if (map.loaded)
            {
                UnloadTexture(map.texture);
            }
        }
        mMaps.clear();
        mReady = false;
    }

    bool BodyGlobes::usesPlanet(BodyId id) const
    {
        return id >= 0 && id < (BodyId)mMaps.size() && !mMaps[(size_t)id].loaded;
    }

    void BodyGlobes::useMap(BodyId id, Vector3 sun) const
    {
        const float glow = mMaps[(size_t)id].star ? 1.0f : 0.0f;
        SetShaderValue(mShader, GetShaderLocation(mShader, "sunDirection"), &sun, SHADER_UNIFORM_VEC3);
        SetShaderValue(mShader, GetShaderLocation(mShader, "glow"), &glow, SHADER_UNIFORM_FLOAT);
        mMaterial.maps[MATERIAL_MAP_ALBEDO].texture = mMaps[(size_t)id].texture; // the one material is shared by every body
    }

    void BodyGlobes::drawGlobe(BodyId id, const Matrix& placed, Vector3 sun) const
    {
        if (!mReady || usesPlanet(id))
        {
            return;
        }
        useMap(id, sun);
        DrawMesh(mSphere, mMaterial, placed);
    }

    void BodyGlobes::placeGround(BodyId id, const Vec3d& position, double radius, double slack)
    {
        if (!mReady || usesPlanet(id))
        {
            return;
        }
        if (id == mGroundBody && (position.normalized() * radius - mGroundSite.origin).length() < slack)
        {
            return;
        }
        if (mGroundBody != NO_BODY)
        {
            UnloadMesh(mGround);
        }
        mGroundSite = SurfaceFrame::below(position, radius);
        mGround = makeGroundPatch(mGroundSite, radius);
        mGroundBody = id;
    }

    void BodyGlobes::drawGround(const Matrix& placed, Vector3 sun) const
    {
        if (!mReady || mGroundBody == NO_BODY)
        {
            return;
        }
        useMap(mGroundBody, sun);
        DrawMesh(mGround, mMaterial, placed);
    }
}
