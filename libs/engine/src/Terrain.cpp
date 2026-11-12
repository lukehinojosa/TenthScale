#include "engine/Terrain.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <FastNoiseLite.h>
#include <raymath.h>

namespace engine
{
    namespace
    {
        // Sand at the bottom, then grass, rock, and snow on the peaks
        Color colorForHeight(float t)
        {
            struct Band
            {
                float start;
                Color color;
            };
            const Band bands[] = {
                { 0.00f, { 194, 178, 128, 255 } },
                { 0.12f, { 90, 140, 60, 255 } },
                { 0.45f, { 70, 110, 50, 255 } },
                { 0.65f, { 120, 110, 100, 255 } },
                { 0.85f, { 240, 240, 245, 255 } },
            };

            for (int i = 4; i > 0; i--)
            {
                if (t >= bands[i - 1].start)
                {
                    float blend = std::clamp((t - bands[i - 1].start) / (bands[i].start - bands[i - 1].start), 0.0f, 1.0f);
                    return ColorLerp(bands[i - 1].color, bands[i].color, blend);
                }
            }
            return bands[0].color;
        }

        void* copyToRaylib(const void* data, std::size_t bytes)
        {
            void* out = MemAlloc((unsigned int)bytes); // UnloadMesh frees with MemFree
            std::memcpy(out, data, bytes);
            return out;
        }
    }

    void Terrain::generate(const TerrainSettings& settings)
    {
        unload();
        mSettings = settings;
        // 256 per side is the most that 16-bit mesh indices can address
        const int n = std::clamp(settings.resolution, 2, 256);
        mSettings.resolution = n;

        FastNoiseLite noise(settings.seed);
        noise.SetNoiseType(FastNoiseLite::NoiseType_OpenSimplex2);
        noise.SetFractalType(FastNoiseLite::FractalType_FBm);
        noise.SetFrequency(settings.frequency);
        noise.SetFractalOctaves(settings.octaves);
        noise.SetFractalLacunarity(settings.lacunarity);
        noise.SetFractalGain(settings.gain);

        const float step = settings.size / (float)(n - 1);
        const float half = settings.size * 0.5f;
        mHeights.assign((std::size_t)n * n, 0.0f);
        for (int z = 0; z < n; z++)
        {
            for (int x = 0; x < n; x++)
            {
                float wx = -half + x * step;
                float wz = -half + z * step;
                float value = noise.GetNoise(wx, wz) * 0.5f + 0.5f; // Noise is -1 to 1
                mHeights[(std::size_t)z * n + x] = value * value * settings.height; // Squaring flattens valleys
            }
        }

        std::vector<float> positions;
        std::vector<float> normals;
        std::vector<float> texcoords;
        std::vector<unsigned char> colors;
        positions.reserve((std::size_t)n * n * 3);
        normals.reserve(positions.capacity());
        texcoords.reserve((std::size_t)n * n * 2);
        colors.reserve((std::size_t)n * n * 4);

        auto heightAt = [&](int x, int z)
        {
            x = std::clamp(x, 0, n - 1);
            z = std::clamp(z, 0, n - 1);
            return mHeights[(std::size_t)z * n + x];
        };

        for (int z = 0; z < n; z++)
        {
            for (int x = 0; x < n; x++)
            {
                float h = heightAt(x, z);
                positions.insert(positions.end(), { -half + x * step, h, -half + z * step });

                // Central differences of the heightfield
                Vector3 normal = Vector3Normalize({ heightAt(x - 1, z) - heightAt(x + 1, z), 2.0f * step, heightAt(x, z - 1) - heightAt(x, z + 1) });
                normals.insert(normals.end(), { normal.x, normal.y, normal.z });
                texcoords.insert(texcoords.end(), { (float)x / (n - 1), (float)z / (n - 1) });

                // Steep slopes show rock whatever their height
                float t = settings.height > 0.0f ? h / settings.height : 0.0f;
                Color color = colorForHeight(t);
                color = ColorLerp(color, Color{ 115, 105, 95, 255 }, std::clamp((0.85f - normal.y) * 4.0f, 0.0f, 1.0f));
                colors.insert(colors.end(), { color.r, color.g, color.b, 255 });
            }
        }

        std::vector<unsigned short> indices;
        indices.reserve((std::size_t)(n - 1) * (n - 1) * 6);
        for (int z = 0; z < n - 1; z++)
        {
            for (int x = 0; x < n - 1; x++)
            {
                unsigned short topLeft = (unsigned short)(z * n + x);
                unsigned short topRight = (unsigned short)(topLeft + 1);
                unsigned short bottomLeft = (unsigned short)((z + 1) * n + x);
                unsigned short bottomRight = (unsigned short)(bottomLeft + 1);
                indices.insert(indices.end(), { topLeft, bottomLeft, topRight, topRight, bottomLeft, bottomRight });
            }
        }

        Mesh mesh{};
        mesh.vertexCount = n * n;
        mesh.triangleCount = (n - 1) * (n - 1) * 2;
        mesh.vertices = (float*)copyToRaylib(positions.data(), positions.size() * sizeof(float));
        mesh.normals = (float*)copyToRaylib(normals.data(), normals.size() * sizeof(float));
        mesh.texcoords = (float*)copyToRaylib(texcoords.data(), texcoords.size() * sizeof(float));
        mesh.colors = (unsigned char*)copyToRaylib(colors.data(), colors.size());
        mesh.indices = (unsigned short*)copyToRaylib(indices.data(), indices.size() * sizeof(unsigned short));
        UploadMesh(&mesh, false);
        mModel = LoadModelFromMesh(mesh);
    }

    void Terrain::unload()
    {
        if (mModel.meshCount > 0)
        {
            UnloadModel(mModel);
            mModel = Model{};
        }
        mHeights.clear();
    }

    float Terrain::getHeight(float x, float z) const
    {
        if (mHeights.empty())
        {
            return 0.0f;
        }

        const int n = mSettings.resolution;
        const float half = mSettings.size * 0.5f;
        float gx = (x + half) / mSettings.size * (n - 1);
        float gz = (z + half) / mSettings.size * (n - 1);
        if (gx < 0.0f || gz < 0.0f || gx > (float)(n - 1) || gz > (float)(n - 1))
        {
            return 0.0f;
        }

        int x0 = std::min((int)gx, n - 2);
        int z0 = std::min((int)gz, n - 2);
        float fx = gx - x0;
        float fz = gz - z0;
        float h00 = mHeights[(std::size_t)z0 * n + x0];
        float h10 = mHeights[(std::size_t)z0 * n + x0 + 1];
        float h01 = mHeights[(std::size_t)(z0 + 1) * n + x0];
        float h11 = mHeights[(std::size_t)(z0 + 1) * n + x0 + 1];
        return Lerp(Lerp(h00, h10, fx), Lerp(h01, h11, fx), fz);
    }
}
