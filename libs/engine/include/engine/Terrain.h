#pragma once

#include <vector>

#include <raylib.h>

namespace engine
{
    struct TerrainSettings
    {
        int resolution = 129; // Vertices per side, up to 256
        float size = 120.0f; // World width and depth, centered on the origin
        float height = 14.0f; // Peak height above y = 0
        float frequency = 0.012f;
        int octaves = 5; // Layers of noise, each finer than the last
        float lacunarity = 2.0f; // How much finer each octave is
        float gain = 0.5f; // How much weaker each octave is
        int seed = 1337;
    };

    // Fractal-noise heightmap with height-based vertex colors and smooth normals
    class Terrain
    {
    public:
        void generate(const TerrainSettings& settings);
        void unload();

        // Bilinear height at a world position; 0 outside the terrain
        float getHeight(float x, float z) const;

        Model& getModel() { return mModel; }
        const TerrainSettings& getSettings() const { return mSettings; }
        bool isGenerated() const { return mModel.meshCount > 0; }

    private:
        TerrainSettings mSettings;
        std::vector<float> mHeights;
        Model mModel{};
    };
}
