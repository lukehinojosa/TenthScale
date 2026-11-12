#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "Compute.h"
#include "Context.h"

namespace gpu
{
    struct Float4
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 0.0f;
    };

    struct BallSettings
    {
        float gravityY = -9.81f;
        float damping = 0.99f; // Fraction of velocity kept per second; 1 is none
        float restitution = 0.6f;
        int iterations = 4; // Passes over the contacts each step, 1 to kMaxIterations
    };

    struct BallStats
    {
        std::uint32_t ballContacts = 0;
        std::uint32_t wallContacts = 0;
    };

    // The uniform grid both versions sort balls into: cells as wide as the biggest ball, inside a cube centered on 0
    struct BallGrid
    {
        static constexpr int kMaxCellsPerSide = 128;

        static BallGrid make(float maxRadius, float boxHalfSize);

        int dims = 1; // Cells along each axis
        float cellSize = 1.0f;
        float boxHalfSize = 1.0f;
        std::uint32_t cellCount = 1;
        std::uint32_t blockCount = 1; // Blocks of 256 cells in the GPU's prefix sum
    };

    constexpr int kMaxIterations = 8;

    // Balls in a box: integrate, sort into a grid, resolve ball and wall contacts. Each sphere is a center (xyz)
    // and radius (w); each velocity is a velocity (xyz) and inverse mass (w, 0 for balls that never move).
    class CpuBallSimulation
    {
    public:
        void reset(const std::vector<Float4>& spheres, const std::vector<Float4>& velocities, float boxHalfSize);
        void step(float dt, const BallSettings& settings);

        const std::vector<Float4>& getSpheres() const { return mSpheres; }
        const std::vector<Float4>& getVelocities() const { return mVelocities; }
        const BallStats& getStats() const { return mStats; }

    private:
        std::uint32_t resolvePair(Float4& pa, Float4& va, std::uint32_t m, float restitution);
        std::uint32_t cellOf(const Float4& sphere) const;

        BallGrid mGrid;
        std::vector<Float4> mSpheres;
        std::vector<Float4> mVelocities;
        std::vector<std::uint32_t> mCellOf;
        std::vector<std::uint32_t> mCellStart;
        std::vector<std::uint32_t> mSortedIndex;
        std::vector<Float4> mSortedSpheres;
        std::vector<Float4> mSortedVelocities;
        BallStats mStats;
    };

    // The same simulation in compute shaders. Steps run on the GPU; the CPU copy is a readback, a frame or two old.
    class GpuBallSimulation
    {
    public:
        bool init(Context& context);
        bool isReady() const { return mpContext != nullptr; }

        void reset(const std::vector<Float4>& spheres, const std::vector<Float4>& velocities, float boxHalfSize);
        void step(float dt, const BallSettings& settings);

        // Starts copying the current state back; skipped while two copies are still on the way
        void requestReadback();

        // Picks up a finished copy; true when the spheres, velocities, and stats changed
        bool poll();

        std::size_t getBallCount() const { return mBallCount; }
        const std::vector<Float4>& getSpheres() const { return mSpheres; }
        const std::vector<Float4>& getVelocities() const { return mVelocities; }
        const BallStats& getStats() const { return mStats; }

        // Steps run on the GPU that the newest readback doesn't include yet
        std::uint64_t getStepsBehind() const { return mStepsSubmitted - mStepsInReadback; }

    private:
        enum Kernel
        {
            CLEAR,
            INTEGRATE,
            SCAN_BLOCKS,
            SCAN_BLOCK_SUMS,
            ADD_BLOCK_OFFSETS,
            SCATTER,
            RESOLVE,
            WALLS,
            WRITE_BACK,
            KERNEL_COUNT
        };

        void buildBindGroups();
        BindGroup makeBindGroup(Kernel kernel, std::uint32_t uniformSlot);

        Context* mpContext = nullptr;
        ShaderModule mModule;
        ComputeProgram mKernels[KERNEL_COUNT];

        Buffer mParams;
        Buffer mSpheresBuffer;
        Buffer mVelocitiesBuffer;
        Buffer mCellOf;
        Buffer mRankInCell;
        Buffer mCellStart;
        Buffer mBlockSums;
        Buffer mSortedIndex;
        Buffer mSortedSpheres;
        Buffer mSortedVelocities;
        Buffer mStatsBuffer;

        BindGroup mBaseGroups[KERNEL_COUNT];
        std::vector<BindGroup> mResolveGroups; // One per pass and cell colour
        std::vector<BindGroup> mWallGroups; // One per pass

        BallGrid mGrid;
        std::size_t mBallCount = 0;
        std::vector<std::uint8_t> mParamBytes;
        struct PendingReadback
        {
            std::uint64_t serial = 0;
            std::uint64_t steps = 0;
            std::uint64_t generation = 0;
        };

        Readback mReadback;
        std::vector<std::uint8_t> mReadbackBytes;
        std::vector<PendingReadback> mPending;
        std::uint64_t mGeneration = 0; // Bumped by reset(), so copies of the old balls are ignored

        std::vector<Float4> mSpheres;
        std::vector<Float4> mVelocities;
        BallStats mStats;
        std::uint64_t mStepsSubmitted = 0;
        std::uint64_t mStepsInReadback = 0;
    };

    // The WGSL source for GpuBallSimulation
    const std::string& getBallShaderSource();
}
