#include "gpu/BallSimulation.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gpu
{
    namespace
    {
        constexpr std::uint32_t SCAN_BLOCK = 256;
        constexpr std::uint64_t UNIFORM_SLOT = 256; // WebGPU's minimum uniform buffer offset alignment
        constexpr int COLORS = 27;

        // Matches Params in the shader, byte for byte
        struct Params
        {
            std::uint32_t ballCount;
            std::uint32_t cellCount;
            std::uint32_t blockCount;
            std::uint32_t countContacts;
            std::int32_t dims[3];
            float cellSize;
            float origin[3];
            float dt;
            float gravity[3];
            float damping;
            std::int32_t colorOffset[3];
            float restitution;
            float boxHalfSize;
            float pad[3];
        };
        static_assert(sizeof(Params) == 96, "Params must match the WGSL struct");

        float dot3(const Float4& a, const Float4& b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        int clampCell(float value, int dims)
        {
            // Saturates like WGSL's float to int conversion, NaN included
            if (!(value > 0.0f))
            {
                return 0;
            }
            if (value >= (float)dims)
            {
                return dims - 1;
            }
            return std::min((int)value, dims - 1);
        }

        int clampIterations(int iterations)
        {
            return std::clamp(iterations, 1, kMaxIterations);
        }
    }

    BallGrid BallGrid::make(float maxRadius, float boxHalfSize)
    {
        BallGrid grid;
        grid.boxHalfSize = std::max(boxHalfSize, 0.001f);
        grid.cellSize = maxRadius > 0.0f ? maxRadius * 2.0f : 1.0f;
        while (true)
        {
            double perSide = std::floor(grid.boxHalfSize * 2.0 / grid.cellSize) + 1.0;
            if (perSide <= kMaxCellsPerSide)
            {
                grid.dims = (int)perSide;
                break;
            }
            grid.cellSize *= 1.25f;
        }
        grid.cellCount = (std::uint32_t)(grid.dims * grid.dims * grid.dims);
        grid.blockCount = (grid.cellCount + 1 + SCAN_BLOCK - 1) / SCAN_BLOCK;
        return grid;
    }

    void CpuBallSimulation::reset(const std::vector<Float4>& spheres, const std::vector<Float4>& velocities, float boxHalfSize)
    {
        mSpheres = spheres;
        mVelocities = velocities;
        mVelocities.resize(mSpheres.size());
        float maxRadius = 0.0f;
        for (const Float4& sphere : mSpheres)
        {
            maxRadius = std::max(maxRadius, sphere.w);
        }
        mGrid = BallGrid::make(maxRadius, boxHalfSize);
        mStats = BallStats();
    }

    std::uint32_t CpuBallSimulation::cellOf(const Float4& sphere) const
    {
        const float origin = -mGrid.boxHalfSize;
        int x = clampCell((sphere.x - origin) / mGrid.cellSize, mGrid.dims);
        int y = clampCell((sphere.y - origin) / mGrid.cellSize, mGrid.dims);
        int z = clampCell((sphere.z - origin) / mGrid.cellSize, mGrid.dims);
        return (std::uint32_t)(x + mGrid.dims * (y + mGrid.dims * z));
    }

    std::uint32_t CpuBallSimulation::resolvePair(Float4& pa, Float4& va, std::uint32_t m, float restitution)
    {
        Float4& pb = mSortedSpheres[m];
        Float4 d{ pa.x - pb.x, pa.y - pb.y, pa.z - pb.z, 0.0f };
        float radiusSum = pa.w + pb.w;
        float distanceSquared = dot3(d, d);
        if (distanceSquared >= radiusSum * radiusSum || distanceSquared <= 0.0f)
        {
            return 0;
        }
        float dist = std::sqrt(distanceSquared);
        Float4 normal{ d.x / dist, d.y / dist, d.z / dist, 0.0f };
        Float4& vb = mSortedVelocities[m];
        float totalInverseMass = va.w + vb.w;
        if (totalInverseMass <= 0.0f)
        {
            return 1;
        }

        Float4 relative{ va.x - vb.x, va.y - vb.y, va.z - vb.z, 0.0f };
        float separatingVelocity = dot3(relative, normal);
        if (separatingVelocity < 0.0f)
        {
            float impulse = -(1.0f + restitution) * separatingVelocity / totalInverseMass;
            va.x += normal.x * impulse * va.w;
            va.y += normal.y * impulse * va.w;
            va.z += normal.z * impulse * va.w;
            vb.x -= normal.x * impulse * vb.w;
            vb.y -= normal.y * impulse * vb.w;
            vb.z -= normal.z * impulse * vb.w;
        }

        float push = (radiusSum - dist) / totalInverseMass;
        pa.x += normal.x * push * va.w;
        pa.y += normal.y * push * va.w;
        pa.z += normal.z * push * va.w;
        pb.x -= normal.x * push * vb.w;
        pb.y -= normal.y * push * vb.w;
        pb.z -= normal.z * push * vb.w;
        return 1;
    }

    void CpuBallSimulation::step(float dt, const BallSettings& settings)
    {
        const std::size_t count = mSpheres.size();
        const int dims = mGrid.dims;
        mStats = BallStats();
        if (count == 0)
        {
            return;
        }

        // Integrate, and count each ball into its cell
        mCellOf.resize(count);
        mCellStart.assign(mGrid.cellCount + 1, 0);
        std::vector<std::uint32_t> rank(count);
        const float dampingFactor = settings.damping != 1.0f ? std::pow(settings.damping, dt) : 1.0f;
        for (std::size_t i = 0; i < count; i++)
        {
            Float4& v = mVelocities[i];
            Float4& p = mSpheres[i];
            v.y += settings.gravityY * dt;
            v.x *= dampingFactor;
            v.y *= dampingFactor;
            v.z *= dampingFactor;
            p.x += v.x * dt;
            p.y += v.y * dt;
            p.z += v.z * dt;
            mCellOf[i] = cellOf(p);
            rank[i] = mCellStart[mCellOf[i]]++;
        }

        // Counts to starts, then scatter into cell order
        std::uint32_t running = 0;
        for (std::uint32_t& start : mCellStart)
        {
            std::uint32_t cellCount = start;
            start = running;
            running += cellCount;
        }
        mSortedIndex.resize(count);
        mSortedSpheres.resize(count);
        mSortedVelocities.resize(count);
        for (std::size_t i = 0; i < count; i++)
        {
            std::uint32_t slot = mCellStart[mCellOf[i]] + rank[i];
            mSortedIndex[slot] = (std::uint32_t)i;
            mSortedSpheres[slot] = mSpheres[i];
            mSortedVelocities[slot] = mVelocities[i];
        }

        const int iterations = clampIterations(settings.iterations);
        for (int pass = 0; pass < iterations; pass++)
        {
            std::uint32_t contacts = 0;
            for (int color = 0; color < COLORS; color++)
            {
                const int ox = color % 3;
                const int oy = (color / 3) % 3;
                const int oz = color / 9;
                for (int cz = oz; cz < dims; cz += 3)
                {
                    for (int cy = oy; cy < dims; cy += 3)
                    {
                        for (int cx = ox; cx < dims; cx += 3)
                        {
                            const int cell = cx + dims * (cy + dims * cz);
                            const int x0 = std::max(cx - 1, 0);
                            const int x1 = std::min(cx + 1, dims - 1);
                            for (std::uint32_t k = mCellStart[cell]; k < mCellStart[cell + 1]; k++)
                            {
                                Float4 pa = mSortedSpheres[k];
                                Float4 va = mSortedVelocities[k];
                                for (int z = std::max(cz - 1, 0); z <= std::min(cz + 1, dims - 1); z++)
                                {
                                    for (int y = std::max(cy - 1, 0); y <= std::min(cy + 1, dims - 1); y++)
                                    {
                                        const int row = dims * (y + dims * z);
                                        const std::uint32_t begin = std::max(mCellStart[row + x0], k + 1);
                                        const std::uint32_t end = mCellStart[row + x1 + 1];
                                        for (std::uint32_t m = begin; m < end; m++)
                                        {
                                            contacts += resolvePair(pa, va, m, settings.restitution);
                                        }
                                    }
                                }
                                mSortedSpheres[k] = pa;
                                mSortedVelocities[k] = va;
                            }
                        }
                    }
                }
            }

            std::uint32_t wallContacts = 0;
            const float half = mGrid.boxHalfSize;
            for (std::size_t k = 0; k < count; k++)
            {
                Float4& p = mSortedSpheres[k];
                Float4& v = mSortedVelocities[k];
                for (int wall = 0; wall < 6; wall++)
                {
                    const float sign = wall % 2 == 0 ? 1.0f : -1.0f;
                    float* pPosition = wall / 2 == 0 ? &p.x : (wall / 2 == 1 ? &p.y : &p.z);
                    float* pVelocity = wall / 2 == 0 ? &v.x : (wall / 2 == 1 ? &v.y : &v.z);
                    const float dist = sign * *pPosition + half - p.w;
                    if (dist >= 0.0f)
                    {
                        continue;
                    }
                    wallContacts++;
                    if (v.w <= 0.0f)
                    {
                        continue;
                    }
                    const float separatingVelocity = sign * *pVelocity;
                    if (separatingVelocity < 0.0f)
                    {
                        *pVelocity += sign * (-(1.0f + settings.restitution) * separatingVelocity);
                    }
                    *pPosition -= sign * dist;
                }
            }

            if (pass == 0)
            {
                mStats.ballContacts = contacts;
                mStats.wallContacts = wallContacts;
            }
        }

        for (std::size_t k = 0; k < count; k++)
        {
            const std::uint32_t i = mSortedIndex[k];
            mSpheres[i] = mSortedSpheres[k];
            mVelocities[i] = mSortedVelocities[k];
        }
    }

    bool GpuBallSimulation::init(Context& context)
    {
        if (!context.isReady())
        {
            return false;
        }
        if (!mModule.load(context, getBallShaderSource(), "ball simulation"))
        {
            return false;
        }

        const char* entryPoints[KERNEL_COUNT] = {
            "clearCells", "integrate", "scanBlocks", "scanBlockSums", "addBlockOffsets",
            "scatter", "resolveColor", "walls", "writeBack"
        };
        for (int kernel = 0; kernel < KERNEL_COUNT; kernel++)
        {
            if (!mKernels[kernel].create(context, mModule, entryPoints[kernel]))
            {
                return false;
            }
        }

        const std::uint64_t slots = 1 + (std::uint64_t)kMaxIterations * COLORS;
        if (!mParams.create(context, slots * UNIFORM_SLOT, Buffer::kUniform, "ball params") ||
            !mStatsBuffer.create(context, sizeof(BallStats), Buffer::kStorage, "ball stats"))
        {
            return false;
        }
        mParamBytes.assign((std::size_t)(slots * UNIFORM_SLOT), 0);
        mpContext = &context;
        reset({}, {}, 1.0f);
        return true;
    }

    void GpuBallSimulation::reset(const std::vector<Float4>& spheres, const std::vector<Float4>& velocities, float boxHalfSize)
    {
        if (mpContext == nullptr)
        {
            return;
        }
        Context& context = *mpContext;
        mBallCount = spheres.size();
        mSpheres = spheres;
        mVelocities = velocities;
        mVelocities.resize(mBallCount);
        mStats = BallStats();
        mGeneration++;
        mPending.clear();
        mStepsSubmitted = 0;
        mStepsInReadback = 0;

        float maxRadius = 0.0f;
        for (const Float4& sphere : spheres)
        {
            maxRadius = std::max(maxRadius, sphere.w);
        }
        mGrid = BallGrid::make(maxRadius, boxHalfSize);

        // Never zero-sized: WebGPU can't bind an empty buffer
        const std::uint64_t balls = std::max<std::uint64_t>(mBallCount, 1);
        mSpheresBuffer.create(context, balls * sizeof(Float4), Buffer::kStorage, "spheres");
        mVelocitiesBuffer.create(context, balls * sizeof(Float4), Buffer::kStorage, "velocities");
        mCellOf.create(context, balls * sizeof(std::uint32_t), Buffer::kStorage, "cellOf");
        mRankInCell.create(context, balls * sizeof(std::uint32_t), Buffer::kStorage, "rankInCell");
        mSortedIndex.create(context, balls * sizeof(std::uint32_t), Buffer::kStorage, "sortedIndex");
        mSortedSpheres.create(context, balls * sizeof(Float4), Buffer::kStorage, "sortedSpheres");
        mSortedVelocities.create(context, balls * sizeof(Float4), Buffer::kStorage, "sortedVelocities");
        mCellStart.create(context, (mGrid.cellCount + 1) * sizeof(std::uint32_t), Buffer::kStorage, "cellStart");
        mBlockSums.create(context, mGrid.blockCount * sizeof(std::uint32_t), Buffer::kStorage, "blockSums");

        mSpheresBuffer.write(context, 0, mSpheres.data(), mBallCount * sizeof(Float4));
        mVelocitiesBuffer.write(context, 0, mVelocities.data(), mBallCount * sizeof(Float4));
        buildBindGroups();
    }

    BindGroup GpuBallSimulation::makeBindGroup(Kernel kernel, std::uint32_t uniformSlot)
    {
        const BufferBinding params{ 0, &mParams, uniformSlot * UNIFORM_SLOT, sizeof(Params) };
        const BufferBinding spheres{ 1, &mSpheresBuffer };
        const BufferBinding velocities{ 2, &mVelocitiesBuffer };
        const BufferBinding cellOf{ 3, &mCellOf };
        const BufferBinding rankInCell{ 4, &mRankInCell };
        const BufferBinding cellStart{ 5, &mCellStart };
        const BufferBinding blockSums{ 6, &mBlockSums };
        const BufferBinding sortedIndex{ 7, &mSortedIndex };
        const BufferBinding sortedSpheres{ 8, &mSortedSpheres };
        const BufferBinding sortedVelocities{ 9, &mSortedVelocities };
        const BufferBinding stats{ 10, &mStatsBuffer };

        // Exactly the bindings each entry point uses, since its layout comes from the shader
        const ComputeProgram& program = mKernels[kernel];
        switch (kernel)
        {
        case CLEAR:
            return program.makeBindGroup(*mpContext, { params, cellStart, stats });
        case INTEGRATE:
            return program.makeBindGroup(*mpContext, { params, spheres, velocities, cellOf, rankInCell, cellStart });
        case SCAN_BLOCKS:
        case ADD_BLOCK_OFFSETS:
            return program.makeBindGroup(*mpContext, { params, cellStart, blockSums });
        case SCAN_BLOCK_SUMS:
            return program.makeBindGroup(*mpContext, { params, blockSums });
        case SCATTER:
            return program.makeBindGroup(*mpContext,
                { params, spheres, velocities, cellOf, rankInCell, cellStart, sortedIndex, sortedSpheres, sortedVelocities });
        case RESOLVE:
            return program.makeBindGroup(*mpContext, { params, cellStart, sortedSpheres, sortedVelocities, stats });
        case WALLS:
            return program.makeBindGroup(*mpContext, { params, sortedSpheres, sortedVelocities, stats });
        case WRITE_BACK:
            return program.makeBindGroup(*mpContext, { params, spheres, velocities, sortedIndex, sortedSpheres, sortedVelocities });
        default:
            return BindGroup();
        }
    }

    void GpuBallSimulation::buildBindGroups()
    {
        for (int kernel = 0; kernel < KERNEL_COUNT; kernel++)
        {
            mBaseGroups[kernel] = makeBindGroup((Kernel)kernel, 0);
        }
        mResolveGroups.clear();
        mWallGroups.clear();
        for (int pass = 0; pass < kMaxIterations; pass++)
        {
            for (int color = 0; color < COLORS; color++)
            {
                mResolveGroups.push_back(makeBindGroup(RESOLVE, (std::uint32_t)(1 + pass * COLORS + color)));
            }
            mWallGroups.push_back(makeBindGroup(WALLS, (std::uint32_t)(1 + pass * COLORS)));
        }
    }

    void GpuBallSimulation::step(float dt, const BallSettings& settings)
    {
        if (mpContext == nullptr || !mpContext->isReady() || mBallCount == 0)
        {
            return;
        }
        Context& context = *mpContext;
        const int iterations = clampIterations(settings.iterations);

        Params base{};
        base.ballCount = (std::uint32_t)mBallCount;
        base.cellCount = mGrid.cellCount;
        base.blockCount = mGrid.blockCount;
        base.countContacts = 1;
        base.dims[0] = base.dims[1] = base.dims[2] = mGrid.dims;
        base.cellSize = mGrid.cellSize;
        base.origin[0] = base.origin[1] = base.origin[2] = -mGrid.boxHalfSize;
        base.dt = dt;
        base.gravity[1] = settings.gravityY;
        base.damping = settings.damping;
        base.restitution = settings.restitution;
        base.boxHalfSize = mGrid.boxHalfSize;

        // Slot 0 for the single kernels, then one per pass and colour
        std::memcpy(mParamBytes.data(), &base, sizeof(Params));
        for (int pass = 0; pass < iterations; pass++)
        {
            for (int color = 0; color < COLORS; color++)
            {
                Params slot = base;
                slot.countContacts = pass == 0 ? 1u : 0u;
                slot.colorOffset[0] = color % 3;
                slot.colorOffset[1] = (color / 3) % 3;
                slot.colorOffset[2] = color / 9;
                std::memcpy(mParamBytes.data() + (1 + pass * COLORS + color) * UNIFORM_SLOT, &slot, sizeof(Params));
            }
        }
        mParams.write(context, 0, mParamBytes.data(), (1 + (std::uint64_t)iterations * COLORS) * UNIFORM_SLOT);

        const std::uint64_t cells = mGrid.cellCount + 1;
        const std::uint64_t scanThreads = (std::uint64_t)mGrid.blockCount * SCAN_BLOCK;
        CommandList commands(context);
        commands.dispatch(mKernels[CLEAR], mBaseGroups[CLEAR], cells);
        commands.dispatch(mKernels[INTEGRATE], mBaseGroups[INTEGRATE], mBallCount);
        commands.dispatch(mKernels[SCAN_BLOCKS], mBaseGroups[SCAN_BLOCKS], scanThreads);
        commands.dispatch(mKernels[SCAN_BLOCK_SUMS], mBaseGroups[SCAN_BLOCK_SUMS], SCAN_BLOCK);
        commands.dispatch(mKernels[ADD_BLOCK_OFFSETS], mBaseGroups[ADD_BLOCK_OFFSETS], scanThreads);
        commands.dispatch(mKernels[SCATTER], mBaseGroups[SCATTER], mBallCount);
        for (int pass = 0; pass < iterations; pass++)
        {
            for (int color = 0; color < COLORS; color++)
            {
                const int ox = color % 3;
                const int oy = (color / 3) % 3;
                const int oz = color / 9;
                const std::uint64_t perColor = (std::uint64_t)((mGrid.dims - ox + 2) / 3) * ((mGrid.dims - oy + 2) / 3) * ((mGrid.dims - oz + 2) / 3);
                commands.dispatch(mKernels[RESOLVE], mResolveGroups[pass * COLORS + color], perColor);
            }
            commands.dispatch(mKernels[WALLS], mWallGroups[pass], mBallCount);
        }
        commands.dispatch(mKernels[WRITE_BACK], mBaseGroups[WRITE_BACK], mBallCount);
        commands.submit();
        mStepsSubmitted++;
    }

    void GpuBallSimulation::requestReadback()
    {
        if (mpContext == nullptr || mBallCount == 0)
        {
            return;
        }
        const std::uint64_t bytes = mBallCount * sizeof(Float4);
        std::uint64_t serial = mReadback.request(*mpContext, {
            ReadRange{ &mSpheresBuffer, 0, bytes },
            ReadRange{ &mVelocitiesBuffer, 0, bytes },
            ReadRange{ &mStatsBuffer, 0, sizeof(BallStats) }
        });
        if (serial != 0)
        {
            mPending.push_back(PendingReadback{ serial, mStepsSubmitted, mGeneration });
        }
    }

    bool GpuBallSimulation::poll()
    {
        std::uint64_t serial = 0;
        if (!mReadback.take(mReadbackBytes, &serial))
        {
            return false;
        }

        auto match = std::find_if(mPending.begin(), mPending.end(), [&](const PendingReadback& pending)
        {
            return pending.serial == serial;
        });
        if (match == mPending.end())
        {
            return false;
        }
        const PendingReadback pending = *match;
        mPending.erase(mPending.begin(), match + 1);

        const std::size_t bytes = mBallCount * sizeof(Float4);
        if (pending.generation != mGeneration || mReadbackBytes.size() < bytes * 2 + sizeof(BallStats))
        {
            return false;
        }
        std::memcpy(mSpheres.data(), mReadbackBytes.data(), bytes);
        std::memcpy(mVelocities.data(), mReadbackBytes.data() + bytes, bytes);
        std::memcpy(&mStats, mReadbackBytes.data() + bytes * 2, sizeof(BallStats));
        mStepsInReadback = pending.steps;
        return true;
    }
}
