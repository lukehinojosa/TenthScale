#include "uniformGrid.h"
#include <algorithm>
#include <cmath>

using namespace cyclone;

// Most cells per point. Points spread far apart would need a huge, mostly empty grid, so past this the
// cells get bigger instead.
static const int kMaxCellsPerPoint = 4;
static const int kMinMaxCells = 4096;

// How build() splits its work across threads: a few jobs per thread, so one
// that finishes early can take another, but none smaller than this many points
static const int kJobsPerThread = 4;
static const int kMinPointsPerJob = 1024;

// Most per-chunk layer counts build() keeps. Each chunk counts its points in
// every layer, so a grid with a huge number of layers gets fewer chunks.
static const int kMaxChunkLayerCounts = 1 << 20;

int UniformGrid::cellOf(const Vector3& point) const
{
    // Clamped so rounding at the far edge can't step outside the grid; close pairs still land in neighbouring
    // cells.
    const int x = std::min(std::max((int)((point.x - m_origin.x) / m_cellSize), 0), m_dimX - 1);
    const int y = std::min(std::max((int)((point.y - m_origin.y) / m_cellSize), 0), m_dimY - 1);
    const int z = std::min(std::max((int)((point.z - m_origin.z) / m_cellSize), 0), m_dimZ - 1);
    return x + m_dimX * (y + m_dimY * z);
}

void UniformGrid::build(const Vector3* points, int count, real cellSize, ThreadPool* threadPool)
{
    m_cellSize = (cellSize > 0) ? cellSize : (real)1;

    // Runs job(0) to job(jobCount - 1), on the threads if there are any
    auto runJobs = [&](int jobCount, const std::function<void(int)>& job)
    {
        if (threadPool)
        {
            threadPool->run(jobCount, job);
        }
        else
        {
            for (int j = 0; j < jobCount; j++)
            {
                job(j);
            }
        }
    };

    // The points are split into chunks of neighbouring indices, one job each
    int chunkCount = 1;
    if (threadPool)
    {
        chunkCount = std::max(1, std::min(threadPool->getThreadCount() * kJobsPerThread, count / kMinPointsPerJob));
    }
    auto chunkRange = [&](int chunk, int& begin, int& end)
    {
        begin = (int)((long long)count * chunk / chunkCount);
        end = (int)((long long)count * (chunk + 1) / chunkCount);
    };

    // Fit the grid around the points, so it works wherever they are. Each
    // chunk finds its own bounds, then they're combined.
    std::vector<Vector3> chunkMin(chunkCount);
    std::vector<Vector3> chunkMax(chunkCount);
    if (count > 0)
    {
        runJobs(chunkCount, [&](int chunk)
        {
            int begin, end;
            chunkRange(chunk, begin, end);
            Vector3 min = points[begin];
            Vector3 max = points[begin];
            for (int i = begin + 1; i < end; i++)
            {
                min.x = std::min(min.x, points[i].x);
                min.y = std::min(min.y, points[i].y);
                min.z = std::min(min.z, points[i].z);
                max.x = std::max(max.x, points[i].x);
                max.y = std::max(max.y, points[i].y);
                max.z = std::max(max.z, points[i].z);
            }
            chunkMin[chunk] = min;
            chunkMax[chunk] = max;
        });
    }
    Vector3 min(0, 0, 0);
    Vector3 max(0, 0, 0);
    if (count > 0)
    {
        min = chunkMin[0];
        max = chunkMax[0];
    }
    for (int chunk = 1; chunk < chunkCount && count > 0; chunk++)
    {
        min.x = std::min(min.x, chunkMin[chunk].x);
        min.y = std::min(min.y, chunkMin[chunk].y);
        min.z = std::min(min.z, chunkMin[chunk].z);
        max.x = std::max(max.x, chunkMax[chunk].x);
        max.y = std::max(max.y, chunkMax[chunk].y);
        max.z = std::max(max.z, chunkMax[chunk].z);
    }
    m_origin = min;
    const Vector3 extent = max - min;

    // Grow the cells until the grid has few enough of them. Cells bigger than
    // the interaction distance are still correct, just more points to check.
    const double maxCells = std::max(count * kMaxCellsPerPoint, kMinMaxCells);
    if (!std::isfinite(extent.x + extent.y + extent.z))
    {
        // A point blew up to infinity or NaN, so no cell size can fit them
        // all. One cell still finds every pair, just slowly.
        m_dimX = m_dimY = m_dimZ = 1;
    }
    else
    {
        for (;;)
        {
            const double cellsX = std::floor(extent.x / m_cellSize) + 1;
            const double cellsY = std::floor(extent.y / m_cellSize) + 1;
            const double cellsZ = std::floor(extent.z / m_cellSize) + 1;
            if (cellsX * cellsY * cellsZ <= maxCells)
            {
                m_dimX = (int)cellsX;
                m_dimY = (int)cellsY;
                m_dimZ = (int)cellsZ;
                break;
            }
            m_cellSize *= (real)1.25;
        }
    }
    const int cellCount = m_dimX * m_dimY * m_dimZ;
    const int cellsPerLayer = getCellsPerLayer();
    const int layerCount = getLayerCount();

    // Points are sorted by cell in two passes that split across threads, first by layer and then by cell. Pass 1
    // works out each chunk's cells and counts its points per layer.
    chunkCount = std::min(chunkCount, std::max(1, kMaxChunkLayerCounts / layerCount));
    m_pointCell.resize(count);
    m_chunkLayerCount.assign((size_t)chunkCount * layerCount, 0);
    runJobs(chunkCount, [&](int chunk)
    {
        int begin, end;
        chunkRange(chunk, begin, end);
        int* layerCounts = &m_chunkLayerCount[(size_t)chunk * layerCount];
        for (int i = begin; i < end; i++)
        {
            const int cell = cellOf(points[i]);
            m_pointCell[i] = cell;
            layerCounts[cell / cellsPerLayer]++;
        }
    });

    // Running totals of those counts, layer by layer then chunk by chunk,
    // give where each layer's points start, and where in that each chunk's go
    m_layerStart.resize(layerCount + 1);
    int running = 0;
    for (int layer = 0; layer < layerCount; layer++)
    {
        m_layerStart[layer] = running;
        for (int chunk = 0; chunk < chunkCount; chunk++)
        {
            int& chunkLayer = m_chunkLayerCount[(size_t)chunk * layerCount + layer];
            const int pointsHere = chunkLayer;
            chunkLayer = running; // Now where this chunk's next point in this layer goes
            running += pointsHere;
        }
    }
    m_layerStart[layerCount] = running;

    // Each chunk drops its points into their layer's slots, in index order
    m_pointsByLayer.resize(count);
    runJobs(chunkCount, [&](int chunk)
    {
        int begin, end;
        chunkRange(chunk, begin, end);
        int* layerCursor = &m_chunkLayerCount[(size_t)chunk * layerCount];
        for (int i = begin; i < end; i++)
        {
            m_pointsByLayer[layerCursor[m_pointCell[i] / cellsPerLayer]++] = i;
        }
    });

    // Pass 2: each layer counts its points per cell and drops them into their slots. Layers don't overlap, so
    // they run at the same time.
    m_cellStart.resize(cellCount + 1);
    m_writeCursor.resize(cellCount);
    m_sortedIndex.resize(count);
    m_sortedPoint.resize(count);
    const int layerJobCount = threadPool ? std::min(layerCount, threadPool->getThreadCount() * kJobsPerThread) : 1;
    runJobs(layerJobCount, [&](int job)
    {
        const int firstLayer = (int)((long long)layerCount * job / layerJobCount);
        const int lastLayer = (int)((long long)layerCount * (job + 1) / layerJobCount);
        for (int layer = firstLayer; layer < lastLayer; layer++)
        {
            const int firstCell = layer * cellsPerLayer;
            int* cellStart = &m_cellStart[firstCell];
            int* writeCursor = &m_writeCursor[firstCell];

            std::fill(cellStart, cellStart + cellsPerLayer, 0);
            for (int k = m_layerStart[layer]; k < m_layerStart[layer + 1]; k++)
            {
                cellStart[m_pointCell[m_pointsByLayer[k]] - firstCell]++;
            }

            // Turn the counts into where each cell's points start
            int cellRunning = m_layerStart[layer];
            for (int c = 0; c < cellsPerLayer; c++)
            {
                const int pointsHere = cellStart[c];
                cellStart[c] = cellRunning;
                writeCursor[c] = cellRunning;
                cellRunning += pointsHere;
            }

            for (int k = m_layerStart[layer]; k < m_layerStart[layer + 1]; k++)
            {
                const int i = m_pointsByLayer[k];
                const int slot = m_writeCursor[m_pointCell[i]]++;
                m_sortedIndex[slot] = i;
                m_sortedPoint[slot] = points[i];
            }
        }
    });
    m_cellStart[cellCount] = count;
}
