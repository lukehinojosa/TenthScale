#pragma once
#include <vector>
#include "core.h"
#include "threadPool.h"

namespace cyclone
{
    // Equal cube cells at least as wide as the largest interaction distance, so interacting points always share
    // a cell or are in neighbouring ones. Cells along a row are next to each other in memory.
    class UniformGrid
    {
    public:
        // Sorts count points into cells at least cellSize wide, split across
        // threadPool if given. The result is the same either way.
        void build(const Vector3* points, int count, real cellSize, ThreadPool* threadPool = nullptr);

        // Calls visit once for every pair of points closer than distance, which can't exceed the cell size. Points
        // are their index in the array given to build().
        template <typename Visit> void forEachPairWithin(real distance, Visit visit) const
        {
            forEachPairInCells(0, getCellCount(), distance, visit);
        }

        // The same, searching only cells cellBegin to cellEnd. Each pair is found from exactly one cell, so separate
        // runs of cells, such as one per thread, visit every pair once between them.
        template <typename Visit> void forEachPairInCells(int cellBegin, int cellEnd, real distance, Visit visit) const;

        // How many points are in the cells before cell. A run of cells' points
        // are from getPointsBefore(begin) up to getPointsBefore(end).
        int getPointsBefore(int cell) const { return m_cellStart[cell]; }

        real getCellSize() const { return m_cellSize; }
        int getCellCount() const { return m_dimX * m_dimY * m_dimZ; }

        // Cells are numbered x first, then y, then z, so each layer (one z)
        // is a run of getCellsPerLayer() cells
        int getCellsPerLayer() const { return m_dimX * m_dimY; }
        int getLayerCount() const { return m_dimZ; }

    private:
        int cellOf(const Vector3& point) const;

        real m_cellSize = 1;
        Vector3 m_origin; // Corner of cell (0, 0, 0)
        int m_dimX = 0, m_dimY = 0, m_dimZ = 0; // Cells along each axis

        // Cell c's points are at m_cellStart[c] up to (not including)
        // m_cellStart[c + 1] in the sorted arrays below
        std::vector<int> m_cellStart;

        // Each point's index and position, grouped by cell, so the pair search reads memory in order.
        std::vector<int> m_sortedIndex;
        std::vector<Vector3> m_sortedPoint;

        // Used while building: each point's cell, by original index, and
        // where the next point goes in each cell
        std::vector<int> m_pointCell;
        std::vector<int> m_writeCursor;

        // Used while building to sort by layer first: each chunk's points per layer, where each layer starts, and
        // the points grouped by layer.
        std::vector<int> m_chunkLayerCount;
        std::vector<int> m_layerStart;
        std::vector<int> m_pointsByLayer;
    };

    template <typename Visit> void UniformGrid::forEachPairInCells(int cellBegin, int cellEnd, real distance, Visit visit) const
    {
        const real distanceSquared = distance * distance;

        // Tests point k against every point in [begin, end) of the sorted arrays
        auto testRange = [&](int k, int begin, int end)
        {
            const Vector3 p = m_sortedPoint[k];
            for (int m = begin; m < end; m++)
            {
                if ((m_sortedPoint[m] - p).squareMagnitude() < distanceSquared)
                {
                    visit(m_sortedIndex[k], m_sortedIndex[m]);
                }
            }
        };

        // Each pair of neighbouring cells is searched from one side only: from every cell, the 13 neighbours that
        // come after it, each one run of memory.
        struct Row
        {
            int dy, dz;
        };
        static const Row kForwardRows[] = { { 1, 0 }, { -1, 1 }, { 0, 1 }, { 1, 1 } };

        // The first cell's x, y, z, then stepped along with the cell below
        int x = cellBegin % m_dimX;
        int y = (cellBegin / m_dimX) % m_dimY;
        int z = cellBegin / (m_dimX * m_dimY);

        for (int cell = cellBegin; cell < cellEnd; cell++)
        {
            const int pointsEnd = m_cellStart[cell + 1];
            if (m_cellStart[cell] != pointsEnd)
            {
                // The neighbouring rows' ranges are the same for every
                // point in this cell, so work them out once
                const int x0 = (x > 0) ? x - 1 : x;
                const int x1 = (x + 1 < m_dimX) ? x + 1 : x;
                int rangeBegin[4], rangeEnd[4];
                int rangeCount = 0;
                for (const Row& row : kForwardRows)
                {
                    const int ny = y + row.dy;
                    const int nz = z + row.dz;
                    if (ny < 0 || ny >= m_dimY || nz >= m_dimZ)
                    {
                        continue;
                    }
                    const int neighbourRow = m_dimX * (ny + m_dimY * nz);
                    rangeBegin[rangeCount] = m_cellStart[neighbourRow + x0];
                    rangeEnd[rangeCount] = m_cellStart[neighbourRow + x1 + 1];
                    rangeCount++;
                }
                // The rest of this cell, then the cell at x + 1, are one run
                const int rowEnd = m_cellStart[cell - x + x1 + 1];

                for (int k = m_cellStart[cell]; k < pointsEnd; k++)
                {
                    // Only points after k in its own cell, so each pair
                    // is visited once and never a point with itself
                    testRange(k, k + 1, rowEnd);
                    for (int r = 0; r < rangeCount; r++)
                    {
                        testRange(k, rangeBegin[r], rangeEnd[r]);
                    }
                }
            }

            // Step to the next cell: along x, wrapping onto the next row and layer
            if (++x == m_dimX)
            {
                x = 0;
                if (++y == m_dimY)
                {
                    y = 0;
                    z++;
                }
            }
        }
    }
}
