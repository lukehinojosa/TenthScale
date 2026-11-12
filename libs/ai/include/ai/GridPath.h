#pragma once

#include <vector>

namespace ai
{
    struct GridPoint
    {
        int x = 0;
        int y = 0;

        bool operator==(const GridPoint& o) const { return x == o.x && y == o.y; }
        bool operator!=(const GridPoint& o) const { return !(*this == o); }
    };

    // A walkable/blocked grid for pathfinding; out-of-bounds counts as blocked
    class GridMap
    {
    public:
        GridMap(int width, int height) : mWidth(width), mHeight(height), mBlocked((std::size_t)width * height, false) {}

        int getWidth() const { return mWidth; }
        int getHeight() const { return mHeight; }

        bool inBounds(GridPoint p) const { return p.x >= 0 && p.y >= 0 && p.x < mWidth && p.y < mHeight; }
        bool isWalkable(GridPoint p) const { return inBounds(p) && !mBlocked[index(p)]; }
        void setBlocked(GridPoint p, bool blocked)
        {
            if (inBounds(p))
            {
                mBlocked[index(p)] = blocked;
            }
        }

        // 4 neighbors, or 8 with diagonals (no corner cutting past blocked cells)
        std::vector<GridPoint> getNeighbors(GridPoint p, bool diagonals) const;

        int index(GridPoint p) const { return p.y * mWidth + p.x; }

    private:
        int mWidth;
        int mHeight;
        std::vector<bool> mBlocked;
    };

    // Each returns start..goal inclusive, or an empty path when goal is unreachable

    // Fewest steps; fine for uniform-cost grids
    std::vector<GridPoint> findPathBFS(const GridMap& map, GridPoint start, GridPoint goal, bool diagonals = false);

    // Shortest path where diagonal steps cost their true length, guided by a heuristic that never overestimates
    std::vector<GridPoint> findPathAStar(const GridMap& map, GridPoint start, GridPoint goal, bool diagonals = false);

    // Step count from every cell to goal (-1 where unreachable); many agents can share one field
    std::vector<int> buildDistanceField(const GridMap& map, GridPoint goal, bool diagonals = false);
}
