#pragma once

#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include "GridPath.h"

namespace ai
{
    enum class Direction : std::uint8_t
    {
        NORTH, // -y
        EAST, // +x
        SOUTH, // +y
        WEST // -x
    };

    // A grid of cells that starts with every wall up; generators knock walls down
    class Maze
    {
    public:
        Maze(int width, int height);

        int getWidth() const { return mWidth; }
        int getHeight() const { return mHeight; }
        bool inBounds(GridPoint p) const { return p.x >= 0 && p.y >= 0 && p.x < mWidth && p.y < mHeight; }

        bool hasWall(GridPoint cell, Direction dir) const;

        // Also clears the matching wall of the neighbor on the other side
        void removeWall(GridPoint cell, Direction dir);
        void resetWalls();

        // Every cell as a 2x+1 by 2y+1 GridMap (walls are blocked cells), for findPathAStar and friends
        GridMap toGridMap() const;

        static GridPoint step(GridPoint p, Direction dir);

    private:
        int mWidth;
        int mHeight;
        std::vector<std::uint8_t> mWalls; // One bit per Direction
    };

    // Builds a maze one step at a time so it can be animated; call step() until it returns false
    class MazeGenerator
    {
    public:
        explicit MazeGenerator(std::uint32_t seed) : mRng(seed) {}
        virtual ~MazeGenerator() = default;

        virtual void begin(Maze& maze) = 0;
        virtual bool step(Maze& maze) = 0;

        void generate(Maze& maze)
        {
            begin(maze);
            while (step(maze))
            {
            }
        }

    protected:
        int randomIndex(std::size_t count) { return std::uniform_int_distribution<int>(0, (int)count - 1)(mRng); }

        std::mt19937 mRng;
    };

    // Depth-first walk with a stack: long winding corridors, few dead ends
    class RecursiveBacktracker : public MazeGenerator
    {
    public:
        using MazeGenerator::MazeGenerator;
        void begin(Maze& maze) override;
        bool step(Maze& maze) override;

    private:
        std::vector<GridPoint> mStack;
        std::vector<bool> mVisited;
    };

    // Grows from a random frontier cell: short branches, many dead ends
    class PrimGenerator : public MazeGenerator
    {
    public:
        using MazeGenerator::MazeGenerator;
        void begin(Maze& maze) override;
        bool step(Maze& maze) override;

    private:
        void addFrontier(const Maze& maze, GridPoint cell);

        std::vector<GridPoint> mFrontier;
        std::vector<std::uint8_t> mState; // 0 untouched, 1 frontier, 2 in maze
    };

    // Random walk until stuck, then hunts row by row for an unvisited cell next to the maze
    class HuntAndKill : public MazeGenerator
    {
    public:
        using MazeGenerator::MazeGenerator;
        void begin(Maze& maze) override;
        bool step(Maze& maze) override;

    private:
        GridPoint mCurrent;
        std::vector<bool> mVisited;
        bool mDone = false;
    };
}
