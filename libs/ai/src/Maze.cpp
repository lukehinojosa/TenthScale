#include "ai/Maze.h"

namespace ai
{
    namespace
    {
        constexpr Direction ALL_DIRECTIONS[] = { Direction::NORTH, Direction::EAST, Direction::SOUTH, Direction::WEST };

        Direction opposite(Direction dir)
        {
            return static_cast<Direction>(((int)dir + 2) % 4);
        }

        std::uint8_t bit(Direction dir)
        {
            return (std::uint8_t)(1u << (int)dir);
        }

        // Directions from cell to in-bounds neighbors whose visited flag equals wantVisited
        std::vector<Direction> neighborsWhere(const Maze& maze, const std::vector<bool>& visited, GridPoint cell, bool wantVisited)
        {
            std::vector<Direction> result;
            for (Direction dir : ALL_DIRECTIONS)
            {
                GridPoint next = Maze::step(cell, dir);
                if (maze.inBounds(next) && visited[next.y * maze.getWidth() + next.x] == wantVisited)
                {
                    result.push_back(dir);
                }
            }
            return result;
        }
    }

    Maze::Maze(int width, int height) : mWidth(width), mHeight(height)
    {
        resetWalls();
    }

    bool Maze::hasWall(GridPoint cell, Direction dir) const
    {
        return (mWalls[cell.y * mWidth + cell.x] & bit(dir)) != 0;
    }

    void Maze::removeWall(GridPoint cell, Direction dir)
    {
        mWalls[cell.y * mWidth + cell.x] &= (std::uint8_t)~bit(dir);
        GridPoint other = step(cell, dir);
        if (inBounds(other))
        {
            mWalls[other.y * mWidth + other.x] &= (std::uint8_t)~bit(opposite(dir));
        }
    }

    void Maze::resetWalls()
    {
        mWalls.assign((std::size_t)mWidth * mHeight, 0x0F);
    }

    GridPoint Maze::step(GridPoint p, Direction dir)
    {
        switch (dir)
        {
        case Direction::NORTH:
            return { p.x, p.y - 1 };
        case Direction::EAST:
            return { p.x + 1, p.y };
        case Direction::SOUTH:
            return { p.x, p.y + 1 };
        default:
            return { p.x - 1, p.y };
        }
    }

    GridMap Maze::toGridMap() const
    {
        GridMap map(mWidth * 2 + 1, mHeight * 2 + 1);
        for (int y = 0; y < map.getHeight(); y++)
        {
            for (int x = 0; x < map.getWidth(); x++)
            {
                map.setBlocked({ x, y }, true);
            }
        }

        for (int y = 0; y < mHeight; y++)
        {
            for (int x = 0; x < mWidth; x++)
            {
                GridPoint cell{ x * 2 + 1, y * 2 + 1 };
                map.setBlocked(cell, false);
                if (!hasWall({ x, y }, Direction::EAST))
                {
                    map.setBlocked({ cell.x + 1, cell.y }, false);
                }
                if (!hasWall({ x, y }, Direction::SOUTH))
                {
                    map.setBlocked({ cell.x, cell.y + 1 }, false);
                }
            }
        }
        return map;
    }

    void RecursiveBacktracker::begin(Maze& maze)
    {
        maze.resetWalls();
        mVisited.assign((std::size_t)maze.getWidth() * maze.getHeight(), false);
        mStack.clear();
        mStack.push_back({ 0, 0 });
        mVisited[0] = true;
    }

    bool RecursiveBacktracker::step(Maze& maze)
    {
        if (mStack.empty())
        {
            return false;
        }

        GridPoint current = mStack.back();
        std::vector<Direction> options = neighborsWhere(maze, mVisited, current, false);
        if (options.empty())
        {
            mStack.pop_back(); // Dead end: backtrack
            return !mStack.empty();
        }

        Direction dir = options.size() == 1 ? options[0] : options[randomIndex(options.size())];
        GridPoint next = Maze::step(current, dir);
        maze.removeWall(current, dir);
        mVisited[next.y * maze.getWidth() + next.x] = true;
        mStack.push_back(next);
        return true;
    }

    void PrimGenerator::begin(Maze& maze)
    {
        maze.resetWalls();
        mState.assign((std::size_t)maze.getWidth() * maze.getHeight(), 0);
        mFrontier.clear();
        mState[0] = 2;
        addFrontier(maze, { 0, 0 });
    }

    void PrimGenerator::addFrontier(const Maze& maze, GridPoint cell)
    {
        for (Direction dir : ALL_DIRECTIONS)
        {
            GridPoint next = Maze::step(cell, dir);
            if (maze.inBounds(next) && mState[next.y * maze.getWidth() + next.x] == 0)
            {
                mState[next.y * maze.getWidth() + next.x] = 1;
                mFrontier.push_back(next);
            }
        }
    }

    bool PrimGenerator::step(Maze& maze)
    {
        if (mFrontier.empty())
        {
            return false;
        }

        int pick = randomIndex(mFrontier.size());
        GridPoint cell = mFrontier[pick];
        mFrontier[pick] = mFrontier.back(); // Order doesn't matter, so swap-remove
        mFrontier.pop_back();

        // Join it to a random neighbor already in the maze
        std::vector<Direction> inMaze;
        for (Direction dir : ALL_DIRECTIONS)
        {
            GridPoint next = Maze::step(cell, dir);
            if (maze.inBounds(next) && mState[next.y * maze.getWidth() + next.x] == 2)
            {
                inMaze.push_back(dir);
            }
        }
        maze.removeWall(cell, inMaze[randomIndex(inMaze.size())]);
        mState[cell.y * maze.getWidth() + cell.x] = 2;
        addFrontier(maze, cell);
        return !mFrontier.empty();
    }

    void HuntAndKill::begin(Maze& maze)
    {
        maze.resetWalls();
        mVisited.assign((std::size_t)maze.getWidth() * maze.getHeight(), false);
        mCurrent = { 0, 0 };
        mVisited[0] = true;
        mDone = false;
    }

    bool HuntAndKill::step(Maze& maze)
    {
        if (mDone)
        {
            return false;
        }

        std::vector<Direction> options = neighborsWhere(maze, mVisited, mCurrent, false);
        if (!options.empty())
        {
            Direction dir = options[randomIndex(options.size())];
            GridPoint next = Maze::step(mCurrent, dir);
            maze.removeWall(mCurrent, dir);
            mVisited[next.y * maze.getWidth() + next.x] = true;
            mCurrent = next;
            return true;
        }

        // Hunt: first unvisited cell (scanning rows) that touches the maze; join it and walk from there
        for (int y = 0; y < maze.getHeight(); y++)
        {
            for (int x = 0; x < maze.getWidth(); x++)
            {
                if (mVisited[y * maze.getWidth() + x])
                {
                    continue;
                }
                std::vector<Direction> joins = neighborsWhere(maze, mVisited, { x, y }, true);
                if (!joins.empty())
                {
                    maze.removeWall({ x, y }, joins[randomIndex(joins.size())]);
                    mVisited[y * maze.getWidth() + x] = true;
                    mCurrent = { x, y };
                    return true;
                }
            }
        }

        mDone = true;
        return false;
    }
}
