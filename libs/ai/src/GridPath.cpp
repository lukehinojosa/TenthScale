#include "ai/GridPath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace ai
{
    namespace
    {
        std::vector<GridPoint> rebuildPath(const GridMap& map, const std::vector<int>& cameFrom, GridPoint start, GridPoint goal)
        {
            std::vector<GridPoint> path;
            int current = map.index(goal);
            const int startIndex = map.index(start);
            while (current != startIndex)
            {
                path.push_back({ current % map.getWidth(), current / map.getWidth() });
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Octile distance: exact cost on an open 8-way grid, Manhattan on a 4-way one
        float heuristic(GridPoint a, GridPoint b, bool diagonals)
        {
            float dx = (float)std::abs(a.x - b.x);
            float dy = (float)std::abs(a.y - b.y);
            if (!diagonals)
            {
                return dx + dy;
            }
            return (dx + dy) + (std::sqrt(2.0f) - 2.0f) * std::min(dx, dy);
        }
    }

    std::vector<GridPoint> GridMap::getNeighbors(GridPoint p, bool diagonals) const
    {
        std::vector<GridPoint> result;
        const GridPoint straight[] = { { p.x, p.y - 1 }, { p.x + 1, p.y }, { p.x, p.y + 1 }, { p.x - 1, p.y } };
        for (GridPoint n : straight)
        {
            if (isWalkable(n))
            {
                result.push_back(n);
            }
        }

        if (diagonals)
        {
            const int offsets[4][2] = { { 1, -1 }, { 1, 1 }, { -1, 1 }, { -1, -1 } };
            for (const auto& offset : offsets)
            {
                GridPoint n{ p.x + offset[0], p.y + offset[1] };
                // Both orthogonal cells must be open, so paths can't slip between two blocked corners
                if (isWalkable(n) && isWalkable({ p.x + offset[0], p.y }) && isWalkable({ p.x, p.y + offset[1] }))
                {
                    result.push_back(n);
                }
            }
        }
        return result;
    }

    std::vector<GridPoint> findPathBFS(const GridMap& map, GridPoint start, GridPoint goal, bool diagonals)
    {
        if (!map.isWalkable(start) || !map.isWalkable(goal))
        {
            return {};
        }

        std::vector<int> cameFrom((std::size_t)map.getWidth() * map.getHeight(), -1);
        std::vector<bool> visited(cameFrom.size(), false);
        std::queue<GridPoint> frontier;
        frontier.push(start);
        visited[map.index(start)] = true;

        while (!frontier.empty())
        {
            GridPoint current = frontier.front();
            frontier.pop();
            if (current == goal)
            {
                return rebuildPath(map, cameFrom, start, goal);
            }

            for (GridPoint next : map.getNeighbors(current, diagonals))
            {
                if (!visited[map.index(next)])
                {
                    visited[map.index(next)] = true;
                    cameFrom[map.index(next)] = map.index(current);
                    frontier.push(next);
                }
            }
        }
        return {};
    }

    std::vector<GridPoint> findPathAStar(const GridMap& map, GridPoint start, GridPoint goal, bool diagonals)
    {
        if (!map.isWalkable(start) || !map.isWalkable(goal))
        {
            return {};
        }

        const std::size_t cellCount = (std::size_t)map.getWidth() * map.getHeight();
        std::vector<float> costSoFar(cellCount, std::numeric_limits<float>::infinity());
        std::vector<int> cameFrom(cellCount, -1);

        struct Entry
        {
            float priority;
            int index;
            bool operator>(const Entry& o) const { return priority > o.priority; }
        };
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

        costSoFar[map.index(start)] = 0.0f;
        open.push({ heuristic(start, goal, diagonals), map.index(start) });

        while (!open.empty())
        {
            Entry entry = open.top();
            open.pop();
            GridPoint current{ entry.index % map.getWidth(), entry.index / map.getWidth() };
            if (current == goal)
            {
                return rebuildPath(map, cameFrom, start, goal);
            }

            // Stale entry: a cheaper route to this cell was already expanded
            if (entry.priority - heuristic(current, goal, diagonals) > costSoFar[entry.index] + 1e-4f)
            {
                continue;
            }

            for (GridPoint next : map.getNeighbors(current, diagonals))
            {
                bool isDiagonal = next.x != current.x && next.y != current.y;
                float newCost = costSoFar[entry.index] + (isDiagonal ? std::sqrt(2.0f) : 1.0f);
                int nextIndex = map.index(next);
                if (newCost < costSoFar[nextIndex])
                {
                    costSoFar[nextIndex] = newCost;
                    cameFrom[nextIndex] = entry.index;
                    open.push({ newCost + heuristic(next, goal, diagonals), nextIndex });
                }
            }
        }
        return {};
    }

    std::vector<int> buildDistanceField(const GridMap& map, GridPoint goal, bool diagonals)
    {
        std::vector<int> distance((std::size_t)map.getWidth() * map.getHeight(), -1);
        if (!map.isWalkable(goal))
        {
            return distance;
        }

        std::queue<GridPoint> frontier;
        frontier.push(goal);
        distance[map.index(goal)] = 0;
        while (!frontier.empty())
        {
            GridPoint current = frontier.front();
            frontier.pop();
            for (GridPoint next : map.getNeighbors(current, diagonals))
            {
                if (distance[map.index(next)] < 0)
                {
                    distance[map.index(next)] = distance[map.index(current)] + 1;
                    frontier.push(next);
                }
            }
        }
        return distance;
    }
}
