#include <doctest/doctest.h>

#include <cmath>

#include <ai/CellularAutomaton.h>
#include <ai/Flocking.h>
#include <ai/GridPath.h>
#include <ai/Maze.h>

namespace
{
    // A perfect maze has exactly one path between any two cells: connected, with cells - 1 openings
    void checkPerfectMaze(const ai::Maze& maze)
    {
        int openings = 0;
        for (int y = 0; y < maze.getHeight(); y++)
        {
            for (int x = 0; x < maze.getWidth(); x++)
            {
                openings += !maze.hasWall({ x, y }, ai::Direction::EAST) && x + 1 < maze.getWidth();
                openings += !maze.hasWall({ x, y }, ai::Direction::SOUTH) && y + 1 < maze.getHeight();
            }
        }
        CHECK(openings == maze.getWidth() * maze.getHeight() - 1);

        ai::GridMap map = maze.toGridMap();
        std::vector<int> field = ai::buildDistanceField(map, { 1, 1 });
        for (int y = 0; y < maze.getHeight(); y++)
        {
            for (int x = 0; x < maze.getWidth(); x++)
            {
                CHECK(field[map.index({ x * 2 + 1, y * 2 + 1 })] >= 0);
            }
        }
    }
}

TEST_CASE("Every maze generator makes a perfect maze")
{
    ai::Maze maze(12, 9);

    ai::RecursiveBacktracker backtracker(1);
    backtracker.generate(maze);
    checkPerfectMaze(maze);

    ai::PrimGenerator prim(2);
    prim.generate(maze);
    checkPerfectMaze(maze);

    ai::HuntAndKill huntAndKill(3);
    huntAndKill.generate(maze);
    checkPerfectMaze(maze);
}

TEST_CASE("The same seed builds the same maze")
{
    ai::Maze a(8, 8);
    ai::Maze b(8, 8);
    ai::RecursiveBacktracker(42).generate(a);
    ai::RecursiveBacktracker(42).generate(b);
    for (int y = 0; y < 8; y++)
    {
        for (int x = 0; x < 8; x++)
        {
            CHECK(a.hasWall({ x, y }, ai::Direction::EAST) == b.hasWall({ x, y }, ai::Direction::EAST));
            CHECK(a.hasWall({ x, y }, ai::Direction::SOUTH) == b.hasWall({ x, y }, ai::Direction::SOUTH));
        }
    }
}

TEST_CASE("A* and BFS go around walls and agree on length")
{
    ai::GridMap map(10, 10);
    for (int y = 0; y < 9; y++)
    {
        map.setBlocked({ 5, y }, true); // Wall with a gap at the bottom
    }

    auto bfs = ai::findPathBFS(map, { 0, 0 }, { 9, 0 });
    auto astar = ai::findPathAStar(map, { 0, 0 }, { 9, 0 });
    REQUIRE_FALSE(bfs.empty());
    REQUIRE_FALSE(astar.empty());
    CHECK(bfs.front() == ai::GridPoint{ 0, 0 });
    CHECK(astar.back() == ai::GridPoint{ 9, 0 });
    CHECK(bfs.size() == astar.size());
    CHECK(bfs.size() == 28); // Down 9, across 9, up 9, plus the start cell

    for (const ai::GridPoint& p : astar)
    {
        CHECK(map.isWalkable(p));
    }

    map.setBlocked({ 5, 9 }, true);
    CHECK(ai::findPathAStar(map, { 0, 0 }, { 9, 0 }).empty());
}

TEST_CASE("Diagonal A* cuts across open ground but not past corners")
{
    ai::GridMap open(5, 5);
    CHECK(ai::findPathAStar(open, { 0, 0 }, { 4, 4 }, true).size() == 5);

    ai::GridMap corner(2, 2);
    corner.setBlocked({ 1, 0 }, true);
    corner.setBlocked({ 0, 1 }, true);
    CHECK(ai::findPathAStar(corner, { 0, 0 }, { 1, 1 }, true).empty());
}

TEST_CASE("Game of Life: a blinker oscillates and a block stays still")
{
    ai::CellularAutomaton life(8, 8);
    life.setAlive(2, 1, true);
    life.setAlive(2, 2, true);
    life.setAlive(2, 3, true);

    life.step();
    CHECK(life.isAlive(1, 2));
    CHECK(life.isAlive(3, 2));
    CHECK_FALSE(life.isAlive(2, 1));
    life.step();
    CHECK(life.isAlive(2, 1));
    CHECK(life.countAlive() == 3);

    ai::CellularAutomaton block(6, 6);
    block.setAlive(4, 4, true);
    block.setAlive(5, 4, true);
    block.setAlive(4, 5, true);
    block.setAlive(5, 5, true);
    block.step();
    CHECK(block.countAlive() == 4);
}

TEST_CASE("Flocking: separation spreads boids and bounds keep them inside")
{
    ai::Flock flock;
    flock.addRule<ai::SeparationRule>(3.0f, 2.0f);
    flock.addRule<ai::BoundedAreaRule>(ai::Vec2{ -10, -10 }, ai::Vec2{ 10, 10 }, 3.0f, 5.0f);
    flock.addBoid({ 0.0f, 0.0f });
    flock.addBoid({ 0.5f, 0.0f });

    for (int i = 0; i < 200; i++)
    {
        flock.update(0.02f);
    }

    const auto& boids = flock.getBoids();
    CHECK((boids[0].position - boids[1].position).length() > 1.5f);
    for (const ai::Boid& boid : boids)
    {
        CHECK(std::abs(boid.position.x) < 10.0f);
        CHECK(boid.velocity.length() <= flock.maxSpeed + 1e-3f);
    }
}

TEST_CASE("Flocking: alignment brings headings together")
{
    ai::Flock flock;
    flock.addRule<ai::AlignmentRule>(4.0f);
    flock.addBoid({ 0, 0 }, { 1, 0 });
    flock.addBoid({ 1, 0 }, { 0, 1 });

    for (int i = 0; i < 100; i++)
    {
        flock.update(0.02f);
    }

    ai::Vec2 a = flock.getBoids()[0].velocity.normalized();
    ai::Vec2 b = flock.getBoids()[1].velocity.normalized();
    CHECK(a.x * b.x + a.y * b.y > 0.99f);
}
