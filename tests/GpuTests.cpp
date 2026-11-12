#include <doctest/doctest.h>

#ifdef TEMPLATE_HAS_GPU_COMPUTE
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

#include <gpu/BallSimulation.h>

namespace
{
    const gpu::BallSettings NO_GRAVITY{ 0.0f, 1.0f, 1.0f, 4 };

    bool waitFor(gpu::Context& context, const std::function<bool()>& condition, double seconds = 10.0)
    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < deadline)
        {
            context.poll();
            if (condition())
            {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return condition();
    }

    // Starts WebGPU once for every test; null when this machine has no adapter
    gpu::Context* getContext()
    {
        static gpu::Context context;
        static bool started = false;
        if (!started)
        {
            started = true;
            context.start();
            waitFor(context, [&]() { return context.getState() != gpu::ContextState::STARTING; });
        }
        return context.isReady() ? &context : nullptr;
    }

    // Two equal balls flying at each other in the middle of the box
    void headOn(std::vector<gpu::Float4>& spheres, std::vector<gpu::Float4>& velocities)
    {
        spheres = { { -1.0f, 0.0f, 0.0f, 0.5f }, { 1.0f, 0.0f, 0.0f, 0.5f } };
        velocities = { { 2.0f, 0.0f, 0.0f, 1.0f }, { -2.0f, 0.0f, 0.0f, 1.0f } };
    }

    std::vector<gpu::Float4> makeCrowd(int count, float half, std::vector<gpu::Float4>& velocities)
    {
        std::vector<gpu::Float4> spheres;
        velocities.clear();
        const int perSide = (int)std::ceil(std::cbrt((double)count));
        const float spacing = (2.0f * half - 1.0f) / (float)perSide;
        for (int i = 0; i < count; i++)
        {
            const int x = i % perSide;
            const int y = (i / perSide) % perSide;
            const int z = i / (perSide * perSide);
            spheres.push_back({ -half + 0.5f + spacing * (x + 0.5f), -half + 0.5f + spacing * (y + 0.5f), -half + 0.5f + spacing * (z + 0.5f), spacing * 0.4f });
            velocities.push_back({ (float)((i * 7) % 5) - 2.0f, 0.0f, (float)((i * 3) % 5) - 2.0f, 1.0f });
        }
        return spheres;
    }
}

TEST_CASE("BallGrid covers the box with cells at least as wide as the biggest ball")
{
    gpu::BallGrid grid = gpu::BallGrid::make(0.5f, 5.0f);
    CHECK(grid.cellSize == doctest::Approx(1.0f));
    CHECK(grid.dims == 11);
    CHECK(grid.cellCount == 11u * 11u * 11u);
    CHECK(grid.blockCount == (grid.cellCount + 1 + 255) / 256);

    // Tiny balls in a huge box: cells grow so the grid stays bounded
    gpu::BallGrid capped = gpu::BallGrid::make(0.001f, 1000.0f);
    CHECK(capped.dims <= gpu::BallGrid::kMaxCellsPerSide);
}

TEST_CASE("CpuBallSimulation: equal balls swap velocities in an elastic head-on hit")
{
    std::vector<gpu::Float4> spheres;
    std::vector<gpu::Float4> velocities;
    headOn(spheres, velocities);
    gpu::CpuBallSimulation simulation;
    simulation.reset(spheres, velocities, 5.0f);
    for (int i = 0; i < 50; i++)
    {
        simulation.step(0.01f, NO_GRAVITY);
    }
    CHECK(simulation.getVelocities()[0].x == doctest::Approx(-2.0f).epsilon(0.001));
    CHECK(simulation.getVelocities()[1].x == doctest::Approx(2.0f).epsilon(0.001));
    CHECK(simulation.getSpheres()[1].x - simulation.getSpheres()[0].x >= 1.0f);
}

TEST_CASE("CpuBallSimulation keeps a crowd inside the box")
{
    std::vector<gpu::Float4> velocities;
    std::vector<gpu::Float4> spheres = makeCrowd(500, 4.0f, velocities);
    gpu::CpuBallSimulation simulation;
    simulation.reset(spheres, velocities, 4.0f);
    for (int i = 0; i < 200; i++)
    {
        simulation.step(0.02f, gpu::BallSettings{});
    }
    for (const gpu::Float4& sphere : simulation.getSpheres())
    {
        REQUIRE(std::isfinite(sphere.y));
        CHECK(std::abs(sphere.x) <= 4.0f - sphere.w + 0.01f);
        CHECK(std::abs(sphere.y) <= 4.0f - sphere.w + 0.01f);
        CHECK(std::abs(sphere.z) <= 4.0f - sphere.w + 0.01f);
    }
    CHECK(simulation.getStats().wallContacts > 0u);
}

TEST_CASE("GpuBallSimulation matches the CPU")
{
    gpu::Context* pContext = getContext();
    if (pContext == nullptr)
    {
        MESSAGE("No WebGPU adapter on this machine, skipping");
        return;
    }
    gpu::GpuBallSimulation gpuSimulation;
    REQUIRE(gpuSimulation.init(*pContext));

    auto runBoth = [&](const std::vector<gpu::Float4>& spheres, const std::vector<gpu::Float4>& velocities,
                       float half, const gpu::BallSettings& settings, int steps, gpu::CpuBallSimulation& cpu)
    {
        cpu.reset(spheres, velocities, half);
        gpuSimulation.reset(spheres, velocities, half);
        for (int i = 0; i < steps; i++)
        {
            cpu.step(0.01f, settings);
            gpuSimulation.step(0.01f, settings);
        }
        gpuSimulation.requestReadback();
        return waitFor(*pContext, [&]() { return gpuSimulation.poll(); });
    };

    SUBCASE("a head-on hit")
    {
        std::vector<gpu::Float4> spheres;
        std::vector<gpu::Float4> velocities;
        headOn(spheres, velocities);
        gpu::CpuBallSimulation cpu;
        REQUIRE(runBoth(spheres, velocities, 5.0f, NO_GRAVITY, 50, cpu));
        CHECK(gpuSimulation.getStepsBehind() == 0u);
        for (int i = 0; i < 2; i++)
        {
            CHECK(gpuSimulation.getSpheres()[i].x == doctest::Approx(cpu.getSpheres()[i].x).epsilon(0.001));
            CHECK(gpuSimulation.getVelocities()[i].x == doctest::Approx(cpu.getVelocities()[i].x).epsilon(0.001));
        }
    }

    SUBCASE("a ball bouncing on the floor")
    {
        std::vector<gpu::Float4> spheres = { { 0.0f, 2.0f, 0.0f, 0.5f } };
        std::vector<gpu::Float4> velocities = { { 0.0f, 0.0f, 0.0f, 1.0f } };
        gpu::CpuBallSimulation cpu;
        REQUIRE(runBoth(spheres, velocities, 3.0f, gpu::BallSettings{}, 150, cpu));
        CHECK(gpuSimulation.getSpheres()[0].y == doctest::Approx(cpu.getSpheres()[0].y).epsilon(0.01));
        CHECK(gpuSimulation.getVelocities()[0].y == doctest::Approx(cpu.getVelocities()[0].y).epsilon(0.01));
    }

    SUBCASE("a crowd stays in the box and counts contacts like the CPU")
    {
        std::vector<gpu::Float4> velocities;
        std::vector<gpu::Float4> spheres = makeCrowd(4000, 6.0f, velocities);
        gpu::CpuBallSimulation cpu;
        REQUIRE(runBoth(spheres, velocities, 6.0f, gpu::BallSettings{}, 100, cpu));
        REQUIRE(gpuSimulation.getBallCount() == spheres.size());
        for (const gpu::Float4& sphere : gpuSimulation.getSpheres())
        {
            REQUIRE(std::isfinite(sphere.x));
            CHECK(std::abs(sphere.y) <= 6.0f - sphere.w + 0.01f);
        }
        // Ball order within a cell differs between the two, so the runs drift apart a little
        const double cpuWalls = cpu.getStats().wallContacts;
        const double gpuWalls = gpuSimulation.getStats().wallContacts;
        CHECK(gpuWalls == doctest::Approx(cpuWalls).epsilon(0.25));
        CHECK(gpuSimulation.getStats().ballContacts > 0u);
    }
}
#endif
