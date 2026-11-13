#include "flight/LaunchPadScene.h"

#include <algorithm>

#include <imgui.h>

#include "flight/DesignStats.h"
#include "flight/PartDrawing.h"

namespace tenth
{
    namespace
    {
        constexpr float PAD_HALF_WIDTH = 8.0f;
        constexpr float PAD_HEIGHT = 2.0f; // the pad's top is at height zero, standing this far above the ground
        constexpr Vector3 TOWER_POSITION = { -2.8f, 0.0f, -3.4f }; // behind the rocket from where the camera starts
        constexpr float TOWER_WIDTH = 1.4f;
        constexpr Vector3 SUN_DIRECTION = { 0.45f, 0.75f, 0.48f };
        constexpr Color GROUND_COLOR = { 74, 96, 62, 255 };
        constexpr Color PAD_COLOR = { 150, 148, 140, 255 };
        constexpr Color TRENCH_COLOR = { 40, 40, 44, 255 };
        constexpr Color TOWER_COLOR = { 170, 70, 50, 255 };
    }

    LaunchPadScene::LaunchPadScene(Camera3D& camera, const RocketDesign& design, const SolarSystem& solarSystem)
        : mCamera(camera), mDesign(design)
    {
        const BodyId earth = solarSystem.findBody("Earth");
        if (earth != NO_BODY)
        {
            mSurfaceGravity = solarSystem.surfaceGravity(earth);
        }
        mOrbit.minPitch = 0.02f;
        mOrbit.maxDistance = 200.0f;
    }

    // Frames the rocket from a little way off, so a taller rocket starts further away
    void LaunchPadScene::onEnter()
    {
        if (!mSkyReady)
        {
            mSkyReady = mSky.init();
        }
        const float height = (float)(mDesign.highestPoint() - mDesign.lowestPoint());
        mOrbit.focus = { 0.0f, height * 0.5f, 0.0f };
        mOrbit.distance = std::max(14.0f, height * 1.8f);
        mOrbit.yaw = 0.6f;
        mOrbit.pitch = 0.15f;
        mOrbit.apply(mCamera);
    }

    void LaunchPadScene::onUpdate(double frameDt, double /*alpha*/)
    {
        mOrbit.update(mCamera, (float)frameDt);
    }

    void LaunchPadScene::onShutdown()
    {
        if (mSkyReady)
        {
            mSky.unload();
            mSkyReady = false;
        }
    }

    void LaunchPadScene::onDraw3D()
    {
        if (mSkyReady)
        {
            mSky.draw(SUN_DIRECTION);
        }
        DrawPlane({ 0.0f, -PAD_HEIGHT, 0.0f }, { 1800.0f, 1800.0f }, GROUND_COLOR);
        DrawCube({ 0.0f, -PAD_HEIGHT * 0.5f, 0.0f }, PAD_HALF_WIDTH * 2.0f, PAD_HEIGHT, PAD_HALF_WIDTH * 2.0f, PAD_COLOR);
        DrawCubeWires({ 0.0f, -PAD_HEIGHT * 0.5f, 0.0f }, PAD_HALF_WIDTH * 2.0f, PAD_HEIGHT, PAD_HALF_WIDTH * 2.0f,
                      ColorBrightness(PAD_COLOR, -0.4f));

        // The flame trench opening under the rocket, drawn a hair above the pad so it doesn't flicker
        DrawCube({ 0.0f, 0.01f, 0.0f }, 2.4f, 0.02f, 2.4f, TRENCH_COLOR);

        // The service tower beside the rocket, a little taller than it
        const float towerHeight = (float)(mDesign.highestPoint() - mDesign.lowestPoint()) + 4.0f;
        const Vector3 towerCentre = { TOWER_POSITION.x, towerHeight * 0.5f, TOWER_POSITION.z };
        DrawCube(towerCentre, TOWER_WIDTH, towerHeight, TOWER_WIDTH, TOWER_COLOR);
        DrawCubeWires(towerCentre, TOWER_WIDTH, towerHeight, TOWER_WIDTH, ColorBrightness(TOWER_COLOR, -0.5f));

        // Standing on the pad: the rocket's lowest point rests at height zero
        drawDesign(mDesign, { 0.0f, (float)-mDesign.lowestPoint(), 0.0f });
    }

    void LaunchPadScene::onGui()
    {
        const DesignStats stats = computeStats(mDesign, mSurfaceGravity);
        ImGui::SetNextWindowPos({ 12.0f, 12.0f }, ImGuiCond_Always);
        ImGui::Begin("Launch pad", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove);
        ImGui::Text("Mass: %.2f t", mDesign.totalMass() / 1000.0);
        ImGui::Text("Height: %.1f m", mDesign.highestPoint() - mDesign.lowestPoint());
        ImGui::Text("Stages: %d   Delta-v: %.0f m/s", (int)stats.stages.size(), stats.totalDeltaVVacuum);
        if (!stats.stages.empty())
        {
            ImGui::Text("Liftoff TWR: %.2f", stats.stages.front().twrSeaLevel);
        }
        ImGui::TextDisabled("Right-drag orbit, W/S lift, wheel zoom");
        if (ImGui::Button("Back to assembly"))
        {
            requestScene(SceneId::Build);
        }
        ImGui::SameLine();
        if (ImGui::Button("Launch"))
        {
            requestScene(SceneId::Flight);
        }
        ImGui::End();
    }
}
