#include "flight/BuildScene.h"

#include <imgui.h>
#include <rlgl.h>

namespace tenth
{
    namespace
    {
        constexpr float PANEL_WIDTH = 280.0f;
        constexpr float PLATE_RADIUS = 3.0f;
        constexpr float PLATE_THICKNESS = 0.3f;
        constexpr Color GROUND_COLOR = { 52, 58, 52, 255 };
        constexpr Color PLATE_COLOR = { 96, 100, 108, 255 };
        constexpr Color PLATE_EDGE_COLOR = { 230, 180, 40, 255 };
    }

    BuildScene::BuildScene(Camera3D& camera) : mCamera(camera)
    {
        mOrbit.leftInset = PANEL_WIDTH;
    }

    void BuildScene::onEnter()
    {
        mOrbit.apply(mCamera);
    }

    void BuildScene::onUpdate(double frameDt, double /*alpha*/)
    {
        mOrbit.update(mCamera, (float)frameDt);
    }

    // The plate's top is at height zero, so parts placed on it sit at y = 0
    void BuildScene::onDraw3D()
    {
        DrawPlane({ 0.0f, -PLATE_THICKNESS, 0.0f }, { 1800.0f, 1800.0f }, GROUND_COLOR);
        DrawCylinder({ 0.0f, -PLATE_THICKNESS, 0.0f }, PLATE_RADIUS, PLATE_RADIUS, PLATE_THICKNESS, 64, PLATE_COLOR);
        DrawCylinderWires({ 0.0f, -PLATE_THICKNESS, 0.0f }, PLATE_RADIUS, PLATE_RADIUS, PLATE_THICKNESS, 64, PLATE_EDGE_COLOR);

        // Lifted a hair so the grid doesn't flicker against the plate's top
        rlPushMatrix();
        rlTranslatef(0.0f, 0.002f, 0.0f);
        DrawGrid(8, 0.5f);
        rlPopMatrix();
    }

    void BuildScene::onGui()
    {
        const ImGuiViewport* pViewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(pViewport->WorkPos);
        ImGui::SetNextWindowSize({ PANEL_WIDTH, pViewport->WorkSize.y });
        const ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
        ImGui::Begin("Parts", nullptr, flags);

        ImGui::TextDisabled("No parts yet");

        // The launch button and controls sit at the bottom of the panel
        const float footerHeight = ImGui::GetFrameHeightWithSpacing() * 2.0f + ImGui::GetTextLineHeightWithSpacing() * 3.0f;
        ImGui::SetCursorPosY(ImGui::GetWindowHeight() - footerHeight);
        ImGui::Separator();
        ImGui::TextDisabled("Right-drag to orbit, wheel to zoom");
        ImGui::TextDisabled("Middle-drag or W/S to move up and down");
        if (ImGui::Button("Launch", { -1.0f, 0.0f }))
        {
            requestScene(SceneId::Flight);
        }
        ImGui::End();
    }
}
