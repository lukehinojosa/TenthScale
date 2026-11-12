#include "flight/BuildScene.h"

#include <cmath>

#include <imgui.h>
#include <raymath.h>
#include <rlgl.h>

#include "flight/PartDrawing.h"

namespace tenth
{
    namespace
    {
        constexpr float PANEL_WIDTH = 280.0f;
        constexpr float PLATE_RADIUS = 3.0f;
        constexpr float PLATE_THICKNESS = 0.3f;
        constexpr float CLICK_SLOP = 5.0f; // pixels the mouse may move between press and release and still count as a click
        constexpr Color GROUND_COLOR = { 52, 58, 52, 255 };
        constexpr Color PLATE_COLOR = { 96, 100, 108, 255 };
        constexpr Color PLATE_EDGE_COLOR = { 230, 180, 40, 255 };
        constexpr Color SELECTED_OUTLINE = { 255, 230, 60, 255 };
        constexpr Color INVALID_COLOR = { 220, 60, 50, 255 };

        constexpr PartCategory CATEGORY_ORDER[] = { PartCategory::Pod,      PartCategory::Tank, PartCategory::Engine,
                                                    PartCategory::Coupling, PartCategory::Aero, PartCategory::Utility };

        ImVec4 toImGui(Color color)
        {
            return { color.r / 255.0f, color.g / 255.0f, color.b / 255.0f, color.a / 255.0f };
        }

        void partTooltip(const PartDefinition& part)
        {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 22.0f);
            ImGui::TextUnformatted(part.name.c_str());
            ImGui::TextDisabled("%s", part.description.c_str());
            ImGui::Separator();
            if (part.fuelMass > 0.0)
            {
                ImGui::Text("Mass: %.0f kg empty, %.0f kg full", part.dryMass, part.wetMass());
            }
            else
            {
                ImGui::Text("Mass: %.0f kg", part.dryMass);
            }
            if (part.isEngine())
            {
                ImGui::Text("Thrust: %.0f kN in vacuum", part.thrustVacuum / 1000.0);
                ImGui::Text("Efficiency: %.0f s at sea level, %.0f s in vacuum", part.ispSeaLevel, part.ispVacuum);
            }
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
    }

    BuildScene::BuildScene(Camera3D& camera, const PartCatalog& catalog, RocketDesign& design)
        : mCamera(camera), mCatalog(catalog), mDesign(design)
    {
        mOrbit.leftInset = PANEL_WIDTH;
    }

    void BuildScene::onEnter()
    {
        mOrbit.apply(mCamera);
    }

    bool BuildScene::mouseOverViewport() const
    {
        return GetMousePosition().x > PANEL_WIDTH;
    }

    // A rocket starts from a pod on an empty plate; attaching more parts comes with stacking
    bool BuildScene::canDrop(const PartDefinition& definition) const
    {
        return mDesign.empty() && definition.category == PartCategory::Pod;
    }

    Vector3 BuildScene::designOffset() const
    {
        return { 0.0f, (float)-mDesign.lowestPoint(), 0.0f };
    }

    PartIndex BuildScene::pickPart(Ray ray) const
    {
        const Vector3 offset = designOffset();
        PartIndex closest = NO_PART;
        float closestDistance = 0.0f;
        for (PartIndex i = 0; i < mDesign.partCount(); i++)
        {
            const PlacedPart& placed = mDesign.part(i);
            const Vector3 position = Vector3Add(toRaylib(placed.position), offset);
            const RayCollision hit = GetRayCollisionBox(ray, partBounds(*placed.pDefinition, position, (float)placed.angle));
            if (hit.hit && (closest == NO_PART || hit.distance < closestDistance))
            {
                closest = i;
                closestDistance = hit.distance;
            }
        }
        return closest;
    }

    void BuildScene::onUpdate(double frameDt, double /*alpha*/)
    {
        mOrbit.update(mCamera, (float)frameDt);
        const Ray ray = GetScreenToWorldRay(GetMousePosition(), mCamera);
        if (mpDragged != nullptr)
        {
            updateDrag(ray);
        }
        else
        {
            updateSelection(ray);
        }
    }

    void BuildScene::updateDrag(Ray ray)
    {
        mDragOverPlate = false;
        if (ray.direction.y < 0.0f)
        {
            const float t = -ray.position.y / ray.direction.y;
            mDragPoint = Vector3Add(ray.position, Vector3Scale(ray.direction, t));
            mDragOverPlate = mouseOverViewport() && std::hypot(mDragPoint.x, mDragPoint.z) <= PLATE_RADIUS;
        }

        // The drag started in the panel, so ImGui holds the mouse; releasing it anywhere ends the drag
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
        {
            if (mDragOverPlate && canDrop(*mpDragged))
            {
                mSelected = mDesign.setRoot(*mpDragged);
            }
            mpDragged = nullptr;
        }
    }

    void BuildScene::updateSelection(Ray ray)
    {
        const bool imguiOwnsMouse = ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureMouse;
        const bool inViewport = mouseOverViewport() && !imguiOwnsMouse;
        mHovered = inViewport ? pickPart(ray) : NO_PART;

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        {
            mPressedInViewport = inViewport;
            mPressPosition = GetMousePosition();
        }

        // A drag that moved is the camera or a slip of the hand, not a click
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && mPressedInViewport)
        {
            mPressedInViewport = false;
            if (Vector2Distance(mPressPosition, GetMousePosition()) <= CLICK_SLOP)
            {
                mSelected = mHovered;
            }
        }
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

        const Vector3 offset = designOffset();
        for (PartIndex i = 0; i < mDesign.partCount(); i++)
        {
            const PlacedPart& placed = mDesign.part(i);
            Color color = categoryColor(placed.pDefinition->category);
            if (i == mHovered)
            {
                color = ColorBrightness(color, 0.25f);
            }
            const Color outline = i == mSelected ? SELECTED_OUTLINE : ColorBrightness(color, -0.5f);
            drawPart(*placed.pDefinition, Vector3Add(toRaylib(placed.position), offset), (float)placed.angle, color, outline);
        }

        // A see-through copy shows where the dragged part would land: centred on the plate, or red where it can't go
        if (mpDragged != nullptr && mouseOverViewport())
        {
            const bool valid = mDragOverPlate && canDrop(*mpDragged);
            const float halfHeight = (float)mpDragged->shape.height * 0.5f;
            const Vector3 position = valid ? Vector3{ 0.0f, halfHeight, 0.0f } : Vector3{ mDragPoint.x, halfHeight, mDragPoint.z };
            const Color color = valid ? categoryColor(mpDragged->category) : INVALID_COLOR;
            drawPart(*mpDragged, position, 0.0f, ColorAlpha(color, 0.45f), ColorAlpha(ColorBrightness(color, -0.4f), 0.8f));
        }
    }

    void BuildScene::onGui()
    {
        const ImGuiViewport* pViewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(pViewport->WorkPos);
        ImGui::SetNextWindowSize({ PANEL_WIDTH, pViewport->WorkSize.y });
        const ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
        ImGui::Begin("Parts", nullptr, flags);

        const ImGuiStyle& style = ImGui::GetStyle();
        const float footerHeight =
            ImGui::GetTextLineHeightWithSpacing() * 4.0f + ImGui::GetFrameHeightWithSpacing() * 2.0f + style.ItemSpacing.y * 2.0f;
        ImGui::BeginChild("PartList", { 0.0f, -footerHeight });
        drawPartList();
        ImGui::EndChild();
        drawFooter();
        ImGui::End();

        if (mpDragged != nullptr)
        {
            if (!mouseOverViewport())
            {
                ImGui::SetTooltip("Drop %s on the build plate", mpDragged->name.c_str());
            }
            else if (!mDesign.empty())
            {
                ImGui::SetTooltip("Attaching parts isn't built yet: Clear to start over");
            }
            else if (!canDrop(*mpDragged))
            {
                ImGui::SetTooltip("A rocket starts from a command pod");
            }
        }
    }

    void BuildScene::drawPartList()
    {
        const float swatchSize = ImGui::GetTextLineHeight();
        for (PartCategory category : CATEGORY_ORDER)
        {
            if (!ImGui::CollapsingHeader(categoryName(category), ImGuiTreeNodeFlags_DefaultOpen))
            {
                continue;
            }
            for (const PartDefinition& part : mCatalog.parts())
            {
                if (part.category != category)
                {
                    continue;
                }
                ImGui::PushID(part.id.c_str());
                ImGui::ColorButton("##swatch", toImGui(categoryColor(category)),
                                   ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop, { swatchSize, swatchSize });
                ImGui::SameLine();
                ImGui::Selectable(part.name.c_str(), mpDragged == &part);
                if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
                {
                    mpDragged = &part;
                }
                else if (ImGui::IsItemHovered() && mpDragged == nullptr)
                {
                    partTooltip(part);
                }
                ImGui::PopID();
            }
        }
    }

    void BuildScene::drawFooter()
    {
        ImGui::Separator();
        if (mSelected != NO_PART && mSelected < mDesign.partCount())
        {
            ImGui::Text("Selected: %s", mDesign.part(mSelected).pDefinition->name.c_str());
        }
        else
        {
            ImGui::TextDisabled("Drag a pod onto the plate to start");
        }
        ImGui::Text("Parts: %d   Mass: %.2f t", mDesign.partCount(), mDesign.totalMass() / 1000.0);
        ImGui::TextDisabled("Right-drag to orbit, wheel to zoom");
        ImGui::TextDisabled("Middle-drag or W/S to move up and down");
        if (ImGui::Button("Clear", { -1.0f, 0.0f }))
        {
            mDesign.clear();
            mSelected = NO_PART;
        }
        if (ImGui::Button("Launch", { -1.0f, 0.0f }))
        {
            requestScene(SceneId::Flight);
        }
    }
}
