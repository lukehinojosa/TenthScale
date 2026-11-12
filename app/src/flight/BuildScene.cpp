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
        constexpr float SNAP_RADIUS = 60.0f; // pixels from a node that the held parts snap to it
        constexpr size_t UNDO_LIMIT = 100;
        constexpr Color GROUND_COLOR = { 52, 58, 52, 255 };
        constexpr Color PLATE_COLOR = { 96, 100, 108, 255 };
        constexpr Color PLATE_EDGE_COLOR = { 230, 180, 40, 255 };
        constexpr Color SELECTED_OUTLINE = { 255, 230, 60, 255 };
        constexpr Color INVALID_COLOR = { 220, 60, 50, 255 };
        constexpr Color NODE_COLOR = { 80, 230, 120, 255 };

        constexpr PartCategory CATEGORY_ORDER[] = { PartCategory::Pod,      PartCategory::Tank, PartCategory::Engine,
                                                    PartCategory::Coupling, PartCategory::Aero, PartCategory::Utility };
        constexpr Node NODES[] = { Node::Top, Node::Bottom };

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

        // Where a stack node sits in the world, given the part's centre
        Vector3 nodePoint(const PartDefinition& definition, Vector3 centre, Node node)
        {
            const float halfHeight = (float)definition.shape.height * 0.5f;
            return { centre.x, centre.y + (node == Node::Top ? halfHeight : -halfHeight), centre.z };
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

    bool BuildScene::canStartRocket() const
    {
        return mDesign.empty() && !mHeld.empty() && mHeld.part(0).pDefinition->category == PartCategory::Pod;
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

    // The free node nearest the mouse on screen that the held parts fit, if one is close enough
    std::optional<BuildScene::Snap> BuildScene::findSnap() const
    {
        if (mHeld.empty() || mDesign.empty() || !mouseOverViewport())
        {
            return std::nullopt;
        }
        const Vector3 offset = designOffset();
        const Vector2 mouse = GetMousePosition();
        std::optional<Snap> best;
        float bestDistance = SNAP_RADIUS;
        for (PartIndex i = 0; i < mDesign.partCount(); i++)
        {
            const PlacedPart& placed = mDesign.part(i);
            const Vector3 centre = Vector3Add(toRaylib(placed.position), offset);
            for (Node node : NODES)
            {
                if (!mDesign.canAttachDesign(i, node, mHeld))
                {
                    continue;
                }
                const Vector2 onScreen = GetWorldToScreen(nodePoint(*placed.pDefinition, centre, node), mCamera);
                const float distance = Vector2Distance(onScreen, mouse);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    const Vec3d rootCentre = mDesign.stackPosition(i, node, *mHeld.part(0).pDefinition);
                    best = Snap{ i, node, Vector3Add(toRaylib(rootCentre), offset) };
                }
            }
        }
        return best;
    }

    void BuildScene::onUpdate(double frameDt, double /*alpha*/)
    {
        mOrbit.update(mCamera, (float)frameDt);
        const Ray ray = GetScreenToWorldRay(GetMousePosition(), mCamera);
        if (!mHeld.empty())
        {
            updateHeld(ray);
        }
        else
        {
            updateSelection(ray);
            updateShortcuts();
        }
    }

    void BuildScene::updateHeld(Ray ray)
    {
        mSnap = findSnap();
        mOverPlate = false;
        if (mDesign.empty())
        {
            // With nothing built yet, the held parts slide along the plate
            if (ray.direction.y < 0.0f)
            {
                const float t = -ray.position.y / ray.direction.y;
                mHeldPoint = Vector3Add(ray.position, Vector3Scale(ray.direction, t));
                mOverPlate = mouseOverViewport() && std::hypot(mHeldPoint.x, mHeldPoint.z) <= PLATE_RADIUS;
            }
        }
        else
        {
            // Otherwise they float on an upright plane through the rocket, facing the camera
            const Vector3 toCamera = Vector3Subtract(mCamera.position, mCamera.target);
            const Vector3 normal = Vector3Normalize({ toCamera.x, 0.0f, toCamera.z });
            const float facing = Vector3DotProduct(ray.direction, normal);
            if (std::abs(facing) > 1e-4f)
            {
                const float t = Vector3DotProduct(Vector3Subtract(mOrbit.focus, ray.position), normal) / facing;
                mHeldPoint = Vector3Add(ray.position, Vector3Scale(ray.direction, std::max(t, 0.0f)));
            }
        }

        // A drag from the panel leaves ImGui holding the mouse, so the release is read here wherever it happens
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
        {
            dropHeld();
        }
    }

    // Parts picked up off the rocket were already saved for undo, so letting go of them anywhere else deletes them
    void BuildScene::dropHeld()
    {
        mSelected = NO_PART;
        if (mSnap.has_value())
        {
            if (mHeldFromPanel)
            {
                pushUndo();
            }
            mSelected = mDesign.attachDesign(mSnap->parent, mSnap->node, mHeld);
        }
        else if (canStartRocket() && mOverPlate)
        {
            if (mHeldFromPanel)
            {
                pushUndo();
            }
            mDesign = mHeld;
            mSelected = 0;
        }
        mHeld.clear();
        mSnap.reset();
    }

    void BuildScene::updateSelection(Ray ray)
    {
        const bool imguiOwnsMouse = ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureMouse;
        const bool inViewport = mouseOverViewport() && !imguiOwnsMouse;
        mHovered = inViewport ? pickPart(ray) : NO_PART;

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        {
            mPressedInViewport = inViewport;
            mPressedPart = inViewport ? mHovered : NO_PART;
            mPressPosition = GetMousePosition();
        }
        const bool moved = Vector2Distance(mPressPosition, GetMousePosition()) > CLICK_SLOP;

        // Dragging a part lifts it off along with everything attached through it
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && mPressedPart != NO_PART && moved)
        {
            pushUndo();
            mHeld = mDesign.extract(mPressedPart);
            mHeldFromPanel = false;
            mPressedPart = NO_PART;
            mPressedInViewport = false;
            mHovered = NO_PART;
            mSelected = NO_PART;
            return;
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && mPressedInViewport)
        {
            mPressedInViewport = false;
            mPressedPart = NO_PART;
            if (!moved)
            {
                mSelected = mHovered;
            }
        }
    }

    void BuildScene::updateShortcuts()
    {
        if (ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureKeyboard)
        {
            return;
        }
        const bool control = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
        const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
        if (control && IsKeyPressed(KEY_Z))
        {
            if (shift)
            {
                redo();
            }
            else
            {
                undo();
            }
        }
        else if (control && IsKeyPressed(KEY_Y))
        {
            redo();
        }
        else if (IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_BACKSPACE))
        {
            deleteSelected();
        }
    }

    void BuildScene::pushUndo()
    {
        mUndo.push_back(mDesign);
        if (mUndo.size() > UNDO_LIMIT)
        {
            mUndo.erase(mUndo.begin());
        }
        mRedo.clear();
    }

    void BuildScene::undo()
    {
        if (mUndo.empty())
        {
            return;
        }
        mRedo.push_back(mDesign);
        mDesign = mUndo.back();
        mUndo.pop_back();
        mSelected = NO_PART;
    }

    void BuildScene::redo()
    {
        if (mRedo.empty())
        {
            return;
        }
        mUndo.push_back(mDesign);
        mDesign = mRedo.back();
        mRedo.pop_back();
        mSelected = NO_PART;
    }

    void BuildScene::deleteSelected()
    {
        if (mSelected == NO_PART || mSelected >= mDesign.partCount())
        {
            return;
        }
        pushUndo();
        mDesign.remove(mSelected);
        mSelected = NO_PART;
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

        if (mHeld.empty() || !mouseOverViewport())
        {
            return;
        }

        // Green markers on every free node the held parts fit, the snapped one larger
        for (PartIndex i = 0; i < mDesign.partCount(); i++)
        {
            const PlacedPart& placed = mDesign.part(i);
            const Vector3 centre = Vector3Add(toRaylib(placed.position), offset);
            for (Node node : NODES)
            {
                if (mDesign.canAttachDesign(i, node, mHeld))
                {
                    const bool snapped = mSnap.has_value() && mSnap->parent == i && mSnap->node == node;
                    DrawSphere(nodePoint(*placed.pDefinition, centre, node), snapped ? 0.16f : 0.09f, NODE_COLOR);
                }
            }
        }

        // A see-through copy shows where the held parts would land, or follows the mouse in red where they can't go
        const float rootHalfHeight = (float)mHeld.part(0).pDefinition->shape.height * 0.5f;
        bool valid = true;
        Vector3 rootPosition = {};
        if (mSnap.has_value())
        {
            rootPosition = mSnap->rootPosition;
        }
        else if (canStartRocket() && mOverPlate)
        {
            rootPosition = { 0.0f, (float)-mHeld.lowestPoint(), 0.0f };
        }
        else
        {
            valid = false;
            rootPosition = mDesign.empty() ? Vector3{ mHeldPoint.x, rootHalfHeight, mHeldPoint.z } : mHeldPoint;
        }
        for (PartIndex i = 0; i < mHeld.partCount(); i++)
        {
            const PlacedPart& placed = mHeld.part(i);
            const Color color = valid ? categoryColor(placed.pDefinition->category) : INVALID_COLOR;
            drawPart(*placed.pDefinition, Vector3Add(toRaylib(placed.position), rootPosition), (float)placed.angle,
                     ColorAlpha(color, 0.45f), ColorAlpha(ColorBrightness(color, -0.4f), 0.8f));
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
            ImGui::GetTextLineHeightWithSpacing() * 4.0f + ImGui::GetFrameHeightWithSpacing() * 3.0f + style.ItemSpacing.y * 2.0f;
        ImGui::BeginChild("PartList", { 0.0f, -footerHeight });
        drawPartList();
        ImGui::EndChild();
        drawFooter();
        ImGui::End();

        if (!mHeld.empty())
        {
            drawHeldTooltip();
        }
    }

    void BuildScene::drawHeldTooltip() const
    {
        const char* name = mHeld.part(0).pDefinition->name.c_str();
        if (!mouseOverViewport())
        {
            if (mHeldFromPanel)
            {
                ImGui::SetTooltip("Drag %s onto the rocket", name);
            }
            else
            {
                ImGui::SetTooltip("Release to delete");
            }
        }
        else if (mSnap.has_value() || (canStartRocket() && mOverPlate))
        {
            return;
        }
        else if (mDesign.empty())
        {
            ImGui::SetTooltip(canStartRocket() ? "Drop it on the build plate" : "A rocket starts from a command pod");
        }
        else if (mHeldFromPanel)
        {
            ImGui::SetTooltip("Bring it to a green marker to attach");
        }
        else
        {
            ImGui::SetTooltip("Bring it to a green marker, or release to delete");
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
                const bool held = mHeldFromPanel && !mHeld.empty() && mHeld.part(0).pDefinition == &part;
                ImGui::Selectable(part.name.c_str(), held);
                if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left) && mHeld.empty())
                {
                    mHeld.setRoot(part);
                    mHeldFromPanel = true;
                }
                else if (ImGui::IsItemHovered() && mHeld.empty())
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
            ImGui::TextDisabled(mDesign.empty() ? "Drag a pod onto the plate to start" : "Click a part to select it");
        }
        ImGui::Text("Parts: %d   Mass: %.2f t", mDesign.partCount(), mDesign.totalMass() / 1000.0);
        ImGui::TextDisabled("Right-drag orbit, wheel zoom, W/S up/down");
        ImGui::TextDisabled("Drag a part to move it, Del deletes it");

        const float third = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 2.0f) / 3.0f;
        ImGui::BeginDisabled(mUndo.empty());
        if (ImGui::Button("Undo", { third, 0.0f }))
        {
            undo();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(mRedo.empty());
        if (ImGui::Button("Redo", { third, 0.0f }))
        {
            redo();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(mSelected == NO_PART);
        if (ImGui::Button("Delete", { third, 0.0f }))
        {
            deleteSelected();
        }
        ImGui::EndDisabled();

        ImGui::BeginDisabled(mDesign.empty());
        if (ImGui::Button("Clear", { -1.0f, 0.0f }))
        {
            pushUndo();
            mDesign.clear();
            mSelected = NO_PART;
        }
        ImGui::EndDisabled();
        if (ImGui::Button("Launch", { -1.0f, 0.0f }))
        {
            requestScene(SceneId::Flight);
        }
    }
}
