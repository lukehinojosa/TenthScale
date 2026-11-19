#include "flight/BuildScene.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <imgui.h>
#include <raymath.h>
#include <rlgl.h>

#include "flight/DesignFile.h"
#include "flight/PartDrawing.h"
#include "flight/StageSymbol.h"

namespace tenth
{
    namespace
    {
        constexpr float PANEL_WIDTH = 280.0f;
        constexpr float RIGHT_PANEL_WIDTH = 270.0f;
        constexpr float ICON_SIZE = 40.0f;
        constexpr const char* STAGE_PAYLOAD = "STAGE_PARTS";
        constexpr double ORBIT_DELTA_V = 3000.0; // m/s a rocket needs to reach low orbit, roughly, losses included
        constexpr float BUILD_HEIGHT = 20.0f; // the pod floats this high above the hall floor
        constexpr float FLOOR_CLEARANCE = 1.0f; // a long rocket is raised so its bottom stays this far off the floor
        constexpr float HALL_HALF_WIDTH = 25.0f;
        constexpr float HALL_HEIGHT = 48.0f;
        constexpr float EMPTY_FOCUS_RANGE = 2.0f; // how far the camera can move up or down from where the first part goes
        constexpr float CLICK_SLOP = 5.0f; // pixels the mouse may move between press and release and still count as a click
        constexpr float SNAP_RADIUS = 60.0f; // pixels from a node that the held parts snap to it
        constexpr size_t UNDO_LIMIT = 100;
        constexpr double ANGLE_STEP = 15.0 * DEG2RAD; // side-mounted parts snap round the rocket in these steps
        constexpr int MAX_SYMMETRY = 4;
        constexpr Color FLOOR_COLOR = { 58, 60, 66, 255 };
        constexpr Color WALL_COLOR = { 82, 88, 98, 255 };
        constexpr Color SIDE_WALL_COLOR = { 72, 78, 88, 255 };
        constexpr Color CEILING_COLOR = { 46, 48, 54, 255 };
        constexpr Color BEAM_COLOR = { 54, 58, 66, 255 };
        constexpr Color DOOR_COLOR = { 104, 110, 120, 255 };
        constexpr Color HAZARD_COLOR = { 230, 180, 40, 255 };
        constexpr Color SELECTED_OUTLINE = { 255, 230, 60, 255 };
        constexpr Color INVALID_COLOR = { 220, 60, 50, 255 };
        constexpr Color NODE_COLOR = { 80, 230, 120, 255 };
        constexpr Color MASS_CENTER_COLOR = { 255, 210, 40, 255 };
        constexpr Color PRESSURE_CENTER_COLOR = { 70, 150, 255, 255 };
        constexpr ImVec4 WARNING_TEXT = { 1.0f, 0.55f, 0.35f, 1.0f };
        constexpr ImU32 ICON_SYMBOL = IM_COL32(240, 242, 246, 255);
        constexpr ImU32 ICON_HIGHLIGHT = IM_COL32(255, 230, 60, 255);

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
            if (part.hasParachute)
            {
                ImGui::TextUnformatted("Built-in parachute");
            }
            if (part.popsOffAtPeak)
            {
                ImGui::TextUnformatted("Pops off at peak altitude");
            }
            if (part.isEngine())
            {
                ImGui::Text("Thrust: %.0f kN in vacuum", part.thrustVacuum / 1000.0);
                ImGui::Text("Efficiency: %.0f s at sea level, %.0f s in vacuum", part.ispSeaLevel, part.ispVacuum);
            }
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }

        // The inside of the assembly building: floor, walls with beams and floor ledges, ceiling, and the big door.
        // A wall the camera has backed out through is left out, along with what hangs on it, so the view stays open.
        void drawHall(Vector3 camera)
        {
            const float w = HALL_HALF_WIDTH * 2.0f;
            const float h = HALL_HEIGHT;
            DrawPlane({ 0.0f, 0.0f, 0.0f }, { w, w }, FLOOR_COLOR);
            rlPushMatrix();
            rlTranslatef(0.0f, 0.01f, 0.0f);
            DrawGrid(25, 2.0f);
            rlPopMatrix();
            if (camera.y < h)
            {
                DrawCube({ 0.0f, h + 0.5f, 0.0f }, w, 1.0f, w, CEILING_COLOR);
            }

            // Each wall faces in along x or z from one side; outward is which way is out of the room
            struct Wall
            {
                Vector3 outward;
                Color color;
            };
            const Wall walls[] = { { { 0.0f, 0.0f, -1.0f }, WALL_COLOR },
                                   { { 0.0f, 0.0f, 1.0f }, WALL_COLOR },
                                   { { -1.0f, 0.0f, 0.0f }, SIDE_WALL_COLOR },
                                   { { 1.0f, 0.0f, 0.0f }, SIDE_WALL_COLOR } };
            for (const Wall& wall : walls)
            {
                const float out = camera.x * wall.outward.x + camera.z * wall.outward.z;
                if (out > HALL_HALF_WIDTH)
                {
                    continue;
                }

                // Places a box against this wall: inset is how far into the room its centre sits, along runs the length
                const bool alongX = wall.outward.x == 0.0f;
                auto onWall = [&](float inset, float along, float height, Vector3 size, Color color)
                {
                    const float edge = HALL_HALF_WIDTH - inset;
                    const Vector3 centre = alongX ? Vector3{ along, height, wall.outward.z * edge } : Vector3{ wall.outward.x * edge, height, along };
                    const Vector3 turned = alongX ? size : Vector3{ size.z, size.y, size.x };
                    DrawCube(centre, turned.x, turned.y, turned.z, color);
                };
                onWall(-0.5f, 0.0f, h * 0.5f, { w, h, 1.0f }, wall.color);

                // Beams up the wall and ledges at each work level give the room a sense of size
                for (float along = -HALL_HALF_WIDTH; along <= HALL_HALF_WIDTH; along += 10.0f)
                {
                    onWall(0.4f, along, h * 0.5f, { 0.8f, h, 0.8f }, BEAM_COLOR);
                }
                for (float level = 12.0f; level < h; level += 12.0f)
                {
                    onWall(1.0f, 0.0f, level, { w, 0.4f, 2.0f }, BEAM_COLOR);
                }

                // The rollout door on the far wall, edged in hazard stripes
                if (wall.outward.z < 0.0f)
                {
                    const float doorWidth = 18.0f;
                    const float doorHeight = 42.0f;
                    onWall(0.05f, 0.0f, doorHeight * 0.5f, { doorWidth, doorHeight, 0.1f }, DOOR_COLOR);
                    onWall(0.1f, -doorWidth * 0.5f, doorHeight * 0.5f, { 0.5f, doorHeight, 0.1f }, HAZARD_COLOR);
                    onWall(0.1f, doorWidth * 0.5f, doorHeight * 0.5f, { 0.5f, doorHeight, 0.1f }, HAZARD_COLOR);
                    onWall(0.1f, 0.0f, doorHeight, { doorWidth + 0.5f, 0.5f, 0.1f }, HAZARD_COLOR);
                }
            }
        }

        // Where a stack node sits in the world, given the part's centre
        Vector3 nodePoint(const PartShape& shape, Vector3 centre, Node node)
        {
            const float halfHeight = (float)shape.height * 0.5f;
            return { centre.x, centre.y + (node == Node::Top ? halfHeight : -halfHeight), centre.z };
        }
    }

    BuildScene::BuildScene(Camera3D& camera, const PartCatalog& catalog, RocketDesign& design, const SolarSystem& solarSystem)
        : mCamera(camera), mCatalog(catalog), mDesign(design)
    {
        mOrbit.leftInset = PANEL_WIDTH;
        mOrbit.rightInset = RIGHT_PANEL_WIDTH;
        mOrbit.focus = { 0.0f, BUILD_HEIGHT, 0.0f };
        mOrbit.distance = 16.0f;
        mOrbit.maxDistance = HALL_HALF_WIDTH * 2.0f - 4.0f; // backing out past a wall cuts it away
        mOrbit.minPitch = -1.2f;
        mOrbit.minCameraHeight = 0.5f;
        mOrbit.maxCameraHeight = HALL_HEIGHT - 1.0f;
        const BodyId earth = solarSystem.findBody("Earth");
        if (earth != NO_BODY)
        {
            mSurfaceGravity = solarSystem.surfaceGravity(earth);
        }
    }

    void BuildScene::onEnter()
    {
        mOrbit.apply(mCamera);
    }

    bool BuildScene::mouseOverViewport() const
    {
        const float x = GetMousePosition().x;
        return x > PANEL_WIDTH && x < (float)GetScreenWidth() - RIGHT_PANEL_WIDTH;
    }

    bool BuildScene::canStartRocket() const
    {
        return mDesign.empty() && !mHeld.empty() && mHeld.part(0).pDefinition->category == PartCategory::Pod;
    }

    // The pod stays put at the build height while parts go on above and below it
    Vector3 BuildScene::designOffset() const
    {
        return { 0.0f, std::max(BUILD_HEIGHT, FLOOR_CLEARANCE - (float)mDesign.lowestPoint()), 0.0f };
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
            const RayCollision hit = GetRayCollisionBox(ray, partBounds(placed.shape, position, (float)placed.angle));
            if (hit.hit && (closest == NO_PART || hit.distance < closestDistance))
            {
                closest = i;
                closestDistance = hit.distance;
            }
        }
        return closest;
    }

    std::optional<BuildScene::Snap> BuildScene::findSnap(Ray ray) const
    {
        if (mHeld.empty() || mDesign.empty() || !mouseOverViewport())
        {
            return std::nullopt;
        }
        return mHeld.part(0).pDefinition->mountsRadially ? findRadialSnap(ray) : findStackSnap();
    }

    // The free node nearest the mouse on screen that the held parts fit, if one is close enough
    std::optional<BuildScene::Snap> BuildScene::findStackSnap() const
    {
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
                const Vector2 onScreen = GetWorldToScreen(nodePoint(placed.shape, centre, node), mCamera);
                const float distance = Vector2Distance(onScreen, mouse);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    Snap snap;
                    snap.parent = i;
                    snap.node = node;
                    best = snap;
                }
            }
        }
        return best;
    }

    // The nearest part under the mouse whose side takes the held parts, and the angle and height of that spot
    std::optional<BuildScene::Snap> BuildScene::findRadialSnap(Ray ray) const
    {
        const Vector3 offset = designOffset();
        std::optional<Snap> best;
        float bestDistance = 0.0f;
        for (PartIndex i = 0; i < mDesign.partCount(); i++)
        {
            if (!mDesign.canAttachDesignRadial(i, mHeld))
            {
                continue;
            }
            const PlacedPart& placed = mDesign.part(i);
            const Vector3 centre = Vector3Add(toRaylib(placed.position), offset);
            const RayCollision hit = GetRayCollisionBox(ray, partBounds(placed.shape, centre, (float)placed.angle));
            if (!hit.hit || (best.has_value() && hit.distance >= bestDistance))
            {
                continue;
            }
            bestDistance = hit.distance;
            const double angle = std::atan2((double)hit.point.x - centre.x, (double)hit.point.z - centre.z);
            Snap snap;
            snap.parent = i;
            snap.radial = true;
            snap.angle = std::round(angle / ANGLE_STEP) * ANGLE_STEP;
            snap.height = (double)hit.point.y - centre.y;
            best = snap;
        }
        return best;
    }

    std::vector<PartIndex> BuildScene::applySnap(RocketDesign& design, const Snap& snap) const
    {
        if (snap.radial)
        {
            return design.attachDesignRadial(snap.parent, snap.angle, snap.height, mHeld, mSymmetry);
        }
        const PartIndex root = design.attachDesign(snap.parent, snap.node, mHeld);
        return root == NO_PART ? std::vector<PartIndex>{} : std::vector<PartIndex>{ root };
    }

    bool BuildScene::sameSymmetry(PartIndex a, PartIndex b) const
    {
        if (a == NO_PART || b == NO_PART || a >= mDesign.partCount() || b >= mDesign.partCount())
        {
            return false;
        }
        const int group = mDesign.part(a).symmetryGroup;
        return a == b || (group != 0 && group == mDesign.part(b).symmetryGroup);
    }

    // The camera's focus stays between the rocket's lowest and highest points, so some of it is always in view
    void BuildScene::limitFocus()
    {
        const float rootHeight = designOffset().y;
        if (mDesign.empty())
        {
            mOrbit.minFocusHeight = rootHeight - EMPTY_FOCUS_RANGE;
            mOrbit.maxFocusHeight = rootHeight + EMPTY_FOCUS_RANGE;
        }
        else
        {
            mOrbit.minFocusHeight = rootHeight + (float)mDesign.lowestPoint();
            mOrbit.maxFocusHeight = rootHeight + (float)mDesign.highestPoint();
        }
    }

    void BuildScene::onUpdate(double frameDt, double /*alpha*/)
    {
        mDesign.updateStaging();
        std::erase_if(mHighlighted, [this](PartIndex index) { return index >= mDesign.partCount(); });
        limitFocus();
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
        if (IsKeyPressed(KEY_X))
        {
            mSymmetry = mSymmetry % MAX_SYMMETRY + 1;
        }
        mSnap = findSnap(ray);

        // Unsnapped parts float on an upright plane through the rocket, facing the camera
        const Vector3 toCamera = Vector3Subtract(mCamera.position, mCamera.target);
        const Vector3 normal = Vector3Normalize({ toCamera.x, 0.0f, toCamera.z });
        const float facing = Vector3DotProduct(ray.direction, normal);
        if (std::abs(facing) > 1e-4f)
        {
            const float t = Vector3DotProduct(Vector3Subtract(mOrbit.focus, ray.position), normal) / facing;
            mHeldPoint = Vector3Add(ray.position, Vector3Scale(ray.direction, std::max(t, 0.0f)));
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
        mHighlighted.clear();
        if (mSnap.has_value())
        {
            if (mHeldFromPanel)
            {
                pushUndo();
            }
            const std::vector<PartIndex> roots = applySnap(mDesign, *mSnap);
            mSelected = roots.empty() ? NO_PART : roots.front();
        }
        else if (canStartRocket() && mouseOverViewport())
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
            mHeld = mDesign.extractWithSymmetry(mPressedPart);
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
                mHighlighted.clear();
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
        else if (IsKeyPressed(KEY_X))
        {
            mSymmetry = mSymmetry % MAX_SYMMETRY + 1;
        }
    }

    bool BuildScene::isHighlighted(PartIndex index) const
    {
        return std::find(mHighlighted.begin(), mHighlighted.end(), index) != mHighlighted.end();
    }

    void BuildScene::pushUndo()
    {
        mHighlighted.clear();
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
        mHighlighted.clear();
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
        mHighlighted.clear();
    }

    void BuildScene::deleteSelected()
    {
        if (mSelected == NO_PART || mSelected >= mDesign.partCount())
        {
            return;
        }
        pushUndo();
        mDesign.removeWithSymmetry(mSelected);
        mSelected = NO_PART;
    }

    void BuildScene::onDraw3D()
    {
        drawHall(mCamera.position);

        const Vector3 offset = designOffset();
        for (PartIndex i = 0; i < mDesign.partCount(); i++)
        {
            const PlacedPart& placed = mDesign.part(i);
            Color color = categoryColor(placed.pDefinition->category);
            const bool highlighted = isHighlighted(i);
            if (sameSymmetry(i, mHovered) || highlighted)
            {
                color = ColorBrightness(color, 0.25f);
            }
            const Color outline = sameSymmetry(i, mSelected) || highlighted ? SELECTED_OUTLINE : ColorBrightness(color, -0.5f);
            drawPart(drawnShape(mDesign, i), Vector3Add(toRaylib(placed.position), offset), (float)placed.angle, color, outline);
        }

        if (mShowMarkers && !mDesign.empty())
        {
            drawMarkers(offset);
        }

        if (mHeld.empty() || !mouseOverViewport())
        {
            return;
        }

        // Green markers on every free node the held parts fit, the snapped one larger
        const bool heldIsRadial = mHeld.part(0).pDefinition->mountsRadially;
        for (PartIndex i = 0; i < mDesign.partCount() && !heldIsRadial; i++)
        {
            const PlacedPart& placed = mDesign.part(i);
            const Vector3 centre = Vector3Add(toRaylib(placed.position), offset);
            for (Node node : NODES)
            {
                if (mDesign.canAttachDesign(i, node, mHeld))
                {
                    const bool snapped = mSnap.has_value() && mSnap->parent == i && mSnap->node == node;
                    DrawSphere(nodePoint(placed.shape, centre, node), snapped ? 0.16f : 0.09f, NODE_COLOR);
                }
            }
        }

        // A see-through copy shows where the held parts would land, every symmetry copy included
        auto drawGhost = [](const PartShape& shape, const PlacedPart& placed, Vector3 position, bool valid)
        {
            const Color color = valid ? categoryColor(placed.pDefinition->category) : INVALID_COLOR;
            drawPart(shape, position, (float)placed.angle, ColorAlpha(color, 0.45f), ColorAlpha(ColorBrightness(color, -0.4f), 0.8f));
        };
        if (mSnap.has_value())
        {
            // Attaching to a copy of the rocket places the parts exactly as a drop would
            RocketDesign preview = mDesign;
            applySnap(preview, *mSnap);
            for (PartIndex i = mDesign.partCount(); i < preview.partCount(); i++)
            {
                drawGhost(drawnShape(preview, i), preview.part(i), Vector3Add(toRaylib(preview.part(i).position), offset), true);
            }
            return;
        }

        // Otherwise a pod for an empty hall shows where it will float, and anything else follows the mouse in red
        const bool valid = canStartRocket();
        const Vector3 rootPosition = valid ? offset : mHeldPoint;
        for (PartIndex i = 0; i < mHeld.partCount(); i++)
        {
            const Vector3 position = Vector3Add(toRaylib(mHeld.part(i).position - mHeld.part(0).position), rootPosition);
            drawGhost(drawnShape(mHeld, i), mHeld.part(i), position, valid);
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
            ImGui::GetTextLineHeightWithSpacing() * 4.0f + ImGui::GetFrameHeightWithSpacing() * 5.0f + style.ItemSpacing.y * 2.0f;
        ImGui::BeginChild("PartList", { 0.0f, -footerHeight });
        drawPartList();
        ImGui::EndChild();
        drawFooter();
        ImGui::End();
        drawRocketPanel();

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
        else if (mSnap.has_value() || canStartRocket())
        {
            return;
        }
        else if (mDesign.empty())
        {
            ImGui::SetTooltip("A rocket starts from a command pod");
        }
        else
        {
            const bool radial = mHeld.part(0).pDefinition->mountsRadially;
            const char* target = radial ? "against the side of a part" : "to a green marker";
            if (mHeldFromPanel)
            {
                ImGui::SetTooltip("Bring it %s to attach", target);
            }
            else
            {
                ImGui::SetTooltip("Bring it %s, or release to delete", target);
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
            ImGui::TextDisabled(mDesign.empty() ? "Drag a pod into the hall to start" : "Click a part to select it");
        }
        ImGui::Text("Parts: %d   Mass: %.2f t", mDesign.partCount(), mDesign.totalMass() / 1000.0);
        ImGui::TextDisabled("Right-drag orbit, W/S lift, wheel zoom");
        ImGui::TextDisabled("Drag to move, Del deletes, X symmetry");

        // Side-mounted parts are placed this many times, evenly around the rocket
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Symmetry");
        for (int copies = 1; copies <= MAX_SYMMETRY; copies++)
        {
            ImGui::SameLine();
            char label[8];
            std::snprintf(label, sizeof(label), "%dx", copies);
            ImGui::RadioButton(label, &mSymmetry, copies);
        }

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
        if (ImGui::Button("Clear", { third, 0.0f }))
        {
            pushUndo();
            mDesign.clear();
            mSelected = NO_PART;
        }
        ImGui::SameLine();
        if (ImGui::Button("Save", { third, 0.0f }))
        {
            mFileMessage.clear();
            ImGui::OpenPopup("Save rocket");
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Load", { third, 0.0f }))
        {
            mFileMessage.clear();
            ImGui::OpenPopup("Load rocket");
        }
        drawSavePopup();
        drawLoadPopup();
        ImGui::BeginDisabled(mDesign.empty() || mDesign.part(0).pDefinition->category != PartCategory::Pod);
        if (ImGui::Button("Roll out to the launch pad", { -1.0f, 0.0f }))
        {
            requestScene(SceneId::Flight);
        }
        ImGui::EndDisabled();
        if (ImGui::Button("Rotating planets", { -1.0f, 0.0f }))
        {
            requestScene(SceneId::Planets);
        }
    }

    // Drawn over the rocket so they show through the parts
    void BuildScene::drawMarkers(Vector3 offset) const
    {
        const Vector3 massCenter = Vector3Add(toRaylib(mDesign.centerOfMass()), offset);
        const Vector3 pressureCenter = Vector3Add(toRaylib(centerOfPressure(mDesign)), offset);
        rlDrawRenderBatchActive();
        rlDisableDepthTest();
        DrawSphere(massCenter, 0.2f, MASS_CENTER_COLOR);
        DrawSphereWires(massCenter, 0.2f, 6, 8, ColorBrightness(MASS_CENTER_COLOR, -0.5f));
        DrawSphere(pressureCenter, 0.15f, PRESSURE_CENTER_COLOR);
        DrawSphereWires(pressureCenter, 0.15f, 6, 8, ColorBrightness(PRESSURE_CENTER_COLOR, -0.5f));
        rlDrawRenderBatchActive();
        rlEnableDepthTest();
    }

    void BuildScene::drawRocketPanel()
    {
        const ImGuiViewport* pViewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos({ pViewport->WorkPos.x + pViewport->WorkSize.x - RIGHT_PANEL_WIDTH, pViewport->WorkPos.y });
        ImGui::SetNextWindowSize({ RIGHT_PANEL_WIDTH, pViewport->WorkSize.y });
        const ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
        ImGui::Begin("Rocket", nullptr, flags);
        if (mDesign.empty())
        {
            ImGui::TextDisabled("Nothing built yet");
            ImGui::End();
            return;
        }

        // Worked out here, not once per frame, since buttons earlier in this frame's GUI may have changed the rocket
        const DesignStats stats = computeStats(mDesign, mSurfaceGravity);
        ImGui::Text("Mass: %.2f t full, %.2f t empty", mDesign.totalMass() / 1000.0, mDesign.dryMass() / 1000.0);
        ImGui::Text("Height: %.1f m", mDesign.highestPoint() - mDesign.lowestPoint());

        drawStaging(stats);

        ImGui::SeparatorText("Balance");
        ImGui::Checkbox("Show markers", &mShowMarkers);
        const float swatchSize = ImGui::GetTextLineHeight();
        const ImGuiColorEditFlags swatchFlags = ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop;
        ImGui::ColorButton("##mass", toImGui(MASS_CENTER_COLOR), swatchFlags, { swatchSize, swatchSize });
        ImGui::SameLine();
        ImGui::TextUnformatted("Center of mass");
        ImGui::ColorButton("##pressure", toImGui(PRESSURE_CENTER_COLOR), swatchFlags, { swatchSize, swatchSize });
        ImGui::SameLine();
        ImGui::TextUnformatted("Center of pressure");
        if (stats.stable)
        {
            ImGui::TextWrapped("Stable: the pressure center is below the mass center, so the air keeps the nose forward.");
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_Text, WARNING_TEXT);
            ImGui::TextWrapped("Unstable: the pressure center is above the mass center. Add fins low on the rocket.");
            ImGui::PopStyleColor();
        }
        ImGui::End();
    }

    void BuildScene::drawStaging(const DesignStats& stats)
    {
        ImGui::SeparatorText("Staging");
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextWrapped("Fires from the bottom up. Drag icons between stages; double-click a stack to spread it out.");
        ImGui::PopStyleColor();

        const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        ImGui::BeginDisabled(!mDesign.customStaging());
        if (ImGui::Button("Auto-stage", { half, 0.0f }))
        {
            pushUndo();
            mDesign.autoStage();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("New stage", { half, 0.0f }))
        {
            mPendingInsert = mDesign.stageCount();
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Adds an empty stage that fires last. Drop icons here to move them into a new one.");
        }

        // Dropping icons on the button puts them in a new last stage
        if (ImGui::BeginDragDropTarget())
        {
            if (ImGui::AcceptDragDropPayload(STAGE_PAYLOAD) != nullptr)
            {
                mPendingInsert = mDesign.stageCount();
                mPendingMove = mDraggedIcon;
                mPendingMoveStage = mDesign.stageCount();
            }
            ImGui::EndDragDropTarget();
        }

        for (int stage = mDesign.stageCount() - 1; stage >= 0; stage--)
        {
            const StageStats empty;
            drawStage(stage, stage < (int)stats.stages.size() ? stats.stages[stage] : empty);
        }

        // Applied once the stages are drawn, so none of them shifted part way through
        if (mPendingInsert != NO_STAGE || mPendingRemove != NO_STAGE || mPendingMoveStage != NO_STAGE)
        {
            pushUndo();
            mExpanded.clear();
        }
        if (mPendingInsert != NO_STAGE)
        {
            mDesign.insertStage(mPendingInsert);
        }
        if (mPendingRemove != NO_STAGE)
        {
            mDesign.removeStage(mPendingRemove);
        }
        if (mPendingMoveStage != NO_STAGE)
        {
            mDesign.moveToStage(mPendingMove, mPendingMoveStage);
            mHighlighted = mPendingMove;
        }
        mPendingInsert = NO_STAGE;
        mPendingRemove = NO_STAGE;
        mPendingMoveStage = NO_STAGE;
        mPendingMove.clear();

        ImGui::Separator();
        ImGui::Text("Total delta-v: %.0f m/s", stats.totalDeltaVVacuum);
        if (!stats.hasEngines)
        {
            ImGui::TextColored(WARNING_TEXT, "No engines yet");
        }
        else if (stats.liftoffTwr < 1.0)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, WARNING_TEXT);
            ImGui::TextWrapped("Liftoff TWR %.2f: too heavy to lift off", stats.liftoffTwr);
            ImGui::PopStyleColor();
        }
        if (stats.totalDeltaVVacuum < ORBIT_DELTA_V)
        {
            ImGui::TextDisabled("Orbit takes roughly %.0f m/s", ORBIT_DELTA_V);
        }
    }

    void BuildScene::drawStage(int stage, const StageStats& stats)
    {
        ImGui::PushID(stage);
        const std::vector<PartIndex> parts = mDesign.partsInStage(stage);
        ImGui::BeginChild("Stage", { 0.0f, 0.0f }, ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);

        ImGui::AlignTextToFramePadding();
        ImGui::Text("Stage %d", stage + 1);
        const float buttonWidth = ImGui::GetFrameHeight();
        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - buttonWidth * 2.0f - ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::Button("+", { buttonWidth, 0.0f }))
        {
            mPendingInsert = stage + 1;
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Add an empty stage above this one");
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!parts.empty());
        if (ImGui::Button("x", { buttonWidth, 0.0f }))
        {
            mPendingRemove = stage;
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        {
            ImGui::SetTooltip(parts.empty() ? "Remove this stage" : "Only an empty stage can be removed");
        }

        // Parts of one kind stack onto a single icon, unless that stack has been spread out
        std::vector<std::pair<const PartDefinition*, std::vector<PartIndex>>> groups;
        for (PartIndex index : parts)
        {
            const PartDefinition* pDefinition = mDesign.part(index).pDefinition;
            auto it = std::find_if(groups.begin(), groups.end(), [pDefinition](const auto& group) { return group.first == pDefinition; });
            if (it == groups.end())
            {
                groups.push_back({ pDefinition, { index } });
            }
            else
            {
                it->second.push_back(index);
            }
        }
        const float rowRight = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
        bool first = true;
        auto place = [&first, rowRight]()
        {
            if (!first && ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + ICON_SIZE <= rowRight)
            {
                ImGui::SameLine();
            }
            first = false;
        };
        for (const auto& [pDefinition, members] : groups)
        {
            const bool expanded = std::find(mExpanded.begin(), mExpanded.end(), std::make_pair(stage, pDefinition)) != mExpanded.end();
            if (expanded || members.size() == 1)
            {
                for (PartIndex index : members)
                {
                    place();
                    ImGui::PushID(index);
                    drawStageIcon(*pDefinition, { index }, stage, false);
                    ImGui::PopID();
                }
            }
            else
            {
                place();
                ImGui::PushID(pDefinition->id.c_str());
                drawStageIcon(*pDefinition, members, stage, true);
                ImGui::PopID();
            }
        }

        if (parts.empty())
        {
            ImGui::TextDisabled("Empty");
        }
        else if (!stats.engines.empty())
        {
            ImGui::Text("Delta-v %.0f m/s   TWR %.2f", stats.deltaVVacuum, stats.twrSeaLevel);
        }
        else if (stats.decoupler != NO_PART)
        {
            ImGui::TextDisabled("Drops the stage below");
        }
        ImGui::EndChild();

        if (ImGui::BeginDragDropTarget())
        {
            if (ImGui::AcceptDragDropPayload(STAGE_PAYLOAD) != nullptr)
            {
                mPendingMove = mDraggedIcon;
                mPendingMoveStage = stage;
            }
            ImGui::EndDragDropTarget();
        }
        ImGui::PopID();
    }

    void BuildScene::drawStageIcon(const PartDefinition& definition, const std::vector<PartIndex>& parts, int stage, bool grouped)
    {
        ImGui::InvisibleButton("icon", { ICON_SIZE, ICON_SIZE });
        const bool hovered = ImGui::IsItemHovered();
        if (ImGui::IsItemClicked(ImGuiMouseButton_Left))
        {
            mHighlighted = parts;
            mSelected = NO_PART;
        }

        // Double-clicking a stack spreads it out; double-clicking a spread-out icon stacks its kind again
        if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
        {
            const std::pair<int, const PartDefinition*> key = { stage, &definition };
            const auto it = std::find(mExpanded.begin(), mExpanded.end(), key);
            if (it != mExpanded.end())
            {
                mExpanded.erase(it);
            }
            else if (grouped)
            {
                mExpanded.push_back(key);
            }
        }
        if (ImGui::BeginDragDropSource())
        {
            mDraggedIcon = parts;
            const int count = (int)parts.size();
            ImGui::SetDragDropPayload(STAGE_PAYLOAD, &count, sizeof(count));
            ImGui::Text("Move %dx %s", count, definition.name.c_str());
            ImGui::EndDragDropSource();
        }

        bool lit = false;
        for (PartIndex index : parts)
        {
            lit = lit || isHighlighted(index) || sameSymmetry(index, mSelected);
        }
        ImDrawList* pList = ImGui::GetWindowDrawList();
        const ImVec2 min = ImGui::GetItemRectMin();
        const ImVec2 max = ImGui::GetItemRectMax();
        const Color background = ColorBrightness(categoryColor(definition.category), hovered ? -0.15f : -0.35f);
        pList->AddRectFilled(min, max, IM_COL32(background.r, background.g, background.b, 255), 6.0f);
        if (lit)
        {
            pList->AddRect(min, max, ICON_HIGHLIGHT, 6.0f, 0, 2.5f);
        }
        drawStageSymbol(pList, min, ICON_SIZE, definition, ICON_SYMBOL);
        if (parts.size() > 1)
        {
            char count[8];
            std::snprintf(count, sizeof(count), "%d", (int)parts.size());
            const ImVec2 textSize = ImGui::CalcTextSize(count);
            const ImVec2 corner = { max.x - textSize.x - 3.0f, max.y - textSize.y - 1.0f };
            pList->AddRectFilled({ corner.x - 2.0f, corner.y }, max, IM_COL32(20, 22, 26, 220), 4.0f);
            pList->AddText(corner, ICON_SYMBOL, count);
        }

        if (hovered && !ImGui::IsMouseDragging(ImGuiMouseButton_Left))
        {
            ImGui::BeginTooltip();
            if (parts.size() > 1)
            {
                ImGui::Text("%dx %s", (int)parts.size(), definition.name.c_str());
            }
            else
            {
                ImGui::TextUnformatted(definition.name.c_str());
            }
            ImGui::TextDisabled("Click to light it up on the rocket, drag to another stage");
            if (grouped)
            {
                ImGui::TextDisabled("Double-click to spread out");
            }
            ImGui::EndTooltip();
        }
    }

    void BuildScene::drawSavePopup()
    {
        if (!ImGui::BeginPopup("Save rocket"))
        {
            return;
        }
        ImGui::TextUnformatted("Name");
        ImGui::SetNextItemWidth(220.0f);
        const bool entered = ImGui::InputText("##name", mRocketName, sizeof(mRocketName), ImGuiInputTextFlags_EnterReturnsTrue);
        const std::string name = cleanRocketName(mRocketName);
        const std::vector<std::string> saved = savedRocketNames();
        if (std::find(saved.begin(), saved.end(), name) != saved.end())
        {
            ImGui::TextDisabled("Replaces the saved %s", name.c_str());
        }
        ImGui::BeginDisabled(name.empty());
        if (ImGui::Button("Save", { 220.0f, 0.0f }) || (entered && !name.empty()))
        {
            if (saveRocket(name, mDesign))
            {
                ImGui::CloseCurrentPopup();
            }
            else
            {
                mFileMessage = "Couldn't save it";
            }
        }
        ImGui::EndDisabled();
        if (!mFileMessage.empty())
        {
            ImGui::TextColored(WARNING_TEXT, "%s", mFileMessage.c_str());
        }
        ImGui::EndPopup();
    }

    void BuildScene::drawLoadPopup()
    {
        if (!ImGui::BeginPopup("Load rocket"))
        {
            return;
        }
        std::optional<RocketDesign> loaded;
        std::string loadedName;
        if (ImGui::Selectable("Default rocket"))
        {
            loaded = defaultRocket(mCatalog);
            loadedName = "My rocket";
        }
        ImGui::SeparatorText("Saved");
        const std::vector<std::string> saved = savedRocketNames();
        if (saved.empty())
        {
            ImGui::TextDisabled("Nothing saved yet");
        }
        for (const std::string& name : saved)
        {
            if (ImGui::Selectable(name.c_str()))
            {
                loaded = loadRocket(name, mCatalog);
                loadedName = name;
                if (!loaded.has_value())
                {
                    mFileMessage = "Couldn't read " + name;
                }
            }
        }
        if (loaded.has_value())
        {
            pushUndo();
            mDesign = *loaded;
            mSelected = NO_PART;
            mExpanded.clear();
            std::snprintf(mRocketName, sizeof(mRocketName), "%s", loadedName.c_str());
            ImGui::CloseCurrentPopup();
        }
        if (!mFileMessage.empty())
        {
            ImGui::TextColored(WARNING_TEXT, "%s", mFileMessage.c_str());
        }
        ImGui::EndPopup();
    }
}
