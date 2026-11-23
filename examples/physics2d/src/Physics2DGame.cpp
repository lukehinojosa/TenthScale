#include "Physics2DGame.h"

#include <algorithm>
#include <utility>

#include <imgui.h>

#include <engine/Conversions.h>

using cyclone2d::real;
using engine::toCyclone;
namespace c2 = cyclone2d;

namespace
{
    // The part of the world kept in view: walls at either side, the ground at the bottom
    constexpr float kViewHalfWidth = 11.0f;
    constexpr float kViewHeight = 15.0f;
    constexpr float kWallX = 10.0f;

    Color randomColor()
    {
        return ColorFromHSV((float)GetRandomValue(0, 360), 0.55f, 0.9f);
    }
}

Physics2DGame::Physics2DGame(engine::GameConfig config, Physics2DSettings physics) : engine::Game(std::move(config))
{
    mWorld.friction = (real)physics.friction;
    mWorld.restitution = (real)physics.restitution;
}

void Physics2DGame::onInit()
{
    resetScene();
}

void Physics2DGame::resetScene()
{
    // Clear the world's pointers before what they point at is destroyed
    stopDrag();
    mWorld.bodies.clear();
    mWorld.colliders.clear();
    mWorld.contactGenerators.clear();
    mWorld.joints.clear();
    mWorld.forces.clear();
    mShapes.clear();
    mJoints.clear();
    mScenery.clear();

    mScenery.push_back(std::make_unique<c2::PlaneCollider>(c2::Vector2(0, 1), (real)0));
    mScenery.push_back(std::make_unique<c2::PlaneCollider>(c2::Vector2(1, 0), (real)-kWallX));
    mScenery.push_back(std::make_unique<c2::PlaneCollider>(c2::Vector2(-1, 0), (real)-kWallX));
    for (const auto& pPlane : mScenery)
    {
        mWorld.colliders.push_back(pPlane.get());
    }
    addStaticBox({ -6.0f, 4.0f }, { 3.0f, 0.15f }, -0.35f);

    // A pyramid of boxes, and a tower beside it
    for (int row = 0; row < 5; row++)
    {
        for (int i = 0; i < 5 - row; i++)
        {
            const real x = (real)(1.0 + i * 1.02 + row * 0.51);
            addBox({ x, (real)(0.5 + row * 1.0) }, { 0.5f, 0.5f }, row % 2 == 0 ? SKYBLUE : DARKBLUE);
        }
    }
    for (int i = 0; i < 6; i++)
    {
        addBox({ -2.0f, (real)(0.3 + i * 0.6) }, { 0.3f, 0.3f }, i % 2 == 0 ? ORANGE : GOLD);
    }

    // Rolling down the ramp onto the tower
    addCircle({ -7.5f, 6.0f }, 0.45f, LIME);
    addPolygon({ -5.0f, 6.5f }, 6, 0.5f, PURPLE);

    addChain({ 6.0f, 12.0f }, 4);
}

Shape2D& Physics2DGame::addShape(std::unique_ptr<c2::Collider> pCollider, c2::Vector2 position, real angle, real mass, Color color)
{
    auto pShape = std::make_unique<Shape2D>();
    pShape->body.setPosition(position);
    pShape->body.setOrientation(angle);
    pShape->body.setMass(mass);
    pShape->pCollider = std::move(pCollider);
    pShape->pCollider->pBody = &pShape->body;
    pShape->previousPosition = position;
    pShape->previousAngle = angle;
    pShape->color = color;

    mWorld.bodies.push_back(&pShape->body);
    mWorld.colliders.push_back(pShape->pCollider.get());
    mShapes.push_back(std::move(pShape));
    return *mShapes.back();
}

Shape2D& Physics2DGame::addBox(c2::Vector2 position, c2::Vector2 halfSize, Color color, real angle)
{
    const real mass = 4 * halfSize.x * halfSize.y;
    Shape2D& shape = addShape(std::make_unique<c2::BoxCollider>(halfSize), position, angle, mass, color);
    shape.body.setBoxMass(mass, halfSize); // without an inertia, contacts can't make it spin
    return shape;
}

Shape2D& Physics2DGame::addCircle(c2::Vector2 position, real radius, Color color)
{
    const real mass = (real)3.14159 * radius * radius;
    Shape2D& shape = addShape(std::make_unique<c2::CircleCollider>(radius), position, 0, mass, color);
    shape.body.setCircleMass(mass, radius);
    return shape;
}

Shape2D& Physics2DGame::addPolygon(c2::Vector2 position, int sides, real radius, Color color, real angle)
{
    auto pPolygon = std::make_unique<c2::PolygonCollider>(c2::PolygonCollider::regular(sides, radius));
    const real mass = pPolygon->getArea();
    const real inertia = pPolygon->computeInertia(mass);
    Shape2D& shape = addShape(std::move(pPolygon), position, angle, mass, color);
    shape.body.setInertia(inertia);
    return shape;
}

void Physics2DGame::addStaticBox(c2::Vector2 position, c2::Vector2 halfSize, real angle)
{
    // No body: the offset places it in the world
    auto pBox = std::make_unique<c2::BoxCollider>(halfSize);
    pBox->offset = c2::Transform(position, angle);
    mWorld.colliders.push_back(pBox.get());
    mScenery.push_back(std::move(pBox));
}

void Physics2DGame::addChain(c2::Vector2 anchor, int links)
{
    // Each joint sits in the gap between two links, so neighbours don't collide where they meet
    const c2::Vector2 halfSize(0.4f, 0.08f);
    const real reach = halfSize.x + (real)0.1;
    c2::RigidBody* pPrevious = nullptr;
    c2::Vector2 previousPoint = anchor;
    for (int i = 0; i < links; i++)
    {
        const c2::Vector2 centre = anchor + c2::Vector2(reach * (real)(2 * i + 1), 0);
        Shape2D& link = addBox(centre, halfSize, i % 2 == 0 ? RED : MAROON);

        // A real chain loses energy at every link, so it settles instead of swaying for minutes
        link.body.setLinearDamping(0.5f);
        link.body.setAngularDamping(0.5f);

        auto pJoint = std::make_unique<c2::Joint>();
        pJoint->set(&link.body, c2::Vector2(-reach, 0), pPrevious, previousPoint);
        mWorld.joints.push_back(pJoint.get());
        mJoints.push_back(std::move(pJoint));

        pPrevious = &link.body;
        previousPoint = c2::Vector2(reach, 0);
    }
}

void Physics2DGame::spawn(SpawnKind kind, c2::Vector2 position)
{
    const real angle = (real)GetRandomValue(0, 314) / 100;
    switch (kind)
    {
    case SpawnKind::Box:
        addBox(position, { (real)GetRandomValue(25, 60) / 100, (real)GetRandomValue(25, 60) / 100 }, randomColor(), angle);
        break;
    case SpawnKind::Circle:
        addCircle(position, (real)GetRandomValue(25, 55) / 100, randomColor());
        break;
    case SpawnKind::Polygon:
        addPolygon(position, GetRandomValue(3, 8), (real)GetRandomValue(35, 65) / 100, randomColor(), angle);
        break;
    }
}

void Physics2DGame::startDrag(c2::Vector2 worldPoint)
{
    c2::Collider* pHit = mWorld.findColliderAt(worldPoint);
    if (pHit == nullptr || pHit->pBody == nullptr)
    {
        spawn(mSpawnKind, worldPoint);
        return;
    }
    for (const auto& pShape : mShapes)
    {
        if (&pShape->body == pHit->pBody)
        {
            mpDragged = pShape.get();
        }
    }
    if (mpDragged == nullptr)
    {
        return;
    }

    // A stiff, damped spring from the grabbed point to the cursor, scaled by mass so everything follows alike
    const real mass = mpDragged->body.getMass();
    mpDragSpring = std::make_unique<c2::RigidBodySpring>(mpDragged->body.getPointInLocalSpace(worldPoint), nullptr, worldPoint, 80 * mass,
                                                         (real)0, 12 * mass);
    mWorld.forces.add(&mpDragged->body, mpDragSpring.get());
    mpDragged->body.setAwake(true);
}

void Physics2DGame::stopDrag()
{
    if (mpDragged != nullptr)
    {
        mWorld.forces.remove(&mpDragged->body, mpDragSpring.get());
    }
    mpDragged = nullptr;
    mpDragSpring.reset();
}

void Physics2DGame::onFixedUpdate(double dt)
{
    for (const auto& pShape : mShapes)
    {
        pShape->previousPosition = pShape->body.getPosition();
        pShape->previousAngle = pShape->body.getOrientation();
    }
    mWorld.runPhysics((real)dt);
}

void Physics2DGame::onUpdate(double, double alpha)
{
    mAlpha = (float)alpha;
    updateView();

    const bool mouseFree = !ImGui::GetIO().WantCaptureMouse;
    const c2::Vector2 mouse = toWorld(GetMousePosition());
    if (mouseFree && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        startDrag(mouse);
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
    {
        stopDrag();
    }
    if (mpDragSpring != nullptr)
    {
        mpDragSpring->otherConnectionPoint = mouse;
    }
    if (mouseFree && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
    {
        spawn(SpawnKind::Circle, mouse);
    }

    if (IsKeyPressed(KEY_ONE))
    {
        mSpawnKind = SpawnKind::Box;
    }
    if (IsKeyPressed(KEY_TWO))
    {
        mSpawnKind = SpawnKind::Circle;
    }
    if (IsKeyPressed(KEY_THREE))
    {
        mSpawnKind = SpawnKind::Polygon;
    }
    if (IsKeyPressed(KEY_SPACE))
    {
        spawn(mSpawnKind, { (real)GetRandomValue(-80, 80) / 10, 13.0f });
    }
    if (IsKeyPressed(KEY_R))
    {
        resetScene();
    }
#ifndef __EMSCRIPTEN__
    if (IsKeyPressed(KEY_ESCAPE))
    {
        requestQuit();
    }
#endif
}

void Physics2DGame::updateView()
{
    const float width = (float)GetScreenWidth();
    const float height = (float)GetScreenHeight();
    mPixelsPerMetre = std::min(width / (2 * kViewHalfWidth), height / kViewHeight);
    mOrigin = { width / 2, height - 0.3f * mPixelsPerMetre };
}

Vector2 Physics2DGame::toScreen(c2::Vector2 point) const
{
    return { mOrigin.x + (float)point.x * mPixelsPerMetre, mOrigin.y - (float)point.y * mPixelsPerMetre };
}

c2::Vector2 Physics2DGame::toWorld(Vector2 screenPoint) const
{
    return toCyclone(Vector2{ (screenPoint.x - mOrigin.x) / mPixelsPerMetre, (mOrigin.y - screenPoint.y) / mPixelsPerMetre });
}

void Physics2DGame::drawCollider(const c2::Collider& collider, const c2::Transform& pose, Color color) const
{
    const Color outline = ColorBrightness(color, -0.5f);
    if (collider.type == c2::ColliderType::Circle)
    {
        const float radius = (float)static_cast<const c2::CircleCollider&>(collider).radius * mPixelsPerMetre;
        const Vector2 centre = toScreen(pose.position);
        DrawCircleV(centre, radius, color);
        DrawCircleLinesV(centre, radius, outline);

        // A spoke, so rolling shows
        DrawLineEx(centre, toScreen(pose.transform(c2::Vector2(static_cast<const c2::CircleCollider&>(collider).radius, 0))), 2.0f, outline);
        return;
    }
    if (collider.type != c2::ColliderType::Polygon)
    {
        return;
    }

    // Counter-clockwise in the world stays counter-clockwise on screen, which is the order raylib's fan wants
    const auto& vertices = static_cast<const c2::PolygonCollider&>(collider).getVertices();
    std::vector<Vector2> points;
    points.push_back(toScreen(pose.position));
    for (const c2::Vector2& vertex : vertices)
    {
        points.push_back(toScreen(pose.transform(vertex)));
    }
    points.push_back(points[1]);
    DrawTriangleFan(points.data(), (int)points.size(), color);
    DrawLineStrip(points.data() + 1, (int)points.size() - 1, outline);
}

void Physics2DGame::onDraw2D()
{
    const Color sceneryColor{ 90, 96, 108, 255 };
    DrawRectangleV(toScreen({ -kWallX - 2, 0 }), { (2 * kWallX + 4) * mPixelsPerMetre, 2 * mPixelsPerMetre }, sceneryColor);
    DrawRectangleV(toScreen({ -kWallX - 2, kViewHeight }), { 2 * mPixelsPerMetre, kViewHeight * mPixelsPerMetre }, sceneryColor);
    DrawRectangleV(toScreen({ kWallX, kViewHeight }), { 2 * mPixelsPerMetre, kViewHeight * mPixelsPerMetre }, sceneryColor);
    for (const auto& pCollider : mScenery)
    {
        drawCollider(*pCollider, pCollider->offset, sceneryColor);
    }

    for (const auto& pShape : mShapes)
    {
        // Physics runs at a fixed rate, so draw each shape between its last two poses
        const c2::Vector2 position = pShape->previousPosition + (pShape->body.getPosition() - pShape->previousPosition) * (real)mAlpha;
        const real angle = pShape->previousAngle + (pShape->body.getOrientation() - pShape->previousAngle) * (real)mAlpha;
        const Color color = pShape->body.isAwake() ? pShape->color : ColorBrightness(pShape->color, -0.4f);
        drawCollider(*pShape->pCollider, c2::Transform(position, angle) * pShape->pCollider->offset, color);
    }

    for (const auto& pJoint : mJoints)
    {
        const Vector2 a = toScreen(pJoint->getWorldPosition(0));
        const Vector2 b = toScreen(pJoint->getWorldPosition(1));
        DrawLineEx(a, b, 2.0f, LIGHTGRAY);
        DrawCircleV(b, 3.0f, LIGHTGRAY);
    }

    if (mpDragged != nullptr)
    {
        const Vector2 grabbed = toScreen(mpDragged->body.getPointInWorldSpace(mpDragSpring->connectionPoint));
        DrawLineEx(grabbed, GetMousePosition(), 2.0f, YELLOW);
        DrawCircleV(grabbed, 4.0f, YELLOW);
    }

    if (mShowContacts)
    {
        const c2::Contact* contacts = mWorld.getContacts();
        for (unsigned i = 0; i < mWorld.getContactCount(); i++)
        {
            const Vector2 point = toScreen(contacts[i].point);
            DrawCircleV(point, 3.0f, RED);
            DrawLineEx(point, toScreen(contacts[i].point + contacts[i].normal * (real)0.4), 1.5f, RED);
        }
    }
}

void Physics2DGame::onGui()
{
    ImGui::SetNextWindowPos({ 10.0f, 10.0f }, ImGuiCond_FirstUseEver);
    ImGui::Begin("2D Physics");
    ImGui::Text("Bodies: %d (%u awake)", (int)mShapes.size(), mWorld.getAwakeCount());
    ImGui::Text("Contacts: %u, pairs tested: %u", mWorld.getContactCount(), mWorld.getPairsTested());

    int kind = (int)mSpawnKind;
    ImGui::RadioButton("Box (1)", &kind, (int)SpawnKind::Box);
    ImGui::SameLine();
    ImGui::RadioButton("Circle (2)", &kind, (int)SpawnKind::Circle);
    ImGui::SameLine();
    ImGui::RadioButton("Polygon (3)", &kind, (int)SpawnKind::Polygon);
    mSpawnKind = (SpawnKind)kind;

    float friction = (float)mWorld.friction;
    float restitution = (float)mWorld.restitution;
    if (ImGui::SliderFloat("Friction", &friction, 0.0f, 1.0f))
    {
        mWorld.friction = (real)friction;
    }
    if (ImGui::SliderFloat("Restitution", &restitution, 0.0f, 1.0f))
    {
        mWorld.restitution = (real)restitution;
    }
    ImGui::Checkbox("Show contacts", &mShowContacts);

    if (ImGui::Button("Drop shape (Space)"))
    {
        spawn(mSpawnKind, { (real)GetRandomValue(-80, 80) / 10, 13.0f });
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset (R)"))
    {
        resetScene();
    }
    ImGui::TextDisabled("Left click: drag a shape, or drop one on empty space");
    ImGui::TextDisabled("Right click: drop a ball");
    ImGui::End();
}
