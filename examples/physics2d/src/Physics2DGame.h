#pragma once

#include <memory>
#include <vector>

#include <cyclone2d/cyclone2d.h>
#include <engine/Game.h>

// A body with its shape, plus its pose before the last physics step so drawing can blend between steps
struct Shape2D
{
    cyclone2d::RigidBody body;
    std::unique_ptr<cyclone2d::Collider> pCollider;
    cyclone2d::Vector2 previousPosition;
    cyclone2d::real previousAngle = 0;
    Color color{};
};

struct Physics2DSettings
{
    float friction = 0.6f;
    float restitution = 0.2f;
};

enum class SpawnKind
{
    Box,
    Circle,
    Polygon
};

// The 2D physics engine on its own: stacks, a ramp, a chain on joints, and shapes to drop and drag with the mouse
class Physics2DGame : public engine::Game
{
public:
    Physics2DGame(engine::GameConfig config, Physics2DSettings physics);

private:
    void onInit() override;
    void onFixedUpdate(double dt) override;
    void onUpdate(double frameDt, double alpha) override;
    void onDraw2D() override;
    void onGui() override;

    void resetScene();
    Shape2D& addShape(std::unique_ptr<cyclone2d::Collider> pCollider, cyclone2d::Vector2 position, cyclone2d::real angle, cyclone2d::real mass,
                      Color color);
    Shape2D& addBox(cyclone2d::Vector2 position, cyclone2d::Vector2 halfSize, Color color, cyclone2d::real angle = 0);
    Shape2D& addCircle(cyclone2d::Vector2 position, cyclone2d::real radius, Color color);
    Shape2D& addPolygon(cyclone2d::Vector2 position, int sides, cyclone2d::real radius, Color color, cyclone2d::real angle = 0);
    void addStaticBox(cyclone2d::Vector2 position, cyclone2d::Vector2 halfSize, cyclone2d::real angle);
    void addChain(cyclone2d::Vector2 anchor, int links);
    void spawn(SpawnKind kind, cyclone2d::Vector2 position);

    void startDrag(cyclone2d::Vector2 worldPoint);
    void stopDrag();

    // The view: metres to pixels with y pointing up, fitted to the window each frame
    void updateView();
    Vector2 toScreen(cyclone2d::Vector2 point) const;
    cyclone2d::Vector2 toWorld(Vector2 screenPoint) const;
    void drawCollider(const cyclone2d::Collider& collider, const cyclone2d::Transform& pose, Color color) const;

    // The world keeps pointers to bodies, colliders, joints and springs, so each lives at a fixed address
    cyclone2d::World mWorld;
    std::vector<std::unique_ptr<cyclone2d::Collider>> mScenery;
    std::vector<std::unique_ptr<Shape2D>> mShapes;
    std::vector<std::unique_ptr<cyclone2d::Joint>> mJoints;

    std::unique_ptr<cyclone2d::RigidBodySpring> mpDragSpring;
    Shape2D* mpDragged = nullptr;

    SpawnKind mSpawnKind = SpawnKind::Box;
    bool mShowContacts = false;
    float mAlpha = 0.0f;
    float mPixelsPerMetre = 40.0f;
    Vector2 mOrigin{};
};
