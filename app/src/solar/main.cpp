#include <iostream>

#include <raylib.h>
#include <raymath.h> //Needed for most Raylib math functions
#include <cyclone/cyclone.h>
#include <imgui.h>
#include <rlImGui.h>

#include <ew/camera.h>
#include <ew/conversions.h>

#include <engine/helpers.h>
#include <engine/Debug.h>

#include <random>
#include <math.h>

using namespace engine;

struct AppSettings {
    cyclone::real cor = 0.5;
}_appSettings;

void initCamera(Camera& camera)
{
    camera.position = { 80.0f, 30.0f, 15.0f };   // Camera position
    camera.target = { 0.0f, 0.0f, 0.0f };       // Camera looking at point
    camera.up = { 0.0f, 1.0f, 0.0f };           // Camera up vecto
    camera.fovy = 45.0f;                        // Camera vertical field-of-view in degrees
    camera.projection = CAMERA_PERSPECTIVE;     // Camera projection type - perspective vs orthographic
}
void handleInput(Camera& camera, cyclone::real dt)
{
    //Input
    if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
        DisableCursor();
    }
    if (IsMouseButtonReleased(MOUSE_RIGHT_BUTTON)) {
        EnableCursor();
    }
    //Only allow movement if the cursor is hidden
    if (IsCursorHidden()) {
        ew::UpdateFlyCamera(&camera, dt);
    }
}

void updateRigidbodys(const std::vector<std::pair<cyclone::Collider*, Model*>>& colliders, cyclone::real dt, const Camera3D& camera)
{
    // update rigidbody
    for (auto c : colliders)
    {
        cyclone::Collider* collider = c.first;
        Model* model = c.second;
        // update rigidbody
        if (collider->hasRigidbody())
        {
            collider->getRigidbody()->integrate(dt);
            // Get point if rigidbody is clicked
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
                RayCollision rayHit = GetRayCollisionMesh(ray, model->meshes[0], ew::CTR(collider->getTransform()));
                if (rayHit.hit) {
                    collider->getRigidbody()->addForceAtPoint(ew::RTC(ray.direction) * 10, ew::RTC(rayHit.point));
                }
            }
        }
    }
}
void updateColliders(const std::vector<std::pair<cyclone::Collider*, Model*>>& colliders)
{
    for (auto c : colliders)
    {
        // update internals
        cyclone::Collider* collider = c.first;
        collider->updateInternals();
    }
}
void checkCollisions(const std::vector<std::pair<cyclone::Collider*, Model*>>& colliders, cyclone::CollisionData* data)
{
    for (int i = 0; i < colliders.size(); i++)
    {
        for (int j = i + 1; j < colliders.size(); j++)
        {
            // check for overlap
            bool hit = cyclone::CollisionTests::checkCollide(colliders[i].first, colliders[j].first, data);
            if (hit)
            {
                colliders[i].first->incrementOverlapCount();
                colliders[j].first->incrementOverlapCount();
            }
        }
    }
}
void draw(const std::vector<std::pair<cyclone::Collider*, Model*>>& colliders, const std::pair<cyclone::Collider*, Model*>& sunCollider)
{
    // draw
    for (auto c : colliders)
    {
        c.second->transform = ew::CTR(c.first->getTransform());
        DrawModel(*c.second, { 0, 0, 0 }, 1, (c.first->getType() == cyclone::ColliderType::Plane) ? WHITE : (c.first->getOverlapCount() > 0) ? RED : LIME);
        DrawModelWires(*c.second, { 0, 0, 0 }, 1, BLACK);
    }
    sunCollider.second->transform = ew::CTR(sunCollider.first->getTransform());
    DrawModel(*sunCollider.second, { 0, 0, 0 }, 1, (sunCollider.first->getType() == cyclone::ColliderType::Plane) ? WHITE : (sunCollider.first->getOverlapCount() > 0) ? YELLOW : ORANGE);
    DrawModelWires(*sunCollider.second, { 0, 0, 0 }, 1, YELLOW);
}

const int BALL_COUNT = 10;
const int ROOM_SIZE = 170;
const int PLANET_SIZE = 10;
const int MOON_SIZE = 1;
const int SUN_SIZE = 14;

void launch(const std::vector<std::pair<cyclone::Collider*, Model*>>& planets, const std::vector<std::pair<cyclone::Collider*, Model*>>& moons, const std::pair<cyclone::Collider*, Model*>& sun)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> mag(-15, 15);
    for (int i = 0; i < planets.size(); i++)
    {
        if (!planets[i].first->getRigidbody()) continue;

        cyclone::Vector3 difference = planets[i].first->getRigidbody()->getPos() - sun.first->getRigidbody()->getPos();
        
        float tangent = atan(difference.z/difference.x);
        tangent += 3.14159;
        difference = cyclone::Vector3(difference.z/tan(tangent), mag(gen), difference.x * tan(tangent));

        cyclone::Vector3 vel(-difference.z*1.75, mag(gen), difference.x* 1.75);
        planets[i].first->getRigidbody()->setLinearVelocity(vel);

        /*
        if (!moons[i].first->getRigidbody()) continue;

        cyclone::Vector3 difference2 = moons[i].first->getRigidbody()->getPos() - planets[i].first->getRigidbody()->getPos();

        tangent = atan(difference2.z / difference2.x);
        tangent += 3.14159;
        difference2 = cyclone::Vector3((difference2.z / tan(tangent)) + (difference.x*2), difference.y, (difference2.x * tan(tangent)) + (difference.z*2));

        cyclone::Vector3 vel2(-difference2.z, mag(gen), difference2.x);
        moons[i].first->getRigidbody()->setLinearVelocity(vel2);*/
    }

}

void resetPos(const std::vector<std::pair<cyclone::Collider*, Model*>>& planets, const std::vector<std::pair<cyclone::Collider*, Model*>>& moons, const std::pair<cyclone::Collider*, Model*>& sun)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(-ROOM_SIZE, ROOM_SIZE);

    for (int i = 0; i < planets.size(); i++)
    {
        if (!planets[i].first->getRigidbody()) continue;

        cyclone::Vector3 pos(dist(gen), dist(gen)/4, dist(gen));
        if (pos.x < ROOM_SIZE / 3 && pos.x >= 0)
        {
            pos.x += ROOM_SIZE / 2;
        }
        else if (pos.x > -ROOM_SIZE / 3 && pos.x < 0)
        {
            pos.x -= ROOM_SIZE / 2;
        }

        if (pos.z < ROOM_SIZE / 3 && pos.z >= 0)
        {
            pos.z += ROOM_SIZE / 2;
        }
        else if (pos.z > -ROOM_SIZE / 3 && pos.z < 0)
        {
            pos.z -= ROOM_SIZE / 2;
        }
        planets[i].first->getRigidbody()->setPosition(pos);
        //moons[i].first->getRigidbody()->setPosition(pos + cyclone::Vector3(PLANET_SIZE * 1.5, PLANET_SIZE * 1, PLANET_SIZE * 1));
    }
    launch(planets, moons, sun);
}


int main() {
   // Initialization \
   --------------------------------------------------------------------------------------

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "contacting on my twitter.com");

    rlImGuiSetup(true);

    Camera3D camera = { 0 };
    initCamera(camera);

    // create models
    Model smallSphereModel = LoadModelFromMesh(GenMeshSphere(MOON_SIZE, 16, 16));
    Model sphereModel = LoadModelFromMesh(GenMeshSphere(PLANET_SIZE, 16, 16));
    Model bigSphereModel = LoadModelFromMesh(GenMeshSphere(SUN_SIZE, 16, 16));

    // create vector of objects 
    std::vector<std::pair<cyclone::Collider*, Model*>> colliders;
    std::vector<std::pair<cyclone::Collider*, Model*>> collidersPlanets;
    std::vector<std::pair<cyclone::Collider*, Model*>> collidersMoons;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(-ROOM_SIZE, ROOM_SIZE);
    std::uniform_int_distribution<int> mag(-45, 45);

    // create all the balls
    for (int i = 0; i < BALL_COUNT; i++)
    {
        cyclone::Vector3 pos(dist(gen), dist(gen) / 4, dist(gen));
        if (pos.x < ROOM_SIZE/3 && pos.x >= 0)
        {
            pos.x += ROOM_SIZE / 2;
        }
        else if (pos.x > -ROOM_SIZE / 3 && pos.x < 0)
        {
            pos.x -= ROOM_SIZE / 2;
        }

        if (pos.z < ROOM_SIZE / 3 && pos.z >= 0)
        {
            pos.z += ROOM_SIZE / 2;
        }
        else if (pos.z > -ROOM_SIZE / 3 && pos.z < 0)
        {
            pos.z -= ROOM_SIZE / 2;
        }

        cyclone::Rigidbody* newRigidbody = new cyclone::Rigidbody(pos);
        cyclone::SphereCollider* sphereCollider = new cyclone::SphereCollider(newRigidbody, cyclone::Matrix4(), PLANET_SIZE);
        newRigidbody->setMass(6000);
        newRigidbody->setInertiaTensor(cyclone::getSphereInertiaTensor(newRigidbody->getMass(), sphereCollider->getRadius()));
        newRigidbody->setAcceleration({ 0, 0, 0 });
        newRigidbody->setLinearDaming(1);

        colliders.push_back({ sphereCollider, &sphereModel });
        collidersPlanets.push_back({ sphereCollider, &sphereModel });

        /*
        cyclone::Rigidbody* moonRigidbody = new cyclone::Rigidbody(pos + cyclone::Vector3(PLANET_SIZE*1.5,PLANET_SIZE*1,PLANET_SIZE*1));
        cyclone::SphereCollider* sphereCollider2 = new cyclone::SphereCollider(moonRigidbody, cyclone::Matrix4(), MOON_SIZE);
        moonRigidbody->setMass(50);
        moonRigidbody->setInertiaTensor(cyclone::getSphereInertiaTensor(moonRigidbody->getMass(), sphereCollider2->getRadius()));
        moonRigidbody->setAcceleration({ 0, 0, 0 });


        colliders.push_back({ sphereCollider2, &smallSphereModel });
        collidersMoons.push_back({ sphereCollider2, &smallSphereModel });*/
    }

    std::vector<std::pair<cyclone::Collider*, Model*>> collidersWithoutSun = colliders;


    cyclone::Rigidbody* newRigidbody = new cyclone::Rigidbody(cyclone::Vector3(0,0,0));
    cyclone::SphereCollider* sphereCollider = new cyclone::SphereCollider(newRigidbody, cyclone::Matrix4(), SUN_SIZE);
    newRigidbody->setMass(200000000);
    newRigidbody->setInertiaTensor(cyclone::getSphereInertiaTensor(newRigidbody->getMass(), sphereCollider->getRadius()));
    newRigidbody->setAcceleration({ 0, 0, 0 });


    colliders.push_back({ sphereCollider, &bigSphereModel });
    


    cyclone::ContactResolver* resolver = new cyclone::ContactResolver();
    cyclone::CollisionData* data = new cyclone::CollisionData();

    Debug::success() << "Application Started";

    data->init(100000);

    

    launch(collidersPlanets, collidersMoons, colliders.back());

    while (!WindowShouldClose()) {
        //Seconds between previous frame and this one
        float deltaTime = GetFrameTime();

        handleInput(camera, deltaTime);
        data->setToStart();

        //Drawing
        BeginDrawing();

        //3D mode draws objects in right handed vector space
        BeginMode3D(camera);
        ClearBackground(BLACK);

        for (int i = 0; i < BALL_COUNT; i++)
        {
            for (int j = 0; j < (BALL_COUNT)+1; j++)
            {
                if (i != j)
                {/*
                    if (colliders[i].first->getRigidbody()->getMass() < 200)
                    {
                        if (colliders[j].first->getRigidbody()->getMass() > 200 && colliders[j].first->getRigidbody()->getMass() < 20000)
                        {
                            cyclone::Vector3 difference = (colliders[j].first->getRigidbody()->getPos() - colliders[i].first->getRigidbody()->getPos());
                            cyclone::Vector3 norm = difference;
                            norm.normalise();

                            float invDifference = 1 / (difference.squareMagnitude());
                            colliders[i].first->getRigidbody()->addForceAtPoint(difference * invDifference * colliders[j].first->getRigidbody()->getMass() * 20, -norm);
                        }
                    }*/
                    cyclone::Vector3 difference = (colliders[j].first->getRigidbody()->getPos() - colliders[i].first->getRigidbody()->getPos());
                    cyclone::Vector3 norm = difference;
                    norm.normalise();
                    float invDifference = 1 / (difference.squareMagnitude());
                    colliders[i].first->getRigidbody()->addForceAtPoint(difference * invDifference * colliders[j].first->getRigidbody()->getMass(), -norm);
                }
            }
        }

        colliders.back().first->getRigidbody()->setLinearVelocity(cyclone::Vector3(0, 0, 0));

        updateRigidbodys(colliders, deltaTime, camera);

        updateColliders(colliders);
        checkCollisions(colliders, data);
        resolver->resolveContacts(data->getArray(), data->getCount(), deltaTime);

        draw(collidersWithoutSun, colliders.back());

        DrawGrid(100, 1.0f);
        EndMode3D();

        //Outside of 3D mode, we can draw 2D things directly to the screen in pixel coordinates
        DrawFPS(GetScreenWidth() - 128, 16);

        //ImGUI
        rlImGuiBegin();
        ImGui::Begin("SETTINGS");

        if (ImGui::Button("Launch", ImVec2(-1, 20)))
            launch(collidersPlanets, collidersMoons, colliders.back());

        if (ImGui::Button("Reset", ImVec2(-1, 20)))
            resetPos(collidersPlanets, collidersMoons, colliders.back());

        ImGui::Text("Coefficient of restitution");
        if (ImGui::SliderFloat("##COR", &_appSettings.cor, 0, 1.25))
            resolver->setCOR(_appSettings.cor);

        ImGui::End();
        rlImGuiEnd();
        EndDrawing();
    }

    // Cleanup
    for (auto c : colliders)
    {
        if (c.first->getRigidbody() != nullptr)
            delete c.first->getRigidbody();

        delete c.first;
    }

    data->cleanup();
    
    UnloadModel(sphereModel);

    CloseWindow();

    Debug::warn() << "Application Closing";

    Debug::removeInstance();
}