#include "solar/PlanetsScene.h"

#include <random>

#include <imgui.h>

#include <engine/Conversions.h>

namespace tenth
{
    namespace
    {
        constexpr int ROOM_SIZE = 170;
        constexpr cyclone::real PLANET_RADIUS = 10;
        constexpr cyclone::real PLANET_MASS = 6000;
        constexpr cyclone::real SUN_RADIUS = 14;
        constexpr cyclone::real SUN_MASS = 200000000;
        constexpr cyclone::real ORBIT_SPEED = 1.75; // sideways speed per metre from the sun at launch
        constexpr cyclone::real CLICK_FORCE = 10;
        constexpr int MAX_CONTACTS = 1000;

        std::mt19937& randomEngine()
        {
            static std::mt19937 engine(std::random_device{}());
            return engine;
        }

        int randomBetween(int low, int high)
        {
            return std::uniform_int_distribution<int>(low, high)(randomEngine());
        }

        // Pushes a coordinate near the sun out by half the room, so no planet starts inside it
        cyclone::real awayFromCentre(cyclone::real value)
        {
            if (value >= 0 && value < ROOM_SIZE / 3)
            {
                return value + ROOM_SIZE / 2;
            }
            if (value < 0 && value > -ROOM_SIZE / 3)
            {
                return value - ROOM_SIZE / 2;
            }
            return value;
        }
    }

    PlanetsScene::PlanetsScene(Camera3D& camera) : mCamera(camera), mContacts(MAX_CONTACTS)
    {
        mCollisionData.contactArray = mContacts.data();
        mResolver.restitution = mRestitution;
        for (int i = 0; i < PLANET_COUNT; i++)
        {
            addBody(i, PLANET_RADIUS, PLANET_MASS, randomPosition());
        }
        addBody(SUN, SUN_RADIUS, SUN_MASS, cyclone::Vector3(0, 0, 0));
        launch();
    }

    void PlanetsScene::addBody(int index, cyclone::real radius, cyclone::real mass, cyclone::Vector3 position)
    {
        Body& body = mBodies[index];
        body.rigidBody.setPosition(position);
        body.rigidBody.setMass(mass);
        const cyclone::real inertia = 0.4 * mass * radius * radius;
        cyclone::Matrix3 tensor;
        tensor.setDiagonal(inertia, inertia, inertia);
        body.rigidBody.setInertiaTensor(tensor);
        body.rigidBody.setAcceleration(cyclone::Vector3(0, 0, 0));
        body.rigidBody.setLinearDamping(1);
        body.rigidBody.setCanSleep(false);
        body.collider.radius = radius;
        body.collider.pBody = &body.rigidBody;
        body.collider.updateInternals();
    }

    cyclone::Vector3 PlanetsScene::randomPosition()
    {
        const cyclone::real x = awayFromCentre(randomBetween(-ROOM_SIZE, ROOM_SIZE));
        const cyclone::real y = randomBetween(-ROOM_SIZE, ROOM_SIZE) / 4;
        const cyclone::real z = awayFromCentre(randomBetween(-ROOM_SIZE, ROOM_SIZE));
        return cyclone::Vector3(x, y, z);
    }

    // Sends each planet sideways around the sun, faster the further out it is, with a random tilt
    void PlanetsScene::launch()
    {
        const cyclone::Vector3 sun = mBodies[SUN].rigidBody.getPosition();
        for (int i = 0; i < PLANET_COUNT; i++)
        {
            const cyclone::Vector3 offset = mBodies[i].rigidBody.getPosition() - sun;
            const cyclone::real tilt = randomBetween(-15, 15);
            mBodies[i].rigidBody.setLinearVelocity(cyclone::Vector3(-offset.z * ORBIT_SPEED, tilt, offset.x * ORBIT_SPEED));
        }
    }

    void PlanetsScene::resetPositions()
    {
        for (int i = 0; i < PLANET_COUNT; i++)
        {
            mBodies[i].rigidBody.setPosition(randomPosition());
            mBodies[i].collider.updateInternals();
        }
        launch();
    }

    void PlanetsScene::onEnter()
    {
        if (!mModelsReady)
        {
            mPlanetModel = LoadModelFromMesh(GenMeshSphere((float)PLANET_RADIUS, 16, 16));
            mSunModel = LoadModelFromMesh(GenMeshSphere((float)SUN_RADIUS, 16, 16));
            mModelsReady = true;
        }
        mCamera.position = { 80.0f, 30.0f, 15.0f };
        mCamera.target = { 0.0f, 0.0f, 0.0f };
        mCamera.up = { 0.0f, 1.0f, 0.0f };
        mCamera.fovy = 45.0f;
        mCamera.projection = CAMERA_PERSPECTIVE;
    }

    void PlanetsScene::onShutdown()
    {
        if (mModelsReady)
        {
            UnloadModel(mPlanetModel);
            UnloadModel(mSunModel);
            mModelsReady = false;
        }
    }

    // Every body pulls each planet toward it; the sun is held in place
    void PlanetsScene::applyGravity()
    {
        for (int i = 0; i < PLANET_COUNT; i++)
        {
            cyclone::RigidBody& planet = mBodies[i].rigidBody;
            for (int j = 0; j < PLANET_COUNT + 1; j++)
            {
                if (i == j)
                {
                    continue;
                }
                const cyclone::Vector3 difference = mBodies[j].rigidBody.getPosition() - planet.getPosition();
                cyclone::Vector3 direction = difference;
                direction.normalise();
                const cyclone::real inverseSquare = 1 / difference.squareMagnitude();
                planet.addForceAtPoint(difference * inverseSquare * mBodies[j].rigidBody.getMass(), direction * -1);
            }
        }
    }

    // Holding the left mouse on a planet pushes it away from the camera
    void PlanetsScene::pushClickedPlanet()
    {
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT) || ImGui::GetIO().WantCaptureMouse)
        {
            return;
        }
        const Ray ray = GetScreenToWorldRay(GetMousePosition(), mCamera);
        for (Body& body : mBodies)
        {
            const RayCollision hit = GetRayCollisionSphere(ray, engine::toRaylib(body.rigidBody.getPosition()), (float)body.collider.radius);
            if (hit.hit)
            {
                body.rigidBody.addForceAtPoint(engine::toCyclone(ray.direction) * CLICK_FORCE, engine::toCyclone(hit.point));
            }
        }
    }

    void PlanetsScene::onFixedUpdate(double dt)
    {
        applyGravity();
        pushClickedPlanet();
        mBodies[SUN].rigidBody.setLinearVelocity(cyclone::Vector3(0, 0, 0));
        for (Body& body : mBodies)
        {
            body.rigidBody.integrate((cyclone::real)dt);
            body.collider.updateInternals();
            body.collider.overlapCount = 0;
        }

        mCollisionData.reset(MAX_CONTACTS);
        for (int i = 0; i < (int)mBodies.size(); i++)
        {
            for (int j = i + 1; j < (int)mBodies.size(); j++)
            {
                if (cyclone::CollisionTests::Generate(mBodies[i].collider, mBodies[j].collider, &mCollisionData) > 0)
                {
                    mBodies[i].collider.overlapCount++;
                    mBodies[j].collider.overlapCount++;
                }
            }
        }
        mResolver.restitution = mRestitution;
        mResolver.resolveContacts(mCollisionData.contactArray, mCollisionData.contactCount);
    }

    void PlanetsScene::onUpdate(double frameDt, double /*alpha*/)
    {
        mFlyCamera.update(mCamera, (float)frameDt);
    }

    void PlanetsScene::onDraw3D()
    {
        ClearBackground(BLACK);
        for (int i = 0; i < (int)mBodies.size(); i++)
        {
            const Body& body = mBodies[i];
            const bool isSun = i == SUN;
            const bool touching = body.collider.overlapCount > 0;
            Model& model = isSun ? mSunModel : mPlanetModel;
            model.transform = engine::toRaylib(body.collider.transform);
            const Color fill = isSun ? (touching ? YELLOW : ORANGE) : (touching ? RED : LIME);
            DrawModel(model, { 0.0f, 0.0f, 0.0f }, 1.0f, fill);
            DrawModelWires(model, { 0.0f, 0.0f, 0.0f }, 1.0f, isSun ? YELLOW : BLACK);
        }
        DrawGrid(100, 1.0f);
    }

    void PlanetsScene::onGui()
    {
        ImGui::Begin("Planets", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);
        if (ImGui::Button("Launch", { 200.0f, 0.0f }))
        {
            launch();
        }
        if (ImGui::Button("Reset", { 200.0f, 0.0f }))
        {
            resetPositions();
        }
        ImGui::Text("Coefficient of restitution");
        ImGui::SetNextItemWidth(200.0f);
        ImGui::SliderFloat("##restitution", &mRestitution, 0.0f, 1.25f);
        ImGui::TextDisabled("Hold right mouse to look, WASD to fly.\nHold left mouse on a planet to push it.");
        if (ImGui::Button("Back to assembly", { 200.0f, 0.0f }))
        {
            requestScene(SceneId::Build);
        }
        ImGui::End();
    }
}
