#include "flight/FlightScene.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <imgui.h>
#include <rlgl.h>

#include "flight/PartDrawing.h"

namespace tenth
{
    namespace
    {
        constexpr float PAD_HALF_WIDTH = 8.0f;
        constexpr float PAD_THICKNESS = 1.0f;
        constexpr float PAD_LIFT = 0.02f; // the pad's top sits this far above the ground so the two don't flicker
        constexpr Vector3 TOWER_POSITION = { -2.8f, 0.0f, -3.4f }; // behind the rocket from where the camera starts
        constexpr float TOWER_WIDTH = 1.4f;
        constexpr Vector3 SUN_DIRECTION = { 0.45f, 0.75f, 0.48f };
        constexpr Color GROUND_COLOR = { 74, 96, 62, 255 };
        constexpr Color PAD_COLOR = { 150, 148, 140, 255 };
        constexpr Color TRENCH_COLOR = { 40, 40, 44, 255 };
        constexpr Color TOWER_COLOR = { 170, 70, 50, 255 };

        constexpr double NEAR_PLANE = 0.1;
        constexpr double FAR_PLANE = 5.0e6; // past the far side of Earth from low orbit

        constexpr int GROUND_RINGS = 36;
        constexpr int GROUND_SEGMENTS = 96;
        constexpr double GROUND_FIRST_RING = 2.0; // metres
        constexpr double GROUND_RING_GROWTH = 1.35;

        Vector3 operator+(Vector3 a, Vector3 b)
        {
            return { a.x + b.x, a.y + b.y, a.z + b.z };
        }

        // A patch of the curved surface around the pad, with rings spaced closer near the middle where the camera is.
        // The whole-planet sphere is too coarse to stand on, so this covers everything in sight from the ground.
        Mesh makeGroundMesh(double radius)
        {
            Mesh mesh{};
            mesh.vertexCount = 1 + GROUND_RINGS * GROUND_SEGMENTS;
            mesh.triangleCount = GROUND_SEGMENTS + (GROUND_RINGS - 1) * GROUND_SEGMENTS * 2;
            mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
            mesh.normals = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
            mesh.texcoords = (float*)MemAlloc(mesh.vertexCount * 2 * sizeof(float));
            mesh.indices = (unsigned short*)MemAlloc(mesh.triangleCount * 3 * sizeof(unsigned short));

            mesh.normals[1] = 1.0f;
            double distance = GROUND_FIRST_RING;
            for (int ring = 0; ring < GROUND_RINGS; ring++)
            {
                const double angle = distance / radius;
                const double across = radius * std::sin(angle);
                const double drop = radius * (std::cos(angle) - 1.0);
                for (int segment = 0; segment < GROUND_SEGMENTS; segment++)
                {
                    const double heading = 2.0 * std::numbers::pi * segment / GROUND_SEGMENTS;
                    const int vertex = 1 + ring * GROUND_SEGMENTS + segment;
                    mesh.vertices[vertex * 3 + 0] = (float)(across * std::cos(heading));
                    mesh.vertices[vertex * 3 + 1] = (float)drop;
                    mesh.vertices[vertex * 3 + 2] = (float)(across * std::sin(heading));
                    mesh.normals[vertex * 3 + 0] = (float)(std::sin(angle) * std::cos(heading));
                    mesh.normals[vertex * 3 + 1] = (float)std::cos(angle);
                    mesh.normals[vertex * 3 + 2] = (float)(std::sin(angle) * std::sin(heading));
                }
                distance *= GROUND_RING_GROWTH;
            }

            // Wound so each triangle faces up, the way the camera sees it
            int index = 0;
            for (int segment = 0; segment < GROUND_SEGMENTS; segment++)
            {
                const int next = (segment + 1) % GROUND_SEGMENTS;
                mesh.indices[index++] = 0;
                mesh.indices[index++] = (unsigned short)(1 + next);
                mesh.indices[index++] = (unsigned short)(1 + segment);
            }
            for (int ring = 0; ring + 1 < GROUND_RINGS; ring++)
            {
                for (int segment = 0; segment < GROUND_SEGMENTS; segment++)
                {
                    const int next = (segment + 1) % GROUND_SEGMENTS;
                    const unsigned short inner = (unsigned short)(1 + ring * GROUND_SEGMENTS + segment);
                    const unsigned short innerNext = (unsigned short)(1 + ring * GROUND_SEGMENTS + next);
                    const unsigned short outer = (unsigned short)(inner + GROUND_SEGMENTS);
                    const unsigned short outerNext = (unsigned short)(innerNext + GROUND_SEGMENTS);
                    mesh.indices[index++] = inner;
                    mesh.indices[index++] = innerNext;
                    mesh.indices[index++] = outer;
                    mesh.indices[index++] = innerNext;
                    mesh.indices[index++] = outerNext;
                    mesh.indices[index++] = outer;
                }
            }
            UploadMesh(&mesh, false);
            return mesh;
        }
    }

    FlightScene::FlightScene(Camera3D& camera, const RocketDesign& design, const SolarSystem& solarSystem)
        : mCamera(camera), mDesign(design), mSolarSystem(solarSystem)
    {
        mOrbit.minPitch = -1.5f;
        mOrbit.maxDistance = 3000.0f;
    }

    // The rocket rolls out as built and stands on the pad at the equator with its tanks full
    void FlightScene::onEnter()
    {
        const BodyId earth = mSolarSystem.findBody("Earth");
        const double radius = mSolarSystem.body(earth).radius;
        const Vehicle vehicle = makeVehicle(mDesign);
        mTime = 0.0;
        mLaunched = false;
        mRocket.setVehicle(vehicle);
        mRocket.place(earth, { radius + vehicle.groundClearance, 0.0, 0.0 }, {});
        mPad = SurfaceFrame::below(mRocket.position(), radius);
        mStats = computeStats(mDesign, mSolarSystem.surfaceGravity(earth));

        if (!mGraphicsReady)
        {
            mSkyReady = mSky.init();
            mEarthModel = LoadModelFromMesh(GenMeshSphere(1.0f, 96, 192));
            mGroundModel = LoadModelFromMesh(makeGroundMesh(radius));
            mGraphicsReady = true;
        }
        mPreviousNear = rlGetCullDistanceNear();
        mPreviousFar = rlGetCullDistanceFar();
        rlSetClipPlanes(NEAR_PLANE, FAR_PLANE);

        // Frames the rocket from a little way off, so a taller rocket starts further away
        const float height = (float)(mDesign.highestPoint() - mDesign.lowestPoint());
        mOrbit.distance = std::max(14.0f, height * 1.8f);
        mOrbit.yaw = 0.6f;
        mOrbit.pitch = 0.15f;
        onUpdate(0.0, 0.0);
    }

    void FlightScene::onExit()
    {
        rlSetClipPlanes(mPreviousNear, mPreviousFar);
    }

    void FlightScene::onFixedUpdate(double dt)
    {
        // The clamps hold it on the pad until launch
        if (!mLaunched)
        {
            return;
        }
        mRocket.step(dt, mSolarSystem, mTime);
        mTime += dt;
    }

    // The camera circles the middle of the rocket and never dips below the ground under it
    void FlightScene::onUpdate(double frameDt, double /*alpha*/)
    {
        const Vehicle& vehicle = mRocket.vehicle();
        const double middle = (vehicle.design.highestPoint() + vehicle.design.lowestPoint()) * 0.5;
        mOrbit.focus = { 0.0f, (float)(middle - vehicle.centerOfMass.y), 0.0f };
        mOrbit.minFocusHeight = mOrbit.focus.y;
        mOrbit.maxFocusHeight = mOrbit.focus.y;
        mOrbit.minCameraHeight = toScene(mPad.origin).y + 0.5f;
        mOrbit.update(mCamera, (float)frameDt);
    }

    Vector3 FlightScene::toScene(const Vec3d& position) const
    {
        return toRaylib(mPad.rotate(position - mRocket.position()));
    }

    // The planet is drawn first and ignores depth, so the finer ground patch and everything after draw over it
    void FlightScene::drawGround() const
    {
        const float radius = (float)mSolarSystem.body(mRocket.reference()).radius;
        rlDrawRenderBatchActive();
        rlDisableDepthTest();
        DrawModelEx(mEarthModel, toScene({}), { 0.0f, 1.0f, 0.0f }, 0.0f, { radius, radius, radius }, GROUND_COLOR);
        rlDrawRenderBatchActive();
        rlEnableDepthTest();
        DrawModel(mGroundModel, toScene(mPad.origin), 1.0f, GROUND_COLOR);
    }

    void FlightScene::onDraw3D()
    {
        if (mSkyReady)
        {
            mSky.draw(SUN_DIRECTION);
        }
        if (mGraphicsReady)
        {
            drawGround();
        }

        const Vector3 pad = toScene(mPad.origin);
        const Vector3 slab = pad + Vector3{ 0.0f, PAD_LIFT - PAD_THICKNESS * 0.5f, 0.0f };
        DrawCube(slab, PAD_HALF_WIDTH * 2.0f, PAD_THICKNESS, PAD_HALF_WIDTH * 2.0f, PAD_COLOR);
        DrawCubeWires(slab, PAD_HALF_WIDTH * 2.0f, PAD_THICKNESS, PAD_HALF_WIDTH * 2.0f, ColorBrightness(PAD_COLOR, -0.4f));

        // The flame trench opening under the rocket, a hair above the pad
        DrawCube(pad + Vector3{ 0.0f, PAD_LIFT + 0.01f, 0.0f }, 2.4f, 0.02f, 2.4f, TRENCH_COLOR);

        // The service tower beside the rocket, a little taller than it
        const float towerHeight = (float)(mDesign.highestPoint() - mDesign.lowestPoint()) + 4.0f;
        const Vector3 tower = pad + Vector3{ TOWER_POSITION.x, towerHeight * 0.5f, TOWER_POSITION.z };
        DrawCube(tower, TOWER_WIDTH, towerHeight, TOWER_WIDTH, TOWER_COLOR);
        DrawCubeWires(tower, TOWER_WIDTH, towerHeight, TOWER_WIDTH, ColorBrightness(TOWER_COLOR, -0.5f));

        // The rocket's center of mass is the scene's origin, and it stands upright until it can turn
        const Vehicle& vehicle = mRocket.vehicle();
        drawDesign(vehicle.design, toRaylib(Vec3d{} - vehicle.centerOfMass));
    }

    void FlightScene::onGui()
    {
        const BodyInfo& reference = mSolarSystem.body(mRocket.reference());
        const Vehicle& vehicle = mRocket.vehicle();
        const double altitude = mRocket.position().length() - reference.radius - vehicle.groundClearance;
        ImGui::SetNextWindowPos({ 12.0f, 12.0f }, ImGuiCond_Always);
        ImGui::Begin("Flight", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove);
        if (!mLaunched)
        {
            ImGui::Text("On the launch pad at %s", reference.name.c_str());
        }
        else if (mRocket.onGround())
        {
            ImGui::Text("On the ground at %s", reference.name.c_str());
        }
        else
        {
            ImGui::Text("Near %s", reference.name.c_str());
        }
        ImGui::Text("Altitude: %.1f km", altitude / 1000.0);
        ImGui::Text("Speed: %.0f m/s", mRocket.velocity().length());
        ImGui::Text("Mission time: %.0f s", mTime);

        ImGui::SeparatorText("Rocket");
        ImGui::Text("Mass: %.2f t   Height: %.1f m", vehicle.mass / 1000.0, vehicle.design.highestPoint() - vehicle.design.lowestPoint());
        ImGui::Text("Parts: %d   Stages: %d", vehicle.design.partCount(), vehicle.design.stageCount());
        ImGui::Text("Delta-v: %.0f m/s   Liftoff TWR: %.2f", mStats.totalDeltaVVacuum, mStats.liftoffTwr);
        ImGui::TextDisabled("Right-drag orbit, wheel zoom");
        ImGui::TextDisabled("Engines come next");

        // Once it's off the ground there's no going back to the building
        if (mRocket.onGround() || !mLaunched)
        {
            if (ImGui::Button("Back to assembly"))
            {
                requestScene(SceneId::Build);
            }
        }
        if (!mLaunched)
        {
            ImGui::SameLine();
            if (ImGui::Button("Launch"))
            {
                mLaunched = true;
            }
        }
        ImGui::End();
    }

    void FlightScene::onShutdown()
    {
        if (mSkyReady)
        {
            mSky.unload();
            mSkyReady = false;
        }
        if (mGraphicsReady)
        {
            UnloadModel(mEarthModel);
            UnloadModel(mGroundModel);
            mGraphicsReady = false;
        }
    }
}
