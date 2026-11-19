#include "flight/FlightScene.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <map>
#include <numbers>
#include <string>

#include <imgui.h>
#include <raymath.h>
#include <rlgl.h>

#include "flight/Aerodynamics.h"
#include "flight/Atmosphere.h"
#include "flight/PartDrawing.h"
#include "flight/StageSymbol.h"

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

        constexpr Color FLAME_COLOR = { 255, 170, 60, 200 };
        constexpr Color FLAME_CORE_COLOR = { 255, 240, 200, 230 };
        constexpr float FLAME_LENGTH = 6.0f; // metres at full throttle in vacuum, per metre of nozzle width
        constexpr Color CANOPY_COLOR = { 240, 120, 40, 255 };
        constexpr Color CANOPY_STRIPE_COLOR = { 245, 245, 240, 255 };
        constexpr Color DAY_ZENITH = { 70, 130, 215, 255 };
        constexpr Color DAY_HORIZON = { 190, 215, 240, 255 };
        constexpr Color SPACE_COLOR = { 4, 6, 14, 255 };
        constexpr Color SPACE_HORIZON_COLOR = { 30, 50, 90, 255 };
        constexpr Color FLASH_COLOR = { 255, 240, 190, 255 };
        constexpr Color SPARK_COLOR = { 255, 190, 80, 255 };
        constexpr Color SMOKE_COLOR = { 200, 200, 205, 255 };
        constexpr int SPARK_COUNT = 30; // for a decoupler; smaller charges make fewer
        constexpr int SMOKE_COUNT = 14;
        constexpr float CAMERA_EASE = 2.5f; // per second: how quickly the camera closes the gap to its new framing
        constexpr double DEBRIS_SETTLE_TIME = 8.0; // seconds lying still before a dropped piece is cleared away
        constexpr double DEBRIS_RANGE = 40.0e3; // metres from the rocket past which a dropped piece is forgotten
        constexpr double THROTTLE_RATE = 0.5; // share of full throttle per second while Shift or Ctrl is held

        constexpr float HUD_MARGIN = 12.0f;
        constexpr ImU32 HUD_PANEL = IM_COL32(14, 20, 28, 225);
        constexpr ImU32 HUD_EDGE = IM_COL32(70, 92, 112, 255);
        constexpr ImU32 HUD_DIM = IM_COL32(44, 54, 64, 255);
        constexpr ImU32 HUD_TEXT = IM_COL32(232, 236, 240, 255);
        constexpr ImU32 HUD_MUTED = IM_COL32(140, 152, 164, 255);
        constexpr ImU32 HUD_GREEN = IM_COL32(60, 220, 100, 255);
        constexpr ImU32 HUD_DARK_GREEN = IM_COL32(20, 70, 36, 255);
        constexpr ImU32 HUD_YELLOW = IM_COL32(240, 200, 60, 255);
        constexpr ImU32 HUD_PINK = IM_COL32(230, 90, 200, 255);
        constexpr ImU32 HUD_CYAN = IM_COL32(80, 200, 230, 255);
        constexpr ImU32 HUD_ON_GREEN = IM_COL32(10, 16, 12, 255);
        constexpr ImGuiWindowFlags OVERLAY_FLAGS = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
                                                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                                                   ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;

        constexpr double NEAR_PLANE = 0.1;
        constexpr double FAR_PLANE = 5.0e6; // past the far side of Earth from low orbit

        constexpr int GROUND_RINGS = 36;
        constexpr int GROUND_SEGMENTS = 96;
        constexpr double GROUND_FIRST_RING = 2.0; // metres
        constexpr double GROUND_RING_GROWTH = 1.35;

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

        ImVec2 offset(ImVec2 a, float x, float y)
        {
            return { a.x + x, a.y + y };
        }

        void text(ImDrawList* pList, ImVec2 at, float size, ImU32 color, const char* value)
        {
            pList->AddText(ImGui::GetFont(), size, at, color, value);
        }

        // Text whose right edge sits at x
        void textRight(ImDrawList* pList, float x, float y, float size, ImU32 color, const char* value)
        {
            const float width = ImGui::GetFont()->CalcTextSizeA(size, FLT_MAX, 0.0f, value).x;
            text(pList, { x - width, y }, size, color, value);
        }

        // A labelled number in a dark box with a coloured edge, like the speed and altitude readouts
        void readout(ImDrawList* pList, ImVec2 min, ImVec2 size, ImU32 accent, const char* label, const char* value, const char* unit)
        {
            const ImVec2 max = offset(min, size.x, size.y);
            pList->AddRectFilled(min, max, IM_COL32(8, 12, 18, 235), 4.0f);
            pList->AddRect(min, max, accent, 4.0f, 0, 1.5f);
            pList->AddRectFilled(min, offset(min, 4.0f, size.y), accent, 4.0f, ImDrawFlags_RoundCornersLeft);
            text(pList, offset(min, 10.0f, 4.0f), 12.0f, accent, label);
            textRight(pList, max.x - 34.0f, min.y + 18.0f, 24.0f, HUD_TEXT, value);
            text(pList, { max.x - 30.0f, min.y + 28.0f }, 12.0f, HUD_MUTED, unit);
        }

        // A fuel gauge: green fill for what's left, with the tonnes written over it
        void fuelBar(ImDrawList* pList, ImVec2 min, ImVec2 size, double left, double capacity)
        {
            const ImVec2 max = offset(min, size.x, size.y);
            const float share = capacity > 0.0 ? (float)std::clamp(left / capacity, 0.0, 1.0) : 0.0f;
            pList->AddRectFilled(min, max, HUD_DIM, 2.0f);
            if (share > 0.0f)
            {
                pList->AddRectFilled(min, { min.x + size.x * share, max.y }, share > 0.15f ? HUD_GREEN : HUD_YELLOW, 2.0f);
            }
            char value[32];
            std::snprintf(value, sizeof(value), "%.2f / %.2f t", left / 1000.0, capacity / 1000.0);
            // A dark shadow keeps the white text readable over both the fill and the empty part
            for (int pass = 0; pass < 2; pass++)
            {
                const float shift = pass == 0 ? 1.0f : 0.0f;
                const ImU32 color = pass == 0 ? IM_COL32(0, 0, 0, 200) : HUD_TEXT;
                text(pList, offset(min, 6.0f + shift, 2.0f + shift), 13.0f, color, "FUEL");
                textRight(pList, max.x - 6.0f + shift, min.y + 2.0f + shift, 13.0f, color, value);
            }
        }

        // Thousands split with commas, as in 5,724
        std::string grouped(double value)
        {
            std::string digits = std::to_string((long long)std::llround(std::max(0.0, value)));
            for (int i = (int)digits.size() - 3; i > 0; i -= 3)
            {
                digits.insert((size_t)i, ",");
            }
            return digits;
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
        mDebris.clear();
        mPuffs.clear();
        mRocket.setVehicle(vehicle);
        mRocket.place(earth, { radius + vehicle.groundClearance, 0.0, 0.0 }, {});
        mPad = SurfaceFrame::below(mRocket.position(), radius);

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
        mZoomTarget = 0.0f;
        mFramedHeight = height;
        mSnapCamera = true;
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
        collectDropped();
        updateDebris(dt);
        updateEffects(dt);

        // Every piece can knock into the rocket and into every other piece
        for (size_t i = 0; i < mDebris.size(); i++)
        {
            collide(mRocket, mDebris[i].rocket, dt);
            for (size_t j = i + 1; j < mDebris.size(); j++)
            {
                collide(mDebris[i].rocket, mDebris[j].rocket, dt);
            }
        }
        mTime += dt;
    }

    void FlightScene::collectDropped()
    {
        for (const Separation& separation : mRocket.takeSeparations())
        {
            burst(separation);

            // The scene is centred on the rocket, so the camera steps back by the jump to stay where it was
            const Vector3 jump = toRaylib(mPad.rotate(separation.shift));
            mFocus = { mFocus.x - jump.x, mFocus.y - jump.y, mFocus.z - jump.z };
        }

        // A shorter rocket is framed closer, the same way rollout frames it
        const RocketDesign& design = mRocket.vehicle().design;
        const float height = (float)(design.highestPoint() - design.lowestPoint());
        if (height != mFramedHeight && mFramedHeight > 0.0f)
        {
            const float target = mZoomTarget > 0.0f ? mZoomTarget : mOrbit.distance;
            mZoomTarget = target * std::max(14.0f, height * 1.8f) / std::max(14.0f, mFramedHeight * 1.8f);
            mFramedHeight = height;
        }
        for (Rocket& piece : mRocket.takeDropped())
        {
            mDebris.push_back({ std::move(piece), 0.0 });
        }
    }

    // A ring of sparks and smoke thrown out sideways from the joint, around a quick flash
    void FlightScene::burst(const Separation& separation)
    {
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        auto between = [&](double low, double high)
        {
            return low + (high - low) * unit(mRandom);
        };

        // Two directions square to the axis, for throwing things out around it
        const Vec3d axis = separation.axis.lengthSquared() > 0.0 ? separation.axis : Vec3d{ 0.0, 1.0, 0.0 };
        const Vec3d helper = std::abs(axis.x) < 0.9 ? Vec3d{ 1.0, 0.0, 0.0 } : Vec3d{ 0.0, 1.0, 0.0 };
        const Vec3d side = axis.cross(helper).normalized();
        const Vec3d other = axis.cross(side);
        auto outward = [&](double lift)
        {
            const double angle = between(0.0, 2.0 * std::numbers::pi);
            return (side * std::cos(angle) + other * std::sin(angle) + axis * between(-lift, lift)).normalized();
        };

        const double size = separation.size;
        Puff flash;
        flash.position = separation.site;
        flash.velocity = separation.velocity;
        flash.drift = separation.velocity;
        flash.life = 0.18;
        flash.radius = (float)(0.5 * size);
        flash.growth = (float)(14.0 * size);
        flash.color = FLASH_COLOR;
        mPuffs.push_back(flash);

        for (int i = 0; i < (int)(SPARK_COUNT * size); i++)
        {
            Puff spark;
            spark.position = separation.site;
            spark.velocity = separation.velocity + outward(0.3) * between(15.0, 35.0) * size;
            spark.drift = separation.velocity;
            spark.life = between(0.3, 0.7);
            spark.fade = 1.5;
            spark.radius = (float)between(0.06, 0.12);
            spark.color = SPARK_COLOR;
            mPuffs.push_back(spark);
        }
        for (int i = 0; i < (int)(SMOKE_COUNT * size); i++)
        {
            Puff smoke;
            smoke.position = separation.site;
            smoke.velocity = separation.velocity + outward(0.5) * between(2.0, 6.0) * size;
            smoke.drift = separation.velocity;
            smoke.life = between(1.2, 2.4);
            smoke.fade = 2.5;
            smoke.radius = (float)(between(0.3, 0.5) * size);
            smoke.growth = (float)(between(0.8, 1.6) * size);
            smoke.color = SMOKE_COLOR;
            mPuffs.push_back(smoke);
        }
    }

    // Puffs fall with the pieces they came off, so the smoke stays where the split happened
    void FlightScene::updateEffects(double dt)
    {
        const double mu = mSolarSystem.body(mRocket.reference()).mu;
        for (Puff& puff : mPuffs)
        {
            const double distanceSquared = puff.position.lengthSquared();
            const Vec3d gravity = distanceSquared > 0.0 ? puff.position.normalized() * (-mu / distanceSquared) : Vec3d{};
            puff.drift += gravity * dt;
            puff.velocity = puff.drift + (puff.velocity + gravity * dt - puff.drift) * std::exp(-puff.fade * dt);
            puff.position += puff.velocity * dt;
            puff.age += dt;
        }
        std::erase_if(mPuffs,
                      [](const Puff& puff)
                      {
                          return puff.age >= puff.life;
                      });
    }

    // Drawn see-through and without writing depth, so overlapping puffs blend instead of cutting into each other
    void FlightScene::drawEffects() const
    {
        if (mPuffs.empty())
        {
            return;
        }
        rlDrawRenderBatchActive();
        rlDisableDepthMask();
        for (const Puff& puff : mPuffs)
        {
            const float left = 1.0f - (float)(puff.age / puff.life);
            const float radius = puff.radius + puff.growth * (float)puff.age;
            DrawSphereEx(toScene(puff.position), radius, 6, 8, Fade(puff.color, left * (puff.color.a / 255.0f)));
        }
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
    }

    void FlightScene::updateDebris(double dt)
    {
        for (Debris& debris : mDebris)
        {
            debris.rocket.step(dt, mSolarSystem, mTime);
            const bool still = debris.rocket.onGround() && debris.rocket.velocity().length() < 0.5;
            debris.settledTime = still ? debris.settledTime + dt : 0.0;
        }
        std::erase_if(mDebris,
                      [this](const Debris& debris)
                      {
                          const bool far = debris.rocket.reference() != mRocket.reference() ||
                                           (debris.rocket.position() - mRocket.position()).length() > DEBRIS_RANGE;
                          return far || debris.settledTime > DEBRIS_SETTLE_TIME;
                      });
    }

    // The camera circles the middle of the rocket and never dips below the ground under it
    void FlightScene::onUpdate(double frameDt, double /*alpha*/)
    {
        // Shift and Ctrl ease the throttle up and down; Z opens it fully and X cuts it
        if (ImGui::GetCurrentContext() == nullptr || !ImGui::GetIO().WantCaptureKeyboard)
        {
            const double ease = (double)(IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) -
                                (double)(IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL));
            mRocket.setThrottle(mRocket.throttle() + ease * THROTTLE_RATE * frameDt);
            if (IsKeyPressed(KEY_Z))
            {
                mRocket.setThrottle(1.0);
            }
            if (IsKeyPressed(KEY_X))
            {
                mRocket.setThrottle(0.0);
            }
            if (IsKeyPressed(KEY_SPACE))
            {
                stage();
            }
        }

        const Vehicle& vehicle = mRocket.vehicle();
        const double middle = (vehicle.design.highestPoint() + vehicle.design.lowestPoint()) * 0.5;
        const Vector3 target = toRaylib(mPad.rotate(mRocket.toWorld({ 0.0, middle - vehicle.centerOfMass.y, 0.0 })));
        const float follow = mSnapCamera ? 1.0f : 1.0f - std::exp(-CAMERA_EASE * (float)frameDt);
        mSnapCamera = false;
        mFocus = { mFocus.x + (target.x - mFocus.x) * follow, mFocus.y + (target.y - mFocus.y) * follow,
                   mFocus.z + (target.z - mFocus.z) * follow };
        mOrbit.focus = mFocus;
        mOrbit.minFocusHeight = mFocus.y;
        mOrbit.maxFocusHeight = mFocus.y;
        mOrbit.minCameraHeight = toScene(mPad.origin).y + 0.5f;
        if (mZoomTarget > 0.0f)
        {
            mOrbit.distance += (mZoomTarget - mOrbit.distance) * follow;
        }
        const float easedDistance = mOrbit.distance;
        mOrbit.update(mCamera, (float)frameDt);

        // Scrolling while it eases moves where it's heading too, so the wheel still works
        if (mZoomTarget > 0.0f)
        {
            mZoomTarget *= mOrbit.distance / easedDistance;
            if (std::abs(mZoomTarget - mOrbit.distance) < 0.01f * mZoomTarget)
            {
                mZoomTarget = 0.0f;
            }
        }
    }

    void FlightScene::stage()
    {
        mLaunched = true;
        mRocket.fireNextStage();
        collectDropped();
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
            // The blue thins to black as the air does, a little slower so the horizon keeps a glow near the top
            const BodyInfo& body = mSolarSystem.body(mRocket.reference());
            const float air = (float)std::pow(pressureRatio(body, mRocket.altitude(mSolarSystem)), 0.35);
            mSky.dayZenith = ColorLerp(SPACE_COLOR, DAY_ZENITH, air);
            mSky.dayHorizon = ColorLerp(SPACE_HORIZON_COLOR, DAY_HORIZON, air);
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

        drawRocket(mRocket);
        for (const Debris& debris : mDebris)
        {
            drawRocket(debris.rocket);
        }
        drawEffects();
    }

    // The flying rocket's center of mass is the scene's origin, so pieces are drawn where they sit from it
    void FlightScene::drawRocket(const Rocket& rocket) const
    {
        const Vector3 x = toRaylib(mPad.rotate(rocket.toWorld({ 1.0, 0.0, 0.0 })));
        const Vector3 y = toRaylib(mPad.rotate(rocket.toWorld({ 0.0, 1.0, 0.0 })));
        const Vector3 z = toRaylib(mPad.rotate(rocket.toWorld({ 0.0, 0.0, 1.0 })));
        const Vector3 at = toScene(rocket.position());
        const Matrix facing = { x.x, y.x, z.x, at.x, x.y, y.y, z.y, at.y, x.z, y.z, z.z, at.z, 0.0f, 0.0f, 0.0f, 1.0f };
        rlPushMatrix();
        rlMultMatrixf(MatrixToFloat(facing));
        drawDesign(rocket.vehicle().design, toRaylib(-rocket.vehicle().centerOfMass));
        drawFlames(rocket);
        drawParachutes(rocket);
        rlPopMatrix();
    }

    // A cone of exhaust under each burning engine, longer with more throttle and in thinner air. Drawn in the
    // rocket's own axes.
    void FlightScene::drawFlames(const Rocket& rocket) const
    {
        const Vehicle& vehicle = rocket.vehicle();
        if (vehicle.throttle <= 0.0)
        {
            return;
        }
        for (PartIndex i = 0; i < vehicle.design.partCount(); i++)
        {
            if (!isBurning(vehicle, i))
            {
                continue;
            }
            const PlacedPart& placed = vehicle.design.part(i);
            const Vec3d nozzle = placed.position - vehicle.centerOfMass - Vec3d{ 0.0, placed.shape.height * 0.5, 0.0 };
            const float width = (float)placed.shape.bottomDiameter;
            const float spread = (float)(1.0 - rocket.pressure() * 0.4);
            const float length = width * FLAME_LENGTH * (float)vehicle.throttle * spread;
            const Vector3 top = toRaylib(nozzle);
            DrawCylinderEx(top, { top.x, top.y - length, top.z }, width * 0.45f, 0.0f, 12, FLAME_COLOR);
            DrawCylinderEx(top, { top.x, top.y - length * 0.5f, top.z }, width * 0.3f, 0.0f, 12, FLAME_CORE_COLOR);
        }
    }

    // An open parachute flies above its part on lines from the part's top. Drawn in the rocket's own axes.
    void FlightScene::drawParachutes(const Rocket& rocket) const
    {
        const Vehicle& vehicle = rocket.vehicle();
        for (PartIndex i = 0; i < vehicle.design.partCount(); i++)
        {
            if (vehicle.canopy[i] <= 0.0)
            {
                continue;
            }

            // It streams out behind sideways motion but never hangs below the pod: while still climbing it stands
            // straight up. It billows as it fills, its width growing with the square root of the share of its area.
            const PlacedPart& placed = vehicle.design.part(i);
            const Vec3d anchor = placed.position - vehicle.centerOfMass + Vec3d{ 0.0, placed.shape.height * 0.5, 0.0 };
            const Vec3d up = rocket.position().normalized();
            const double speed = rocket.velocity().length();
            Vec3d trailing = speed > 0.0 ? rocket.velocity() * (-1.0 / std::max(speed, 5.0)) : Vec3d{};
            trailing -= up * std::min(0.0, trailing.dot(up));
            const Vec3d out = rocket.toLocal((trailing + up * 0.5).normalized());
            const Vec3d helper = std::abs(out.x) < 0.9 ? Vec3d{ 1.0, 0.0, 0.0 } : Vec3d{ 0.0, 0.0, 1.0 };
            const Vec3d side = out.cross(helper).normalized();
            const Vec3d other = out.cross(side);
            const double radius = PARACHUTE_DIAMETER * 0.5 * std::sqrt(vehicle.canopy[i]);
            const Vec3d rim = anchor + out * PARACHUTE_RISE;
            const Vec3d crown = rim + out * (radius * 0.35);
            DrawCylinderEx(toRaylib(rim), toRaylib(crown), (float)radius, (float)(radius * 0.75), 16, CANOPY_COLOR);
            DrawCylinderEx(toRaylib(crown), toRaylib(crown + out * (radius * 0.25)), (float)(radius * 0.75), (float)(radius * 0.2), 16,
                           CANOPY_STRIPE_COLOR);
            for (int line = 0; line < 8; line++)
            {
                const double angle = 2.0 * std::numbers::pi * line / 8.0;
                const Vec3d edge = rim + (side * std::cos(angle) + other * std::sin(angle)) * radius;
                DrawLine3D(toRaylib(anchor), toRaylib(edge), CANOPY_STRIPE_COLOR);
            }
        }
    }

    void FlightScene::onGui()
    {
        mStats = computeStats(mRocket.vehicle(), mSolarSystem.surfaceGravity(mRocket.reference()));
        drawStatusBar();
        drawGauges();
        drawStageStack();
    }

    // Where the rocket is, the mission clock, and the way back to the building while it's still on the ground
    void FlightScene::drawStatusBar()
    {
        const BodyInfo& reference = mSolarSystem.body(mRocket.reference());
        ImGui::SetNextWindowPos({ HUD_MARGIN, HUD_MARGIN }, ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.88f);
        ImGui::Begin("Mission", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoFocusOnAppearing);
        const char* status = !mLaunched ? "On the launch pad" : mRocket.onGround() ? "On the ground" : "Flying";
        const int seconds = (int)mTime;
        ImGui::Text("%s   %s   T+ %02d:%02d:%02d", reference.name.c_str(), status, seconds / 3600, seconds / 60 % 60, seconds % 60);
        if (mRocket.onGround() || !mLaunched)
        {
            ImGui::SameLine();
            if (ImGui::SmallButton("Back to assembly"))
            {
                requestScene(SceneId::Build);
            }
        }
        ImGui::TextDisabled("Space stage   Shift/Ctrl throttle   Z full   X cut   Right-drag look");
        ImGui::End();
    }

    // Bottom left: the throttle as an arc, with speed, altitude, and climb rate beside it
    void FlightScene::drawGauges()
    {
        const ImVec2 size = { 370.0f, 190.0f };
        const ImVec2 min = { HUD_MARGIN, (float)GetScreenHeight() - size.y - HUD_MARGIN };
        ImGui::SetNextWindowPos(min, ImGuiCond_Always);
        ImGui::SetNextWindowSize(size, ImGuiCond_Always);
        ImGui::Begin("Gauges", nullptr, OVERLAY_FLAGS | ImGuiWindowFlags_NoInputs);
        ImDrawList* pList = ImGui::GetWindowDrawList();
        pList->AddRectFilled(min, offset(min, size.x, size.y), HUD_PANEL, 10.0f);
        pList->AddRect(min, offset(min, size.x, size.y), HUD_EDGE, 10.0f, 0, 1.5f);

        // The arc runs up the left side, filling from the bottom as the throttle opens
        const ImVec2 centre = offset(min, 100.0f, 95.0f);
        const float radius = 70.0f;
        const float start = PI * 0.62f;
        const float end = PI * 1.38f;
        const float throttle = (float)mRocket.throttle();
        pList->PathArcTo(centre, radius, start, end, 40);
        pList->PathStroke(HUD_DIM, 0, 16.0f);
        if (throttle > 0.0f)
        {
            pList->PathArcTo(centre, radius, start, start + (end - start) * throttle, 40);
            pList->PathStroke(HUD_GREEN, 0, 16.0f);
        }
        for (int tick = 0; tick <= 4; tick++)
        {
            const float angle = start + (end - start) * tick / 4.0f;
            const float c = std::cos(angle);
            const float s = std::sin(angle);
            pList->AddLine(offset(centre, c * (radius + 10.0f), s * (radius + 10.0f)), offset(centre, c * (radius + 16.0f), s * (radius + 16.0f)),
                           HUD_MUTED, 2.0f);
        }
        char value[32];
        std::snprintf(value, sizeof(value), "%d%%", (int)std::lround(throttle * 100.0f));
        textRight(pList, centre.x + 50.0f, centre.y - 26.0f, 32.0f, HUD_TEXT, value);
        text(pList, offset(centre, -16.0f, 10.0f), 12.0f, HUD_GREEN, "THROTTLE");
        std::snprintf(value, sizeof(value), "%.0f kN", mRocket.thrust() / 1000.0);
        text(pList, offset(centre, -16.0f, 26.0f), 14.0f, HUD_MUTED, value);

        const Vec3d up = mRocket.position().normalized();
        const double altitude = std::max(0.0, mRocket.altitude(mSolarSystem));
        const ImVec2 box = { 170.0f, 50.0f };
        const float left = min.x + size.x - box.x - 12.0f;
        std::snprintf(value, sizeof(value), "%.1f", mRocket.velocity().length());
        readout(pList, { left, min.y + 12.0f }, box, HUD_YELLOW, "SURFACE SPEED", value, "m/s");
        if (altitude < 10000.0)
        {
            std::snprintf(value, sizeof(value), "%.0f", altitude);
            readout(pList, { left, min.y + 70.0f }, box, HUD_PINK, "ALTITUDE", value, "m");
        }
        else
        {
            std::snprintf(value, sizeof(value), "%.2f", altitude / 1000.0);
            readout(pList, { left, min.y + 70.0f }, box, HUD_PINK, "ALTITUDE", value, "km");
        }
        std::snprintf(value, sizeof(value), "%+.1f", mRocket.velocity().dot(up));
        readout(pList, { left, min.y + 128.0f }, box, HUD_CYAN, "VERTICAL SPEED", value, "m/s");
        ImGui::End();
    }

    // Bottom right: the stages still to fire stacked upward, the one burning or about to burn opened out with its
    // engines and fuel, and the GO button that fires the next stage
    void FlightScene::drawStageStack()
    {
        const Vehicle& vehicle = mRocket.vehicle();
        const RocketDesign& design = vehicle.design;
        // The stage opened out is the latest one with an engine still burning, or else the next to fire
        int burningStage = -1;
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            if (isBurning(vehicle, i))
            {
                burningStage = std::max(burningStage, design.part(i).stage);
            }
        }
        const bool burning = burningStage >= 0;
        const int current = burning ? burningStage : vehicle.nextStage;
        const float width = 240.0f;
        const float right = (float)GetScreenWidth() - HUD_MARGIN;
        float bottom = (float)GetScreenHeight() - HUD_MARGIN;
        ImGui::SetNextWindowPos({ 0.0f, 0.0f }, ImGuiCond_Always);
        ImGui::SetNextWindowSize({ (float)GetScreenWidth(), (float)GetScreenHeight() }, ImGuiCond_Always);
        ImGui::Begin("Stages", nullptr, OVERLAY_FLAGS | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoInputs);
        ImDrawList* pList = ImGui::GetWindowDrawList();
        char value[64];

        // GO is lit green while there's a stage left to fire
        const bool canStage = vehicle.nextStage < design.stageCount();
        const ImVec2 goMin = { right - width, bottom - 70.0f };
        const ImVec2 goMax = { right, bottom };
        pList->AddRectFilled(goMin, goMax, canStage ? HUD_DARK_GREEN : HUD_PANEL, 6.0f);
        pList->AddRect(goMin, goMax, canStage ? HUD_GREEN : HUD_EDGE, 6.0f, 0, 2.0f);
        text(pList, offset(goMin, 12.0f, 4.0f), 34.0f, canStage ? HUD_GREEN : HUD_MUTED, mLaunched ? "GO" : "LAUNCH");
        std::snprintf(value, sizeof(value), "%d", design.stageCount() - vehicle.nextStage);
        pList->AddRectFilled({ goMax.x - 44.0f, goMin.y + 8.0f }, { goMax.x - 8.0f, goMin.y + 38.0f }, canStage ? HUD_GREEN : HUD_DIM, 4.0f);
        textRight(pList, goMax.x - 16.0f, goMin.y + 10.0f, 26.0f, HUD_ON_GREEN, value);
        text(pList, { goMin.x + 12.0f, goMax.y - 22.0f }, 14.0f, HUD_MUTED, "dV");
        std::snprintf(value, sizeof(value), "%s m/s", grouped(mStats.totalDeltaVVacuum).c_str());
        textRight(pList, goMax.x - 10.0f, goMax.y - 25.0f, 19.0f, HUD_TEXT, value);
        bottom = goMin.y - 8.0f;

        // The current stage, opened out: its delta-v, what it fires, and the fuel each engine group draws from
        if (current >= 0 && current < design.stageCount())
        {
            std::map<const PartDefinition*, int> kinds;
            std::vector<PartIndex> sections;
            for (PartIndex i : design.partsInStage(current))
            {
                kinds[design.part(i).pDefinition]++;
                if (design.part(i).pDefinition->isEngine() &&
                    std::find(sections.begin(), sections.end(), vehicle.sections[i]) == sections.end())
                {
                    sections.push_back(vehicle.sections[i]);
                }
            }
            const float height = 42.0f + kinds.size() * 30.0f + sections.size() * 24.0f + 6.0f;
            const ImVec2 panelMin = { right - width, bottom - height };
            pList->AddRectFilled(panelMin, { right, bottom }, HUD_PANEL, 6.0f);
            pList->AddRect(panelMin, { right, bottom }, burning ? HUD_GREEN : HUD_EDGE, 6.0f, 0, 1.5f);
            const double deltaV = current < (int)mStats.stages.size() ? mStats.stages[current].deltaVVacuum : 0.0;
            text(pList, offset(panelMin, 10.0f, 10.0f), 14.0f, HUD_MUTED, "dV");
            std::snprintf(value, sizeof(value), "%s m/s", grouped(deltaV).c_str());
            textRight(pList, right - 52.0f, panelMin.y + 6.0f, 22.0f, HUD_TEXT, value);
            std::snprintf(value, sizeof(value), "%02d", current + 1);
            textRight(pList, right - 10.0f, panelMin.y + 6.0f, 22.0f, burning ? HUD_GREEN : HUD_TEXT, value);

            float y = panelMin.y + 40.0f;
            for (const auto& [pDefinition, count] : kinds)
            {
                pList->AddRectFilled({ panelMin.x + 10.0f, y }, { panelMin.x + 36.0f, y + 26.0f }, HUD_DIM, 4.0f);
                drawStageSymbol(pList, { panelMin.x + 10.0f, y }, 26.0f, *pDefinition, HUD_TEXT);
                std::snprintf(value, sizeof(value), "%s (%d)", pDefinition->name.c_str(), count);
                text(pList, { panelMin.x + 44.0f, y + 5.0f }, 15.0f, HUD_TEXT, value);
                y += 30.0f;
            }
            for (PartIndex section : sections)
            {
                fuelBar(pList, { panelMin.x + 10.0f, y }, { width - 20.0f, 18.0f }, sectionFuel(vehicle, section), sectionCapacity(vehicle, section));
                y += 24.0f;
            }
            bottom = panelMin.y - 8.0f;
        }

        // The later stages as small numbered boxes, the last to fire at the top
        const float boxSize = 56.0f;
        for (int stageIndex = vehicle.nextStage + (burning ? 0 : 1); stageIndex < design.stageCount(); stageIndex++)
        {
            const ImVec2 boxMin = { right - boxSize, bottom - boxSize };
            pList->AddRectFilled(boxMin, { right, bottom }, HUD_PANEL, 6.0f);
            pList->AddRect(boxMin, { right, bottom }, HUD_EDGE, 6.0f, 0, 1.5f);
            std::snprintf(value, sizeof(value), "%02d", stageIndex + 1);
            text(pList, offset(boxMin, 8.0f, 3.0f), 16.0f, HUD_TEXT, value);
            const std::vector<PartIndex> parts = design.partsInStage(stageIndex);
            if (!parts.empty())
            {
                drawStageSymbol(pList, offset(boxMin, 16.0f, 22.0f), 30.0f, *design.part(parts.front()).pDefinition, HUD_TEXT);
            }
            bottom = boxMin.y - 6.0f;
        }
        ImGui::End();

        // Clicking GO stages, the same as Space
        ImGui::SetNextWindowPos(goMin, ImGuiCond_Always);
        ImGui::SetNextWindowSize({ width, 70.0f }, ImGuiCond_Always);
        ImGui::Begin("Go", nullptr, OVERLAY_FLAGS);
        ImGui::SetCursorPos({ 0.0f, 0.0f });
        if (ImGui::InvisibleButton("go", { width, 70.0f }) && canStage)
        {
            stage();
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
