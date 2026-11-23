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
        constexpr ImU32 HUD_YELLOW = IM_COL32(240, 200, 60, 255);
        constexpr ImU32 HUD_PINK = IM_COL32(230, 90, 200, 255);
        constexpr ImU32 HUD_CYAN = IM_COL32(80, 200, 230, 255);
        constexpr ImU32 HUD_BLUE = IM_COL32(90, 150, 255, 255);
        constexpr ImU32 HUD_ORANGE = IM_COL32(255, 150, 40, 255);
        constexpr ImU32 HUD_RED = IM_COL32(240, 90, 80, 255);
        constexpr ImU32 HUD_GO = IM_COL32(60, 210, 100, 255);
        constexpr ImU32 HUD_GO_EDGE = IM_COL32(150, 255, 170, 255);
        constexpr ImU32 HUD_GO_INK = IM_COL32(10, 70, 30, 255);
        constexpr ImU32 HUD_BOX = IM_COL32(8, 12, 18, 245);
        constexpr float NAVBALL_BEZEL = 106.0f; // the dark ring round the ball that carries the heading tape
        constexpr float ARC_RADIUS = 150.0f; // the throttle and climb rate arcs either side of the ball
        constexpr float ARC_WIDTH = 14.0f;
        constexpr ImVec2 ORBIT_STRIP_SIZE = { 340.0f, 50.0f };
        constexpr double G_FORCE_SMOOTHING = 4.0; // per second: how quickly the gauge follows the felt push
        constexpr ImVec4 NAVBALL_SKY = { 70.0f, 140.0f, 225.0f, 255.0f };
        constexpr ImVec4 NAVBALL_GROUND = { 165.0f, 105.0f, 55.0f, 255.0f };
        constexpr float NAVBALL_RADIUS = 78.0f;
        constexpr int NAVBALL_RINGS = 24;
        constexpr int NAVBALL_SEGMENTS = 72;
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

        // A labelled number boxed at the side of the instruments, like the speed and altitude
        void sideBox(ImDrawList* pList, ImVec2 centre, ImU32 accent, const char* label, const char* value, const char* unit)
        {
            const ImVec2 min = offset(centre, -42.0f, -26.0f);
            const ImVec2 max = offset(centre, 42.0f, 26.0f);
            pList->AddRectFilled(min, max, HUD_BOX, 3.0f);
            pList->AddRect(min, max, accent, 3.0f, 0, 1.5f);
            text(pList, offset(min, 6.0f, 3.0f), 11.0f, accent, label);
            textRight(pList, max.x - 6.0f, min.y + 15.0f, 22.0f, HUD_TEXT, value);
            textRight(pList, max.x - 6.0f, max.y - 13.0f, 11.0f, accent, unit);
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

        // Text centred on a point
        void textCentred(ImDrawList* pList, ImVec2 at, float size, ImU32 color, const char* value)
        {
            const ImVec2 extent = ImGui::GetFont()->CalcTextSizeA(size, FLT_MAX, 0.0f, value);
            text(pList, { at.x - extent.x * 0.5f, at.y - extent.y * 0.5f }, size, color, value);
        }

        // Hours, minutes, and seconds, as in 00:04:35
        std::string clockText(double seconds)
        {
            const long long total = (long long)std::max(0.0, seconds);
            char value[32];
            std::snprintf(value, sizeof(value), "%02lld:%02lld:%02lld", total / 3600, total / 60 % 60, total % 60);
            return value;
        }

        // A dark panel with an edge, the backing for every part of the HUD
        void panel(ImDrawList* pList, ImVec2 min, ImVec2 max, ImU32 edge)
        {
            pList->AddRectFilled(min, max, HUD_PANEL, 6.0f);
            pList->AddRect(min, max, edge, 6.0f, 0, 1.5f);
        }

        // A small name sitting along a panel's bottom edge
        void caption(ImDrawList* pList, ImVec2 min, ImVec2 max, const char* label)
        {
            textCentred(pList, { (min.x + max.x) * 0.5f, max.y - 7.0f }, 11.0f, HUD_CYAN, label);
        }

        // A height above the surface in kilometres, or metres while it's low
        void writeHeight(char* value, size_t size, double height, const char*& unit)
        {
            if (std::abs(height) < 10000.0)
            {
                std::snprintf(value, size, "%.0f", height);
                unit = "m";
            }
            else
            {
                std::snprintf(value, size, "%.1f", height / 1000.0);
                unit = "km";
            }
        }

        // The navball's colour for a direction this far above the horizon, darkened toward the ball's edge
        ImU32 navballColor(double elevation, double facing)
        {
            const float t = (float)std::clamp(elevation * 25.0 + 0.5, 0.0, 1.0);
            const float shade = 0.55f + 0.45f * (float)facing;
            auto mix = [&](float ground, float sky)
            {
                return (int)((ground + (sky - ground) * t) * shade);
            };
            return IM_COL32(mix(NAVBALL_GROUND.x, NAVBALL_SKY.x), mix(NAVBALL_GROUND.y, NAVBALL_SKY.y), mix(NAVBALL_GROUND.z, NAVBALL_SKY.z), 255);
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

        // The same, keeping a minus sign
        std::string signedGrouped(double value)
        {
            return std::string(value < 0.0 ? "-" : "") + grouped(std::abs(value));
        }
    }

    FlightScene::FlightScene(Camera3D& camera, const RocketDesign& design, const SolarSystem& solarSystem)
        : mCamera(camera), mDesign(design), mSolarSystem(solarSystem)
    {
        mOrbit.minPitch = -1.5f;
        mOrbit.liftWithKeys = false;
        mOrbit.maxDistance = 3000.0f;
    }

    // The rocket rolls out as built and stands on the pad at the equator with its tanks full
    void FlightScene::onEnter()
    {
        const BodyId earth = mSolarSystem.findBody("Earth");
        const double radius = mSolarSystem.body(earth).radius;
        const Vehicle vehicle = makeVehicle(mDesign);
        mTime = 0.0;
        mGForce = 1.0;
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
        const Vec3d before = mRocket.velocity();
        const BodyId reference = mRocket.reference();
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

        // What the crew feels is the change in velocity gravity alone wouldn't have made, along the rocket
        if (mRocket.reference() == reference)
        {
            const Vec3d position = mRocket.position();
            const double distance = position.length();
            const Vec3d gravity = position * (-mSolarSystem.body(reference).mu / (distance * distance * distance));
            const Vec3d felt = (mRocket.velocity() - before) / dt - gravity;
            const double gForce = felt.dot(mRocket.toWorld({ 0.0, 1.0, 0.0 })) / STANDARD_GRAVITY;
            mGForce += (gForce - mGForce) * std::min(1.0, dt * G_FORCE_SMOOTHING);
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

            // W and S pitch it, A and D yaw it, Q and E roll it
            auto axis = [](KeyboardKey positive, KeyboardKey negative)
            {
                return (double)IsKeyDown(positive) - (double)IsKeyDown(negative);
            };
            mRocket.setSteering({ axis(KEY_S, KEY_W), axis(KEY_Q, KEY_E), axis(KEY_A, KEY_D) });
        }
        else
        {
            mRocket.setSteering({});
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
        drawDesign(rocket.vehicle().design, toRaylib(-rocket.vehicle().centerOfMass), &rocket.vehicle().gimbal);
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
            const Vec3d back = -thrustDirection(vehicle, i);
            const Vec3d nozzle = gimbalPivot(vehicle, i) - vehicle.centerOfMass + back * placed.shape.height;
            const float width = (float)placed.shape.bottomDiameter;
            const float spread = (float)(1.0 - rocket.pressure() * 0.4);
            const float length = width * FLAME_LENGTH * (float)vehicle.throttle * spread;
            const Vector3 top = toRaylib(nozzle);
            DrawCylinderEx(top, toRaylib(nozzle + back * length), width * 0.45f, 0.0f, 12, FLAME_COLOR);
            DrawCylinderEx(top, toRaylib(nozzle + back * (length * 0.5)), width * 0.3f, 0.0f, 12, FLAME_CORE_COLOR);
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
        drawClock();
        drawInstruments();
        drawOrbit();
        drawGForce();
        drawStageStack();
    }

    // Top left: where the rocket is, from the body it's around out to the one that body orbits, and the way back
    // to the building while it's still on the ground
    void FlightScene::drawStatusBar()
    {
        const BodyInfo& reference = mSolarSystem.body(mRocket.reference());
        ImGui::SetNextWindowPos({ HUD_MARGIN, HUD_MARGIN }, ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.88f);
        ImGui::Begin("Mission", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoFocusOnAppearing);
        const char* status = !mLaunched ? "On the launch pad" : mRocket.onGround() ? "On the ground" : "Flying";
        if (reference.parent != NO_BODY)
        {
            ImGui::TextUnformatted(mSolarSystem.body(reference.parent).name.c_str());
            ImGui::SameLine();
            ImGui::TextDisabled("\\");
            ImGui::SameLine();
        }
        ImGui::TextUnformatted(reference.name.c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("\\");
        ImGui::SameLine();
        ImGui::TextUnformatted(status);
        if (mRocket.onGround() || !mLaunched)
        {
            ImGui::SameLine();
            if (ImGui::SmallButton("Back to assembly"))
            {
                requestScene(SceneId::Build);
            }
        }
        ImGui::TextDisabled("Space stage   WASD/QE steer   Shift/Ctrl throttle   Z full   X cut   Right-drag look");
        ImGui::End();
    }

    // Bottom middle: the mission clock, counting up from launch
    void FlightScene::drawClock()
    {
        const ImVec2 size = { 220.0f, 46.0f };
        const ImVec2 min = { ((float)GetScreenWidth() - size.x) * 0.5f, (float)GetScreenHeight() - size.y - HUD_MARGIN };
        const ImVec2 max = offset(min, size.x, size.y);
        ImGui::SetNextWindowPos(min, ImGuiCond_Always);
        ImGui::SetNextWindowSize(size, ImGuiCond_Always);
        ImGui::Begin("Clock", nullptr, OVERLAY_FLAGS | ImGuiWindowFlags_NoInputs);
        ImDrawList* pList = ImGui::GetWindowDrawList();
        panel(pList, min, max, mLaunched ? HUD_GREEN : HUD_EDGE);
        text(pList, offset(min, 12.0f, 8.0f), 16.0f, mLaunched ? HUD_GREEN : HUD_MUTED, "T+");
        textRight(pList, max.x - 12.0f, min.y + 4.0f, 26.0f, HUD_TEXT, clockText(mTime).c_str());
        caption(pList, min, max, "MISSION TIME");
        ImGui::End();
    }

    // Bottom left: the navball in a ring carrying the heading tape, the throttle arc to its left, the climb rate arc
    // to its right, and the speed and altitude boxed either side
    void FlightScene::drawInstruments()
    {
        const float radius = NAVBALL_RADIUS;
        const ImVec2 centre = { HUD_MARGIN + ARC_RADIUS + ARC_WIDTH,
                                (float)GetScreenHeight() - HUD_MARGIN - ORBIT_STRIP_SIZE.y - 8.0f - ARC_RADIUS * 0.85f };
        ImGui::SetNextWindowPos({ 0.0f, 0.0f }, ImGuiCond_Always);
        ImGui::SetNextWindowSize({ (float)GetScreenWidth(), (float)GetScreenHeight() }, ImGuiCond_Always);
        ImGui::Begin("Instruments", nullptr, OVERLAY_FLAGS | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoInputs);
        ImDrawList* pList = ImGui::GetWindowDrawList();
        pList->AddCircleFilled(centre, NAVBALL_BEZEL, HUD_PANEL, 96);
        pList->AddCircle(centre, NAVBALL_BEZEL, HUD_EDGE, 96, 1.5f);

        const SurfaceFrame here = SurfaceFrame::below(mRocket.position(), 1.0);
        const Vec3d north = -here.south;
        const double degree = std::numbers::pi / 180.0;

        // The nose is the middle of the ball. Across and down it are the rocket's own +Z and +X, the way a pilot
        // facing up the nose would see them.
        auto onBall = [&](const Vec3d& direction, ImVec2& at)
        {
            const Vec3d local = mRocket.toLocal(direction);
            at = offset(centre, (float)local.z * radius, (float)local.x * radius);
            return local.y >= 0.0;
        };
        auto direction = [&](double heading, double elevation)
        {
            const Vec3d level = north * std::cos(heading) + here.east * std::sin(heading);
            return level * std::cos(elevation) + here.up * std::sin(elevation);
        };

        // The ball itself, as rings of points round the middle, each coloured by whether it looks at sky or ground
        const ImVec2 white = ImGui::GetFontTexUvWhitePixel();
        const int vertices = 1 + NAVBALL_RINGS * NAVBALL_SEGMENTS;
        const int triangles = NAVBALL_SEGMENTS + (NAVBALL_RINGS - 1) * NAVBALL_SEGMENTS * 2;
        pList->PrimReserve(triangles * 3, vertices);
        const unsigned int base = pList->_VtxCurrentIdx;
        auto writePoint = [&](double across, double down)
        {
            const double facing = std::sqrt(std::max(0.0, 1.0 - across * across - down * down));
            const Vec3d look = mRocket.toWorld({ down, facing, across });
            pList->PrimWriteVtx(offset(centre, (float)across * radius, (float)down * radius), white, navballColor(look.dot(here.up), facing));
        };
        writePoint(0.0, 0.0);
        for (int ring = 1; ring <= NAVBALL_RINGS; ring++)
        {
            const double reach = (double)ring / NAVBALL_RINGS;
            for (int segment = 0; segment < NAVBALL_SEGMENTS; segment++)
            {
                const double angle = 2.0 * std::numbers::pi * segment / NAVBALL_SEGMENTS;
                writePoint(reach * std::cos(angle), reach * std::sin(angle));
            }
        }
        auto ringPoint = [&](int ring, int segment)
        {
            return (ImDrawIdx)(base + 1 + (ring - 1) * NAVBALL_SEGMENTS + segment % NAVBALL_SEGMENTS);
        };
        for (int segment = 0; segment < NAVBALL_SEGMENTS; segment++)
        {
            pList->PrimWriteIdx((ImDrawIdx)base);
            pList->PrimWriteIdx(ringPoint(1, segment));
            pList->PrimWriteIdx(ringPoint(1, segment + 1));
        }
        for (int ring = 1; ring < NAVBALL_RINGS; ring++)
        {
            for (int segment = 0; segment < NAVBALL_SEGMENTS; segment++)
            {
                pList->PrimWriteIdx(ringPoint(ring, segment));
                pList->PrimWriteIdx(ringPoint(ring + 1, segment));
                pList->PrimWriteIdx(ringPoint(ring + 1, segment + 1));
                pList->PrimWriteIdx(ringPoint(ring, segment));
                pList->PrimWriteIdx(ringPoint(ring + 1, segment + 1));
                pList->PrimWriteIdx(ringPoint(ring, segment + 1));
            }
        }

        // Lines across the ball: the horizon, rings every 30 degrees above and below it, and the compass headings
        auto stroke = [&](auto along, ImU32 color, float thickness)
        {
            ImVec2 previous;
            bool previousShown = false;
            for (int step = 0; step <= 96; step++)
            {
                ImVec2 at;
                const bool shown = onBall(along(step / 96.0), at);
                if (shown && previousShown)
                {
                    pList->AddLine(previous, at, color, thickness);
                }
                previous = at;
                previousShown = shown;
            }
        };
        for (int elevation = -60; elevation <= 60; elevation += 30)
        {
            const ImU32 color = elevation == 0 ? HUD_TEXT : IM_COL32(255, 255, 255, 90);
            stroke([&](double t) { return direction(t * 2.0 * std::numbers::pi, elevation * degree); }, color, elevation == 0 ? 2.0f : 1.0f);
        }
        for (int heading = 0; heading < 360; heading += 45)
        {
            stroke([&](double t) { return direction(heading * degree, (t * 160.0 - 80.0) * degree); }, IM_COL32(255, 255, 255, 60), 1.0f);
        }
        const char* compass[] = { "N", "E", "S", "W" };
        for (int i = 0; i < 4; i++)
        {
            ImVec2 at;
            if (onBall(direction(i * 90.0 * degree, 0.0), at))
            {
                textCentred(pList, offset(at, 0.0f, -9.0f), 15.0f, HUD_TEXT, compass[i]);
            }
        }

        // Prograde is a circle with three ticks, where it's moving; retrograde, a crossed circle opposite
        const double speed = mRocket.velocity().length();
        ImVec2 at;
        if (speed > 0.5 && onBall(mRocket.velocity() / speed, at))
        {
            pList->AddCircle(at, 7.0f, HUD_YELLOW, 20, 2.0f);
            pList->AddLine(offset(at, 0.0f, -7.0f), offset(at, 0.0f, -14.0f), HUD_YELLOW, 2.0f);
            pList->AddLine(offset(at, -7.0f, 0.0f), offset(at, -14.0f, 0.0f), HUD_YELLOW, 2.0f);
            pList->AddLine(offset(at, 7.0f, 0.0f), offset(at, 14.0f, 0.0f), HUD_YELLOW, 2.0f);
            pList->AddCircleFilled(at, 1.5f, HUD_YELLOW);
        }
        if (speed > 0.5 && onBall(mRocket.velocity() / -speed, at))
        {
            pList->AddCircle(at, 7.0f, HUD_YELLOW, 20, 2.0f);
            pList->AddLine(offset(at, -5.0f, -5.0f), offset(at, 5.0f, 5.0f), HUD_YELLOW, 2.0f);
            pList->AddLine(offset(at, -5.0f, 5.0f), offset(at, 5.0f, -5.0f), HUD_YELLOW, 2.0f);
        }

        // The nose marker stays fixed in the middle while the ball turns behind it
        const ImVec2 nose[] = { offset(centre, -26.0f, 0.0f), offset(centre, -10.0f, 0.0f), offset(centre, 0.0f, 9.0f), offset(centre, 10.0f, 0.0f),
                                offset(centre, 26.0f, 0.0f) };
        pList->AddPolyline(nose, 5, HUD_ORANGE, 0, 3.0f);
        pList->AddCircleFilled(centre, 2.5f, HUD_ORANGE);
        pList->AddCircle(centre, radius + 1.0f, HUD_EDGE, 72, 2.0f);

        // Heading is the compass bearing of the nose, which is meaningless while it points straight up or down
        const Vec3d facing = mRocket.toWorld({ 0.0, 1.0, 0.0 });
        const double pitch = std::asin(std::clamp(facing.dot(here.up), -1.0, 1.0)) / degree;
        const bool hasHeading = std::abs(pitch) < 89.5;
        double heading = std::atan2(facing.dot(here.east), facing.dot(north)) / degree;
        heading = heading < 0.0 ? heading + 360.0 : heading;

        // The heading tape runs round the top of the ring, centred on where the nose points
        const double span = 60.0; // degrees of heading shown either side of the middle
        const float sweep = PI * 0.32f;
        if (hasHeading)
        {
            for (int mark = (int)std::ceil((heading - span) / 10.0) * 10; mark <= heading + span; mark += 10)
            {
                const float angle = -PI * 0.5f + (float)((mark - heading) / span) * sweep;
                const float c = std::cos(angle);
                const float s = std::sin(angle);
                const bool major = (mark % 30 + 30) % 30 == 0;
                const float length = major ? 8.0f : 5.0f;
                pList->AddLine(offset(centre, c * (radius + 3.0f), s * (radius + 3.0f)), offset(centre, c * (radius + 3.0f + length), s * (radius + 3.0f + length)),
                               major ? HUD_TEXT : HUD_MUTED, 1.5f);
                // The label under the heading box would hide behind its pointer
                if (major && std::abs(mark - heading) > 8.0)
                {
                    char label[8];
                    std::snprintf(label, sizeof(label), "%d", (mark % 360 + 360) % 360);
                    textCentred(pList, offset(centre, c * (radius + 18.0f), s * (radius + 18.0f)), 10.0f, HUD_MUTED, label);
                }
            }
        }
        char value[32];
        const ImVec2 headingMin = offset(centre, -32.0f, -NAVBALL_BEZEL - 24.0f);
        const ImVec2 headingMax = offset(centre, 32.0f, -NAVBALL_BEZEL + 6.0f);
        pList->AddRectFilled(headingMin, headingMax, HUD_BOX, 3.0f);
        pList->AddRect(headingMin, headingMax, HUD_TEXT, 3.0f, 0, 1.5f);
        pList->AddTriangleFilled({ centre.x - 5.0f, headingMax.y }, { centre.x + 5.0f, headingMax.y }, { centre.x, headingMax.y + 6.0f }, HUD_TEXT);
        if (hasHeading)
        {
            std::snprintf(value, sizeof(value), "%03d", (int)std::lround(heading) % 360);
        }
        else
        {
            std::snprintf(value, sizeof(value), "---");
        }
        textCentred(pList, { centre.x, (headingMin.y + headingMax.y) * 0.5f }, 22.0f, HUD_TEXT, value);
        std::snprintf(value, sizeof(value), "PITCH %+.0f", pitch);
        textCentred(pList, offset(centre, 0.0f, radius + 15.0f), 11.0f, HUD_MUTED, value);

        // The throttle fills the left arc from the bottom, its share tagged at the top and the thrust at the bottom
        const float throttle = (float)mRocket.throttle();
        const float low = PI * 0.7f;
        const float high = PI * 1.3f;
        pList->PathArcTo(centre, ARC_RADIUS, low, high, 40);
        pList->PathStroke(HUD_DIM, 0, ARC_WIDTH);
        if (throttle > 0.0f)
        {
            pList->PathArcTo(centre, ARC_RADIUS, low, low + (high - low) * throttle, 40);
            pList->PathStroke(HUD_GREEN, 0, ARC_WIDTH);
        }
        for (int tick = 0; tick <= 4; tick++)
        {
            const float angle = low + (high - low) * tick / 4.0f;
            const float c = std::cos(angle);
            const float s = std::sin(angle);
            pList->AddLine(offset(centre, c * (ARC_RADIUS + 9.0f), s * (ARC_RADIUS + 9.0f)), offset(centre, c * (ARC_RADIUS + 14.0f), s * (ARC_RADIUS + 14.0f)),
                           HUD_MUTED, 2.0f);
        }
        const ImVec2 arcTop = offset(centre, std::cos(high) * ARC_RADIUS, std::sin(high) * ARC_RADIUS);
        const ImVec2 tagMin = offset(arcTop, -24.0f, -34.0f);
        const ImVec2 tagMax = offset(arcTop, 24.0f, -12.0f);
        pList->AddRectFilled(tagMin, tagMax, HUD_BOX, 3.0f);
        pList->AddRect(tagMin, tagMax, HUD_GREEN, 3.0f, 0, 1.5f);
        std::snprintf(value, sizeof(value), "%d%%", (int)std::lround(throttle * 100.0f));
        textCentred(pList, { arcTop.x, (tagMin.y + tagMax.y) * 0.5f }, 16.0f, HUD_TEXT, value);
        const ImVec2 arcBottom = offset(centre, std::cos(low) * ARC_RADIUS, std::sin(low) * ARC_RADIUS);
        std::snprintf(value, sizeof(value), "%.0f kN", mRocket.thrust() / 1000.0);
        textRight(pList, arcBottom.x - 12.0f, arcBottom.y - 4.0f, 13.0f, HUD_MUTED, value);

        // The climb rate swings the right arc up or down from the middle, spread out so small rates still show
        const Vec3d up = mRocket.position().normalized();
        const double climb = mRocket.velocity().dot(up);
        auto climbAngle = [](double rate)
        {
            const float share = (float)std::min(1.0, std::log10(1.0 + std::abs(rate)) / 3.0);
            return (rate >= 0.0 ? -share : share) * PI * 0.3f;
        };
        pList->PathArcTo(centre, ARC_RADIUS, -PI * 0.3f, PI * 0.3f, 40);
        pList->PathStroke(HUD_DIM, 0, ARC_WIDTH);
        const float climbTo = climbAngle(climb);
        if (std::abs(climbTo) > 0.001f)
        {
            pList->PathArcTo(centre, ARC_RADIUS, std::min(0.0f, climbTo), std::max(0.0f, climbTo), 40);
            pList->PathStroke(climb >= 0.0 ? HUD_BLUE : HUD_RED, 0, ARC_WIDTH);
        }
        for (double rate : { -100.0, -10.0, 10.0, 100.0 })
        {
            const float angle = climbAngle(rate);
            const float c = std::cos(angle);
            const float s = std::sin(angle);
            pList->AddLine(offset(centre, c * (ARC_RADIUS + 9.0f), s * (ARC_RADIUS + 9.0f)), offset(centre, c * (ARC_RADIUS + 14.0f), s * (ARC_RADIUS + 14.0f)),
                           HUD_MUTED, 2.0f);
            std::snprintf(value, sizeof(value), "%+.0f", rate);
            text(pList, offset(centre, c * (ARC_RADIUS + 17.0f), s * (ARC_RADIUS + 17.0f) - 6.0f), 10.0f, HUD_MUTED, value);
        }
        pList->AddLine(offset(centre, ARC_RADIUS - ARC_WIDTH * 0.5f, 0.0f), offset(centre, ARC_RADIUS + 14.0f, 0.0f), HUD_TEXT, 2.0f);
        std::snprintf(value, sizeof(value), "%+.1f", climb);
        text(pList, offset(centre, ARC_RADIUS + 18.0f, -14.0f), 16.0f, HUD_TEXT, value);
        text(pList, offset(centre, ARC_RADIUS + 18.0f, 2.0f), 11.0f, HUD_CYAN, "m/s");

        // Speed and altitude sit boxed over the sides of the ring
        std::snprintf(value, sizeof(value), "%.1f", speed);
        sideBox(pList, offset(centre, -100.0f, 0.0f), HUD_YELLOW, "SURFACE", value, "m/s");
        const char* unit = "m";
        writeHeight(value, sizeof(value), std::max(0.0, mRocket.altitude(mSolarSystem)), unit);
        sideBox(pList, offset(centre, 100.0f, 0.0f), HUD_PINK, "ALTITUDE", value, unit);
        ImGui::End();
    }

    // Under the instruments: how high and low the rocket's orbit reaches, and how long until it gets to each
    void FlightScene::drawOrbit()
    {
        const BodyInfo& reference = mSolarSystem.body(mRocket.reference());
        const OrbitInfo orbit = orbitFrom(mRocket.position(), mRocket.velocity(), reference.mu);
        const ImVec2 min = { HUD_MARGIN, (float)GetScreenHeight() - ORBIT_STRIP_SIZE.y - HUD_MARGIN };
        const ImVec2 max = offset(min, ORBIT_STRIP_SIZE.x, ORBIT_STRIP_SIZE.y);
        ImGui::SetNextWindowPos(min, ImGuiCond_Always);
        ImGui::SetNextWindowSize(ORBIT_STRIP_SIZE, ImGuiCond_Always);
        ImGui::Begin("Orbit", nullptr, OVERLAY_FLAGS | ImGuiWindowFlags_NoInputs);
        ImDrawList* pList = ImGui::GetWindowDrawList();
        panel(pList, min, max, HUD_EDGE);

        // Each line is a name, the height in metres, and when it gets there
        auto line = [&](float y, ImU32 accent, const char* label, const std::string& height, const std::string& when)
        {
            text(pList, { min.x + 10.0f, y }, 15.0f, accent, label);
            textRight(pList, min.x + 160.0f, y, 15.0f, HUD_TEXT, height.c_str());
            text(pList, { min.x + 164.0f, y + 3.0f }, 11.0f, accent, "m");
            text(pList, { min.x + 186.0f, y + 1.0f }, 13.0f, HUD_MUTED, when.c_str());
        };
        if (orbit.escaping)
        {
            line(min.y + 4.0f, HUD_BLUE, "AP", "---", "escaping");
        }
        else
        {
            line(min.y + 4.0f, HUD_BLUE, "AP", signedGrouped(orbit.apoapsis - reference.radius), "in T-" + clockText(orbit.timeToApoapsis));
        }

        // A periapsis under the surface means the orbit still runs into the ground
        const double low = orbit.periapsis - reference.radius;
        const bool passed = orbit.escaping && orbit.timeToPeriapsis <= 0.0;
        line(min.y + 20.0f, low < 0.0 ? HUD_RED : HUD_PINK, "PE", signedGrouped(low), passed ? "passed" : "in T-" + clockText(orbit.timeToPeriapsis));
        caption(pList, min, max, "ORBITAL INFO");
        char value[32];
        std::snprintf(value, sizeof(value), "INC %.1f", orbit.inclination * 180.0 / std::numbers::pi);
        textRight(pList, max.x - 8.0f, max.y - 13.0f, 11.0f, HUD_MUTED, value);
        ImGui::End();
    }

    // Top right: how hard the crew is pressed back into their seats, in multiples of Earth's gravity
    void FlightScene::drawGForce()
    {
        const ImVec2 size = { 58.0f, 140.0f };
        const ImVec2 min = { (float)GetScreenWidth() - size.x - HUD_MARGIN, HUD_MARGIN };
        const ImVec2 max = offset(min, size.x, size.y);
        ImGui::SetNextWindowPos(min, ImGuiCond_Always);
        ImGui::SetNextWindowSize(size, ImGuiCond_Always);
        ImGui::Begin("G force", nullptr, OVERLAY_FLAGS | ImGuiWindowFlags_NoInputs);
        ImDrawList* pList = ImGui::GetWindowDrawList();
        panel(pList, min, max, HUD_EDGE);
        const float top = min.y + 26.0f;
        const float bottom = max.y - 20.0f;
        const float barX = min.x + 34.0f;
        auto heightOf = [&](double g)
        {
            return bottom - (float)((std::clamp(g, -5.0, 15.0) + 5.0) / 20.0) * (bottom - top);
        };
        pList->AddRectFilled({ barX, top }, { barX + 10.0f, bottom }, HUD_DIM, 2.0f);
        const float zero = heightOf(0.0);
        const float now = heightOf(mGForce);
        const bool hard = mGForce > 6.0 || mGForce < -2.0;
        pList->AddRectFilled({ barX, std::min(zero, now) }, { barX + 10.0f, std::max(zero, now) + 1.0f }, hard ? HUD_RED : HUD_GREEN, 2.0f);
        for (int g : { 15, 5, 0, -5 })
        {
            char label[8];
            std::snprintf(label, sizeof(label), "%d", g);
            textRight(pList, barX - 5.0f, heightOf(g) - 6.0f, 11.0f, HUD_MUTED, label);
            pList->AddLine({ barX - 3.0f, heightOf(g) }, { barX, heightOf(g) }, HUD_MUTED, 1.0f);
        }
        char value[16];
        std::snprintf(value, sizeof(value), "%.1f", mGForce);
        textCentred(pList, { (min.x + max.x) * 0.5f, min.y + 12.0f }, 16.0f, hard ? HUD_RED : HUD_TEXT, value);
        caption(pList, min, max, "G FORCE");
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
        const ImU32 ink = canStage ? HUD_GO_INK : HUD_MUTED;
        pList->AddRectFilled(goMin, goMax, canStage ? HUD_GO : HUD_PANEL, 6.0f);
        pList->AddRect(goMin, goMax, canStage ? HUD_GO_EDGE : HUD_EDGE, 6.0f, 0, 2.0f);
        text(pList, offset(goMin, 12.0f, 4.0f), 34.0f, ink, mLaunched ? "GO" : "LAUNCH");
        std::snprintf(value, sizeof(value), "%d", design.stageCount() - vehicle.nextStage);
        pList->AddRectFilled({ goMax.x - 44.0f, goMin.y + 8.0f }, { goMax.x - 8.0f, goMin.y + 38.0f }, canStage ? HUD_GO_INK : HUD_DIM, 4.0f);
        textRight(pList, goMax.x - 16.0f, goMin.y + 10.0f, 26.0f, canStage ? HUD_GO : HUD_MUTED, value);
        text(pList, { goMin.x + 12.0f, goMax.y - 22.0f }, 14.0f, ink, "dV");
        std::snprintf(value, sizeof(value), "%s m/s", grouped(mStats.totalDeltaVVacuum).c_str());
        textRight(pList, goMax.x - 10.0f, goMax.y - 25.0f, 19.0f, canStage ? HUD_GO_INK : HUD_TEXT, value);
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
