#include "flight/MapView.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include <raymath.h>
#include <rlgl.h>

#include "flight/Globe.h"
#include "flight/PlanetSurface.h"
#include "flight/SpaceSky.h"
#include "flight/SurfaceFrame.h"

namespace tenth
{
    namespace
    {
        constexpr int PATH_SEGMENTS = 360;
        constexpr int ORBIT_SEGMENTS = 256;
        constexpr double FAR_PLANE = 1.0e13;
        constexpr float MAX_DISTANCE = 2.0e11f; // far enough out to see Earth's whole orbit round the Sun
        constexpr double ROCKET_CLOSEST = 400.0; // metres
        constexpr double ROCKET_FRAMING = 4.0e4;
        constexpr double BODY_CLOSEST = 1.3; // body radii from its centre
        constexpr double BODY_FRAMING = 4.0;
        constexpr double CAMERA_CLEARANCE = 1.02; // body radii the camera is kept outside of
        constexpr float EASE = 6.0f; // per second: how quickly the view slides and zooms to a new focus
        constexpr float ZOOM_STEP = 0.15f;
        constexpr float DOT = 3.0f; // pixels: a body smaller than this on screen shows as a dot this size
        constexpr float PICK = 10.0f; // pixels round a marker that a click still lands on
        constexpr double ORBIT_FADE_START = 0.01; // an orbit fades out once the eye is this share of its size from the body
        constexpr double ORBIT_FADE_END = 0.04;

        constexpr ImU32 PATH_COLOR = IM_COL32(90, 175, 255, 255);
        constexpr ImU32 ORBIT_COLOR = IM_COL32(150, 165, 185, 150);
        constexpr ImU32 SPHERE_COLOR = IM_COL32(150, 165, 185, 50); // the edge of a sphere of influence
        constexpr ImU32 TARGET_COLOR = IM_COL32(190, 110, 255, 255);
        constexpr ImU32 APOAPSIS_COLOR = IM_COL32(90, 150, 255, 255);
        constexpr ImU32 PERIAPSIS_COLOR = IM_COL32(230, 90, 200, 255);
        constexpr ImU32 WARNING_COLOR = IM_COL32(225, 55, 55, 255);
        constexpr ImU32 LABEL_BACK = IM_COL32(8, 12, 18, 225);
        constexpr ImU32 TEXT_COLOR = IM_COL32(232, 236, 240, 255);
        constexpr ImU32 MUTED_COLOR = IM_COL32(150, 162, 174, 255);
        constexpr ImU32 ROCKET_COLOR = IM_COL32(55, 105, 215, 255);
        constexpr ImU32 DEBRIS_COLOR = IM_COL32(115, 125, 140, 255);
        constexpr ImU32 STAR_DOT = IM_COL32(255, 220, 150, 255);
        constexpr ImU32 PLANET_DOT = IM_COL32(90, 140, 210, 255);
        constexpr ImU32 MOON_DOT = IM_COL32(175, 175, 175, 255);
        constexpr float LABEL_SIZE = 13.0f;

        Vector3 toVector(const Vec3d& v)
        {
            return { (float)v.x, (float)v.y, (float)v.z };
        }

        Vec3d toVec3d(Vector3 v)
        {
            return { v.x, v.y, v.z };
        }

        ImVec2 offset(ImVec2 a, float x, float y)
        {
            return { a.x + x, a.y + y };
        }

        ImU32 withAlpha(ImU32 color, float share)
        {
            const unsigned int alpha = (unsigned int)((float)((color >> IM_COL32_A_SHIFT) & 0xFF) * std::clamp(share, 0.0f, 1.0f));
            return (color & ~IM_COL32_A_MASK) | (alpha << IM_COL32_A_SHIFT);
        }

        std::string upper(const std::string& name)
        {
            std::string result = name;
            for (char& c : result)
            {
                c = (char)std::toupper((unsigned char)c);
            }
            return result;
        }

        std::string grouped(double value)
        {
            std::string digits = std::to_string((long long)std::llround(std::abs(value)));
            for (int i = (int)digits.size() - 3; i > 0; i -= 3)
            {
                digits.insert((size_t)i, ",");
            }
            return (value < 0.0 ? "-" : "") + digits;
        }

        // Metres while it's low, kilometres above that
        std::string heightText(double metres)
        {
            return std::abs(metres) < 10000.0 ? grouped(metres) + " m" : grouped(metres / 1000.0) + " km";
        }

        // How long until something happens, as in T-00:46:05, with days in front once it's that far off
        std::string countdown(double seconds)
        {
            const long long total = (long long)std::max(0.0, seconds);
            char value[48];
            if (total >= 86400)
            {
                std::snprintf(value, sizeof(value), "T-%lldd %02lld:%02lld:%02lld", total / 86400, total / 3600 % 24, total / 60 % 60, total % 60);
            }
            else
            {
                std::snprintf(value, sizeof(value), "T-%02lld:%02lld:%02lld", total / 3600, total / 60 % 60, total % 60);
            }
            return value;
        }

        // Turns points measured from the view's centre into places on screen, and says when a body hides one
        struct Projector
        {
            struct Ball
            {
                Vec3d centre;
                double radius = 0.0;
            };

            Vec3d eye;
            Vec3d forward;
            Vec3d right;
            Vec3d up;
            double scale = 1.0; // pixels across for each unit sideways at unit depth
            ImVec2 middle;
            double nearest = 1.0; // anything closer to the eye than this counts as behind it
            std::vector<Ball> balls;

            double depth(const Vec3d& p) const { return (p - eye).dot(forward); }

            ImVec2 place(const Vec3d& p) const
            {
                const Vec3d d = p - eye;
                const double z = std::max(d.dot(forward), nearest);
                return { middle.x + (float)(d.dot(right) / z * scale), middle.y - (float)(d.dot(up) / z * scale) };
            }

            bool hidden(const Vec3d& p) const
            {
                const Vec3d d = p - eye;
                const double length = d.length();
                if (length <= 0.0)
                {
                    return false;
                }
                const Vec3d along = d / length;
                for (const Ball& ball : balls)
                {
                    const Vec3d toCentre = ball.centre - eye;
                    const double closest = toCentre.dot(along);
                    const double missSquared = toCentre.lengthSquared() - closest * closest;
                    const double radiusSquared = ball.radius * ball.radius;
                    if (closest <= 0.0 || missSquared >= radiusSquared)
                    {
                        continue;
                    }
                    // Where the sight line first enters the body; a point on the near surface itself still shows
                    const double entry = closest - std::sqrt(radiusSquared - missSquared);
                    if (entry < length - ball.radius * 1e-4)
                    {
                        return true;
                    }
                }
                return false;
            }

            // On screen and not behind a body
            bool shows(const Vec3d& p, ImVec2& at) const
            {
                if (depth(p) <= nearest || hidden(p))
                {
                    return false;
                }
                at = place(p);
                return true;
            }

            float pixels(const Vec3d& centre, double radius) const
            {
                const double z = depth(centre);
                return z > nearest ? (float)(radius / z * scale) : 0.0f;
            }
        };

        Projector makeProjector(const Camera3D& camera)
        {
            Projector view;
            view.eye = toVec3d(camera.position);
            view.forward = (toVec3d(camera.target) - view.eye).normalized();
            view.right = view.forward.cross(toVec3d(camera.up)).normalized();
            view.up = view.right.cross(view.forward);
            const float height = (float)GetScreenHeight();
            view.scale = height * 0.5 / std::tan(camera.fovy * DEG2RAD * 0.5);
            view.middle = { (float)GetScreenWidth() * 0.5f, height * 0.5f };
            view.nearest = rlGetCullDistanceNear();
            return view;
        }

        // A line through points measured from the view's centre, broken where a body hides it and cut short where it
        // passes behind the eye
        void drawLine(ImDrawList* pList, const Projector& view, const std::vector<Vec3d>& points, ImU32 color, float thickness)
        {
            std::vector<ImVec2> run;
            auto flush = [&]()
            {
                if (run.size() >= 2)
                {
                    pList->AddPolyline(run.data(), (int)run.size(), color, 0, thickness);
                }
                run.clear();
            };
            const double front = view.nearest * 1.001;
            for (size_t i = 0; i < points.size(); i++)
            {
                const Vec3d& p = points[i];
                const bool ahead = view.depth(p) > view.nearest;
                const bool visible = ahead && !view.hidden(p);
                if (i > 0)
                {
                    const Vec3d& q = points[i - 1];
                    const bool wasAhead = view.depth(q) > view.nearest;
                    if (ahead != wasAhead)
                    {
                        const double t = (view.depth(q) - front) / (view.depth(q) - view.depth(p));
                        const ImVec2 crossing = view.place(q + (p - q) * t);
                        if (visible || !run.empty())
                        {
                            run.push_back(crossing);
                        }
                    }
                }
                if (visible)
                {
                    run.push_back(view.place(p));
                }
                else
                {
                    flush();
                }
            }
            flush();
        }

        bool mouseOver(ImVec2 min, ImVec2 max)
        {
            const ImVec2 mouse = ImGui::GetMousePos();
            return mouse.x >= min.x && mouse.x <= max.x && mouse.y >= min.y && mouse.y <= max.y;
        }

        bool mouseNear(ImVec2 at, float reach)
        {
            const ImVec2 mouse = ImGui::GetMousePos();
            const float dx = mouse.x - at.x;
            const float dy = mouse.y - at.y;
            return dx * dx + dy * dy <= reach * reach;
        }

        // A point on a path with its name on a tag up and to the right; true while the mouse is over either
        bool drawNode(ImDrawList* pList, ImVec2 at, ImU32 color, const std::string& name)
        {
            pList->AddCircleFilled(at, 4.5f, LABEL_BACK, 16);
            pList->AddCircle(at, 4.5f, color, 16, 2.0f);
            const ImVec2 bend = offset(at, 12.0f, -12.0f);
            pList->AddLine(offset(at, 3.0f, -3.0f), bend, color, 1.5f);
            const ImVec2 size = ImGui::GetFont()->CalcTextSizeA(LABEL_SIZE, FLT_MAX, 0.0f, name.c_str());
            const ImVec2 min = { bend.x, bend.y - size.y * 0.5f - 3.0f };
            const ImVec2 max = { min.x + size.x + 10.0f, min.y + size.y + 6.0f };
            pList->AddRectFilled(min, max, LABEL_BACK, 3.0f);
            pList->AddRect(min, max, color, 3.0f, 0, 1.2f);
            pList->AddText(ImGui::GetFont(), LABEL_SIZE, offset(min, 5.0f, 3.0f), TEXT_COLOR, name.c_str());
            return mouseOver(min, max) || mouseNear(at, PICK);
        }

        // A red warning sign, where the path runs into the ground
        bool drawWarning(ImDrawList* pList, ImVec2 at)
        {
            const float s = 9.0f;
            pList->AddRectFilled(offset(at, -s, -s), offset(at, s, s), WARNING_COLOR, 3.0f);
            pList->AddTriangle(offset(at, 0.0f, -s * 0.7f), offset(at, -s * 0.7f, s * 0.55f), offset(at, s * 0.7f, s * 0.55f), TEXT_COLOR, 1.5f);
            pList->AddLine(offset(at, 0.0f, -s * 0.25f), offset(at, 0.0f, s * 0.15f), TEXT_COLOR, 1.5f);
            pList->AddCircleFilled(offset(at, 0.0f, s * 0.35f), 1.0f, TEXT_COLOR);
            return mouseNear(at, s + 2.0f);
        }

        // The rocket's badge: a small upright rocket on a blue tile
        void drawRocketBadge(ImDrawList* pList, ImVec2 at)
        {
            const float s = 9.0f;
            pList->AddRectFilled(offset(at, -s, -s), offset(at, s, s), ROCKET_COLOR, 3.0f);
            pList->AddRect(offset(at, -s, -s), offset(at, s, s), IM_COL32(170, 200, 255, 255), 3.0f, 0, 1.2f);
            pList->AddRectFilled(offset(at, -2.0f, -3.0f), offset(at, 2.0f, 4.0f), TEXT_COLOR);
            pList->AddTriangleFilled(offset(at, -2.0f, -3.0f), offset(at, 2.0f, -3.0f), offset(at, 0.0f, -7.0f), TEXT_COLOR);
            pList->AddTriangleFilled(offset(at, -2.0f, 1.0f), offset(at, -2.0f, 5.0f), offset(at, -5.0f, 6.0f), TEXT_COLOR);
            pList->AddTriangleFilled(offset(at, 2.0f, 1.0f), offset(at, 5.0f, 6.0f), offset(at, 2.0f, 5.0f), TEXT_COLOR);
        }

        // What a body is, for the menu: the star the rest go round, a planet round it, or a moon round a planet
        const char* kindOf(const SolarSystem& system, BodyId id)
        {
            const BodyId parent = system.body(id).parent;
            if (parent == NO_BODY)
            {
                return "STAR";
            }
            return system.body(parent).parent == NO_BODY ? "PLANET" : "MOON";
        }
    }

    void MapView::init(const SolarSystem& system)
    {
        mDots.clear();
        for (BodyId id = 0; id < system.bodyCount(); id++)
        {
            const BodyId parent = system.body(id).parent;
            mDots.push_back(parent == NO_BODY ? STAR_DOT : (system.body(parent).parent == NO_BODY ? PLANET_DOT : MOON_DOT));
        }
        mCamera.liftWithKeys = false;
        mCamera.minPitch = -1.45f;
        mCamera.maxPitch = 1.45f;
        mCamera.zoomStep = ZOOM_STEP;
        mCamera.maxDistance = MAX_DISTANCE;
    }

    Vec3d MapView::bodyPosition(const Flight& flight, BodyId id) const
    {
        return flight.pSystem->positionAt(id, flight.time);
    }

    Vec3d MapView::focusPosition(const Flight& flight, BodyId focus) const
    {
        if (focus == NO_BODY)
        {
            return bodyPosition(flight, flight.reference) + flight.position;
        }
        return bodyPosition(flight, focus);
    }

    Vec3d MapView::sunFrom(const Flight& flight, BodyId id) const
    {
        BodyId root = id;
        while (flight.pSystem->body(root).parent != NO_BODY)
        {
            root = flight.pSystem->body(root).parent;
        }
        const Vec3d toward = bodyPosition(flight, root) - bodyPosition(flight, id);
        return toward.lengthSquared() > 0.0 ? toward.normalized() : Vec3d{ 0.0, 1.0, 0.0 };
    }

    double MapView::minimumDistance(const Flight& flight, BodyId focus) const
    {
        return focus == NO_BODY ? ROCKET_CLOSEST : flight.pSystem->body(focus).radius * BODY_CLOSEST;
    }

    double MapView::framingDistance(const Flight& flight, BodyId focus) const
    {
        return focus == NO_BODY ? ROCKET_FRAMING : flight.pSystem->body(focus).radius * BODY_FRAMING;
    }

    void MapView::setFocus(const Flight& flight, BodyId focus)
    {
        if (focus == mFocus)
        {
            return;
        }
        // The view keeps looking where it was and slides over to the new centre
        const Vec3d lookingAt = focusPosition(flight, mFocus) + mEase;
        mFocus = focus;
        mEase = lookingAt - focusPosition(flight, focus);
        mDistanceTarget = (float)framingDistance(flight, focus);
    }

    void MapView::reset(const Flight& flight)
    {
        const BodyInfo& reference = flight.pSystem->body(flight.reference);
        mFocus = flight.reference;
        mOrigin = focusPosition(flight, mFocus);
        mEase = {};
        mDistanceTarget = 0.0f;
        mMenu.open = false;

        // Far enough out to take in the whole orbit, or the way out of the sphere of influence
        double reach = std::max(flight.position.length(), reference.radius * 1.2);
        Conic conic;
        if (conicFrom(flight.position, flight.velocity, reference.mu, conic))
        {
            reach = std::max(reach, conic.closed() ? conic.semiLatus / (1.0 - conic.eccentricity) : reference.sphereOfInfluence);
        }
        reach = std::min(reach, reference.sphereOfInfluence);
        mCamera.distance = (float)(reach * 3.0);
        mCamera.yaw = 0.6f;
        mCamera.pitch = 0.5f;
    }

    void MapView::update(Camera3D& camera, const Flight& flight, float dt)
    {
        const bool keyboardFree = ImGui::GetCurrentContext() == nullptr || !ImGui::GetIO().WantCaptureKeyboard;
        if (keyboardFree && IsKeyPressed(KEY_TAB))
        {
            const BodyId next = mFocus + 1 < flight.pSystem->bodyCount() ? mFocus + 1 : NO_BODY;
            setFocus(flight, mFocus == NO_BODY ? 0 : next);
        }

        mOrigin = focusPosition(flight, mFocus);
        mEase = mEase * std::exp(-EASE * dt);
        const Vector3 focus = toVector(mEase);
        mCamera.focus = focus;
        mCamera.minFocusHeight = focus.y;
        mCamera.maxFocusHeight = focus.y;
        mCamera.minDistance = (float)minimumDistance(flight, mFocus);

        // Zooming by hand while it eases takes over from the easing
        const bool mouseFree = ImGui::GetCurrentContext() == nullptr || !ImGui::GetIO().WantCaptureMouse;
        if (mouseFree && GetMouseWheelMove() != 0.0f)
        {
            mDistanceTarget = 0.0f;
        }
        if (mDistanceTarget > 0.0f)
        {
            mCamera.distance += (mDistanceTarget - mCamera.distance) * (1.0f - std::exp(-EASE * dt));
            if (std::abs(mDistanceTarget - mCamera.distance) < 0.01f * mDistanceTarget)
            {
                mDistanceTarget = 0.0f;
            }
        }
        mCamera.update(camera, dt);

        // The camera is kept outside every body, and the near plane sits short of the closest one so none is cut off
        Vec3d eye = mOrigin + toVec3d(camera.position);
        double nearest = (toVec3d(camera.position) - toVec3d(camera.target)).length();
        for (BodyId id = 0; id < flight.pSystem->bodyCount(); id++)
        {
            const double radius = flight.pSystem->body(id).radius;
            const Vec3d centre = bodyPosition(flight, id);
            const Vec3d out = eye - centre;
            if (out.length() < radius * CAMERA_CLEARANCE)
            {
                eye = centre + out.normalized() * (radius * CAMERA_CLEARANCE);
                camera.position = toVector(eye - mOrigin);
            }
            nearest = std::min(nearest, (eye - centre).length() - radius);
        }
        rlSetClipPlanes(std::max(1.0, nearest * 0.5), FAR_PLANE);
    }

    void MapView::draw(const Camera3D& camera, const Flight& flight, const PlanetSurface& earth, const SpaceSky& sky,
                       const BodyGlobes& globes) const
    {
        const SolarSystem& system = *flight.pSystem;
        const Vec3d eye = mOrigin + toVec3d(camera.position);
        SurfaceFrame axes;
        axes.east = { 1.0, 0.0, 0.0 };
        axes.up = { 0.0, 1.0, 0.0 };
        axes.south = { 0.0, 0.0, 1.0 };

        // The stars, with Earth's air glowing round it. The Sun is a body here, so the sky leaves out its own.
        for (BodyId id = 0; id < system.bodyCount(); id++)
        {
            if (!globes.usesPlanet(id))
            {
                continue;
            }
            const BodyInfo& info = system.body(id);
            SpaceSky::View view;
            view.axes = axes;
            view.centre = toVector(bodyPosition(flight, id) - mOrigin);
            view.camera = camera.position;
            view.sun = toVector(sunFrom(flight, id));
            view.radius = info.radius;
            view.atmosphereHeight = info.atmosphereHeight;
            view.cameraHeight = (eye - bodyPosition(flight, id)).length() - info.radius;
            view.sunDisc = false;
            sky.draw(view);
        }

        for (BodyId id = 0; id < system.bodyCount(); id++)
        {
            const BodyInfo& info = system.body(id);
            const Vec3d position = bodyPosition(flight, id);
            const Vector3 centre = toVector(position - mOrigin);
            const Vec3d sun = sunFrom(flight, id);
            if (globes.usesPlanet(id))
            {
                PlanetSurface::View view;
                view.axes = axes;
                view.centre = centre;
                view.camera = camera.position;
                view.sun = toVector(sun);
                view.cameraHeight = (eye - position).length() - info.radius;
                view.ground = false;
                earth.draw(view);
                continue;
            }

            const BodyAxes turned = bodyAxes(system, id, flight.time);
            const double r = info.radius;
            globes.drawGlobe(id, placedMatrix(turned.x * r, turned.y * r, turned.z * r, centre), toVector(sun));
        }
    }

    void MapView::drawOverlay(const Camera3D& camera, const Flight& flight, BodyId& target)
    {
        const SolarSystem& system = *flight.pSystem;
        ImDrawList* pList = ImGui::GetBackgroundDrawList();
        Projector view = makeProjector(camera);
        for (BodyId id = 0; id < system.bodyCount(); id++)
        {
            view.balls.push_back({ bodyPosition(flight, id) - mOrigin, system.body(id).radius });
        }
        const bool mouseFree = !ImGui::GetIO().WantCaptureMouse;
        const Vec3d eye = mOrigin + view.eye;

        // What a click can land on: each body, and the rocket as NO_BODY
        struct Pick
        {
            ImVec2 at;
            float reach = 0.0f;
            BodyId body = NO_BODY;
        };
        std::vector<Pick> picks;
        auto tooltip = [&](bool hovered, const std::string& lines)
        {
            if (hovered && mouseFree)
            {
                ImGui::SetTooltip("%s", lines.c_str());
            }
        };

        // Each body's orbit round its parent, fading out up close where it would only cut across the view
        for (BodyId id = 0; id < system.bodyCount(); id++)
        {
            const BodyInfo& info = system.body(id);
            if (info.parent == NO_BODY)
            {
                continue;
            }
            const Vec3d parent = bodyPosition(flight, info.parent);
            const Vec3d offsetFromParent = bodyPosition(flight, id) - parent;
            const Vec3d velocity = system.velocityAt(id, flight.time) - system.velocityAt(info.parent, flight.time);
            const double size = offsetFromParent.length();
            const double closeness = (eye - bodyPosition(flight, id)).length() / size;
            const float share = (float)std::clamp((closeness - ORBIT_FADE_START) / (ORBIT_FADE_END - ORBIT_FADE_START), 0.0, 1.0);
            if (share <= 0.0f)
            {
                continue;
            }
            PredictedPath orbit = predictPath(offsetFromParent, velocity, system.body(info.parent).mu, 0.0, std::numeric_limits<double>::infinity(),
                                              ORBIT_SEGMENTS);
            for (Vec3d& point : orbit.points)
            {
                point = point + parent - mOrigin;
            }
            drawLine(pList, view, orbit.points, withAlpha(id == target ? TARGET_COLOR : ORBIT_COLOR, share * (id == target ? 0.8f : 1.0f)), 1.5f);
        }

        // The bodies themselves: a dot once too small to see, the edge of their sphere of influence, and a ring round
        // the target
        for (BodyId id = 0; id < system.bodyCount(); id++)
        {
            const BodyInfo& info = system.body(id);
            const Vec3d centre = bodyPosition(flight, id) - mOrigin;
            ImVec2 at;
            if (view.depth(centre) <= view.nearest)
            {
                continue;
            }
            at = view.place(centre);
            const float size = view.pixels(centre, info.radius);
            if (info.parent != NO_BODY && (eye - bodyPosition(flight, id)).length() > info.sphereOfInfluence)
            {
                const float edge = view.pixels(centre, info.sphereOfInfluence);
                if (edge > size + 6.0f && edge < (float)GetScreenHeight() * 4.0f)
                {
                    pList->AddCircle(at, edge, SPHERE_COLOR, 96, 1.0f);
                }
            }
            const bool covered = view.hidden(centre + (view.eye - centre).normalized() * info.radius);
            if (covered)
            {
                continue;
            }
            if (size < DOT)
            {
                pList->AddCircleFilled(at, DOT, mDots[(size_t)id], 12);
                pList->AddText(ImGui::GetFont(), 12.0f, offset(at, DOT + 4.0f, -7.0f), MUTED_COLOR, info.name.c_str());
            }
            if (id == target)
            {
                const float ring = std::max(size, DOT) + 6.0f;
                pList->AddCircle(at, ring, TARGET_COLOR, 48, 2.0f);
                pList->AddText(ImGui::GetFont(), 12.0f, offset(at, ring * 0.72f + 4.0f, ring * 0.72f), TARGET_COLOR, "TARGET");
            }
            picks.push_back({ at, std::max(size, DOT + PICK * 0.5f), id });
            const bool overBody = mouseNear(at, std::max(size, DOT + PICK * 0.5f));
            tooltip(overBody && size >= DOT, info.name);
        }

        // The rocket's path from here, with its high and low points and where it comes down or leaves
        const BodyInfo& reference = system.body(flight.reference);
        const PredictedPath path = predictPath(flight.position, flight.velocity, reference.mu, reference.radius, reference.sphereOfInfluence,
                                               PATH_SEGMENTS);
        const Vec3d base = bodyPosition(flight, flight.reference) - mOrigin;
        std::vector<Vec3d> points = path.points;
        for (Vec3d& point : points)
        {
            point = point + base;
        }
        drawLine(pList, view, points, PATH_COLOR, 2.5f);

        const std::string name = upper(reference.name);
        const Conic& conic = path.conic;
        ImVec2 at;
        if (path.reachesApoapsis && view.shows(base + conic.positionAt(path.apoapsisAngle), at))
        {
            const bool hovered = drawNode(pList, at, APOAPSIS_COLOR, name + " AP");
            tooltip(hovered, "Apoapsis  " + heightText(conic.radiusAt(path.apoapsisAngle) - reference.radius) + "\nin " +
                                 countdown(conic.timeUntil(path.apoapsisAngle)));
        }
        if (path.reachesPeriapsis && view.shows(base + conic.positionAt(path.periapsisAngle), at))
        {
            const bool hovered = drawNode(pList, at, PERIAPSIS_COLOR, name + " PE");
            tooltip(hovered, "Periapsis  " + heightText(conic.radiusAt(path.periapsisAngle) - reference.radius) + "\nin " +
                                 countdown(conic.timeUntil(path.periapsisAngle)));
        }
        if (!path.points.empty() && path.end == PathEnd::Impact && view.shows(base + conic.positionAt(path.endAngle), at))
        {
            tooltip(drawWarning(pList, at), "Impact with " + reference.name + "\nin " + countdown(path.endTime));
        }
        if (!path.points.empty() && path.end == PathEnd::Escape && view.shows(base + conic.positionAt(path.endAngle), at))
        {
            const bool hovered = drawNode(pList, at, PATH_COLOR, name + " ESCAPE");
            tooltip(hovered, "Leaves " + reference.name + "'s sphere of influence\nin " + countdown(path.endTime));
        }

        // Dropped pieces, then the rocket over them
        for (const Vec3d& piece : flight.debris)
        {
            if (view.shows(base + piece, at))
            {
                pList->AddRectFilled(offset(at, -4.0f, -4.0f), offset(at, 4.0f, 4.0f), DEBRIS_COLOR, 2.0f);
            }
        }
        if (view.shows(base + flight.position, at))
        {
            drawRocketBadge(pList, at);
            picks.push_back({ at, PICK, NO_BODY });
        }

        // A click on a body or the rocket opens its menu by the mouse; a click on nothing closes it
        if (mouseFree && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            const ImVec2 mouse = ImGui::GetMousePos();
            const Pick* pBest = nullptr;
            float bestDistance = FLT_MAX;
            for (const Pick& pick : picks)
            {
                const float distance = std::hypot(mouse.x - pick.at.x, mouse.y - pick.at.y);
                if (distance <= pick.reach && distance < bestDistance)
                {
                    pBest = &pick;
                    bestDistance = distance;
                }
            }
            mMenu.open = pBest != nullptr;
            if (pBest != nullptr)
            {
                mMenu.fresh = true;
                mMenu.at = offset(mouse, 10.0f, 10.0f);
                mMenu.body = pBest->body;
            }
        }
        if (!mMenu.open)
        {
            return;
        }
        ImGui::SetNextWindowPos(mMenu.at, ImGuiCond_Always);
        ImGui::Begin("Map menu", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove);
        const ImVec2 button = { 130.0f, 0.0f };
        if (mMenu.body == NO_BODY)
        {
            ImGui::TextUnformatted("Rocket");
            ImGui::TextDisabled("VESSEL");
        }
        else
        {
            ImGui::TextUnformatted(system.body(mMenu.body).name.c_str());
            ImGui::TextDisabled("%s", kindOf(system, mMenu.body));
        }
        ImGui::Separator();
        bool done = false;
        if (mMenu.body != NO_BODY && mMenu.body != flight.reference)
        {
            if (target == mMenu.body)
            {
                if (ImGui::Button("Clear Target", button))
                {
                    target = NO_BODY;
                    done = true;
                }
            }
            else if (ImGui::Button("Set Target", button))
            {
                target = mMenu.body;
                done = true;
            }
        }
        if (ImGui::Button("Focus", button))
        {
            setFocus(flight, mMenu.body);
            done = true;
        }
        if (!mMenu.fresh && ImGui::IsMouseClicked(ImGuiMouseButton_Right) && !ImGui::IsWindowHovered())
        {
            done = true;
        }
        ImGui::End();
        mMenu.fresh = false;
        mMenu.open = !done;
    }
}
