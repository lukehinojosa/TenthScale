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
#include "shared/SceneNames.h"

namespace tenth
{
    namespace
    {
        constexpr double DEGREES = std::numbers::pi / 180.0;
        constexpr double FEET = 0.3048; // metres
        constexpr double PAD_LATITUDE = (28.0 + 36.0 / 60.0 + 30.0 / 3600.0) * DEGREES; // Kennedy Space Center, pad 39A
        constexpr double PAD_LONGITUDE = -(80.0 + 36.0 / 60.0 + 16.0 / 3600.0) * DEGREES;
        constexpr double PAD_HEIGHT = 48.0 * FEET;
        constexpr double PAD_LENGTH = 390.0 * FEET; // north to south, along the flame trench
        constexpr double PAD_WIDTH = 325.0 * FEET;
        constexpr double PAD_SLOPE = 3.0; // the sloped sides run this far out for each metre they drop
        constexpr float PAD_LIFT = 0.03f; // markings sit this far above the pad's top so the two don't flicker
        constexpr float PAD_BURY = 1.0f; // the sides reach a little below the ground so no gap shows round the foot
        constexpr float TRENCH_WIDTH = 58.0f * (float)FEET;
        constexpr float TRENCH_LENGTH = 150.0f * (float)FEET; // of the opening across the top
        constexpr Vector3 TOWER_POSITION = { -10.0f, 0.0f, -3.4f }; // on the trench's west edge, behind the rocket from the camera
        constexpr float TOWER_WIDTH = 1.4f;
        constexpr Color GROUND_COLOR = { 74, 96, 62, 255 };
        constexpr Color PAD_COLOR = { 150, 148, 140, 255 };
        constexpr Color PAD_SIDE_COLOR = { 96, 112, 74, 255 }; // grassed over
        constexpr Color TRENCH_COLOR = { 40, 40, 44, 255 };
        constexpr Color TOWER_COLOR = { 170, 70, 50, 255 };

        constexpr Color FLAME_COLOR = { 255, 170, 60, 200 };
        constexpr Color FLAME_CORE_COLOR = { 255, 240, 200, 230 };
        constexpr float FLAME_LENGTH = 6.0f; // metres at full throttle in vacuum, per metre of nozzle width
        constexpr Color CANOPY_COLOR = { 240, 120, 40, 255 };
        constexpr Color CANOPY_STRIPE_COLOR = { 245, 245, 240, 255 };
        constexpr Color FLASH_COLOR = { 255, 240, 190, 255 };
        constexpr Color SPARK_COLOR = { 255, 190, 80, 255 };
        constexpr Color SMOKE_COLOR = { 200, 200, 205, 255 };
        constexpr int SPARK_COUNT = 30; // for a decoupler; smaller charges make fewer
        constexpr int SMOKE_COUNT = 14;
        constexpr float CAMERA_EASE = 2.5f; // per second: how quickly the camera closes the gap to its new framing
        constexpr double FREE_LOOK_SHARE = 0.01; // of the body's radius: how high space starts where there's no air
        constexpr double DEBRIS_SETTLE_TIME = 8.0; // seconds lying still before a dropped piece is cleared away
        constexpr double DEBRIS_RANGE = 40.0e3; // metres from the rocket past which a dropped piece is forgotten
        constexpr double THROTTLE_RATE = 0.5; // share of full throttle per second while Shift or Ctrl is held
        constexpr double WARP_BUDGET = 0.5; // share of each tick's real time time warp may spend on physics
        constexpr double FUEL_WARNING = 10.0; // seconds of fuel left in a burning section below which warp holds at 1x

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
        constexpr ImU32 HUD_PURPLE = IM_COL32(190, 110, 255, 255);
        constexpr ImU32 HUD_GO = IM_COL32(60, 210, 100, 255);
        constexpr ImU32 HUD_GO_EDGE = IM_COL32(150, 255, 170, 255);
        constexpr ImU32 HUD_GO_INK = IM_COL32(10, 70, 30, 255);
        constexpr ImU32 HUD_BOX = IM_COL32(8, 12, 18, 245);
        constexpr float NAVBALL_BEZEL = 106.0f; // the dark ring round the ball that carries the heading tape
        constexpr float ARC_RADIUS = 150.0f; // the throttle and climb rate arcs either side of the ball
        constexpr float ARC_WIDTH = 14.0f;
        constexpr ImVec2 ORBIT_STRIP_SIZE = { 340.0f, 50.0f };
        constexpr ImVec2 CLOCK_SIZE = { 300.0f, 80.0f };
        constexpr ImVec2 GAME_VIEW_SIZE = { 104.0f, 52.0f };
        constexpr float PAUSE_RADIUS = 15.0f;
        constexpr float WARP_ARROW = 7.0f; // half the height of each time warp arrow
        constexpr float WARP_SPACING = 21.0f;
        constexpr float SAS_DIAL_RADIUS = 74.0f;
        constexpr ImVec2 SAS_DIAL_OFFSET = { 302.0f, -6.0f }; // from the navball's centre to the dial's
        constexpr float SAS_RING = 52.0f; // how far out the mode buttons sit
        constexpr float SAS_BUTTON = 17.0f;
        constexpr float SAS_HUB = 30.0f; // the middle, where the rocket turns to point at the chosen mode
        constexpr ImVec2 SAS_TAB_SIZE = { 172.0f, 52.0f }; // the strip under the dial for the modes that aren't directions
        constexpr float SAS_POINTER_EASE = 10.0f; // per second
        constexpr float SAS_SWITCH = 15.0f; // the on and off button sitting in the navball's ring
        constexpr float SAS_SWITCH_ANGLE = PI * 0.33f; // below the right of the ring, clear of the altitude tape
        constexpr float HEADING_SWEEP = PI * 0.2f; // how much of the ring the heading tape fills either side of its middle
        constexpr float SIDE_SWEEP = PI * 0.25f; // the same for the speed and altitude tapes
        constexpr float MARKER_SIZE = 7.0f;
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
        constexpr double FAR_PLANE = 1.0e13; // past the Sun, so every body fits in the view
        constexpr double GROUND_SLACK = 1000.0; // metres the rocket moves before the ground patch under it is rebuilt

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

        // A labelled number boxed at the side of the instruments, like the speed and altitude. Given a share of the
        // atmosphere climbed through, a gauge of its four layers runs up the box's right edge with a red mark at it.
        void sideBox(ImDrawList* pList, ImVec2 centre, ImU32 accent, const char* label, const char* value, const char* unit, float air = -1.0f)
        {
            const ImVec2 min = offset(centre, -42.0f, -26.0f);
            const ImVec2 max = offset(centre, 42.0f, 26.0f);
            pList->AddRectFilled(min, max, HUD_BOX, 3.0f);
            pList->AddRect(min, max, accent, 3.0f, 0, 1.5f);
            text(pList, offset(min, 6.0f, 3.0f), 11.0f, accent, label);
            const float right = air >= 0.0f ? max.x - 14.0f : max.x - 6.0f;
            textRight(pList, right, min.y + 15.0f, 22.0f, HUD_TEXT, value);
            textRight(pList, right, max.y - 13.0f, 11.0f, accent, unit);
            if (air < 0.0f)
            {
                return;
            }
            const ImU32 layers[] = { IM_COL32(120, 180, 235, 255), IM_COL32(70, 125, 205, 255), IM_COL32(40, 70, 155, 255), IM_COL32(28, 30, 80, 255) };
            const float top = min.y + 5.0f;
            const float bottom = max.y - 5.0f;
            const float left = max.x - 10.0f;
            const float layer = (bottom - top) / 4.0f;
            for (int i = 0; i < 4; i++)
            {
                pList->AddRectFilled({ left, bottom - layer * (i + 1) + 0.5f }, { left + 5.0f, bottom - layer * i - 0.5f }, layers[i]);
            }
            const float mark = bottom - (bottom - top) * std::clamp(air, 0.0f, 1.0f);
            pList->AddLine({ left - 3.0f, mark }, { left + 8.0f, mark }, HUD_RED, 2.5f);
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

        // A round number to step a tape by, so about three marks show either side of the middle
        double tapeStep(double value)
        {
            const double raw = std::max(std::abs(value) * 0.2, 2.0);
            const double power = std::pow(10.0, std::floor(std::log10(raw)));
            for (double factor : { 1.0, 2.0, 5.0 })
            {
                if (power * factor >= raw)
                {
                    return power * factor;
                }
            }
            return power * 10.0;
        }

        // Marks round part of the navball's ring sliding past its fixed middle as the value changes, larger values
        // lying toward the end a positive sweep runs to. Major marks are labelled; nothing below lowest is shown.
        template <typename Label>
        void drawTape(ImDrawList* pList, ImVec2 centre, float radius, float middle, float sweep, double value, double step, int minors,
                      double lowest, Label label)
        {
            const double span = step * 3.0;
            const double minor = step / minors;
            for (double mark = std::ceil((value - span) / minor) * minor; mark <= value + span; mark += minor)
            {
                if (mark < lowest - minor * 0.5)
                {
                    continue;
                }
                const float along = (float)((mark - value) / span);
                const float angle = middle + along * sweep;
                const float c = std::cos(angle);
                const float s = std::sin(angle);
                const bool major = std::abs(std::remainder(mark, step)) < minor * 0.5;
                const float length = major ? 8.0f : 4.0f;
                pList->AddLine(offset(centre, c * (radius + 3.0f), s * (radius + 3.0f)), offset(centre, c * (radius + 3.0f + length), s * (radius + 3.0f + length)),
                               major ? HUD_TEXT : HUD_MUTED, 1.5f);
                // Labels right at the ends would run into the next tape's
                if (major && std::abs(along) < 0.85f)
                {
                    textCentred(pList, offset(centre, c * (radius + 18.0f), s * (radius + 18.0f)), 10.0f, HUD_MUTED, label(mark).c_str());
                }
            }
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

        // The navball's middle, bottom left above the orbit strip
        ImVec2 navballCentre()
        {
            return { HUD_MARGIN + ARC_RADIUS + ARC_WIDTH, (float)GetScreenHeight() - HUD_MARGIN - ORBIT_STRIP_SIZE.y - 8.0f - ARC_RADIUS * 0.85f };
        }

        // The clock's top left: centred along the bottom, unless that would run it into the SAS control tab on a narrow
        // window
        ImVec2 clockCorner()
        {
            const float clear = navballCentre().x + SAS_DIAL_OFFSET.x + SAS_TAB_SIZE.x * 0.5f + HUD_MARGIN;
            return { std::max(((float)GetScreenWidth() - CLOCK_SIZE.x) * 0.5f, clear), (float)GetScreenHeight() - CLOCK_SIZE.y - HUD_MARGIN };
        }

        ImU32 markerColor(SasMode mode)
        {
            switch (mode)
            {
            case SasMode::Prograde:
            case SasMode::Retrograde:
                return HUD_YELLOW;
            case SasMode::Normal:
            case SasMode::AntiNormal:
                return HUD_PINK;
            case SasMode::RadialOut:
            case SasMode::RadialIn:
                return HUD_CYAN;
            case SasMode::Target:
            case SasMode::AntiTarget:
                return HUD_PURPLE;
            default:
                return HUD_GREEN;
            }
        }

        const char* modeName(SasMode mode)
        {
            switch (mode)
            {
            case SasMode::Off:
                return "Off";
            case SasMode::Stability:
                return "Stability";
            case SasMode::Prograde:
                return "Prograde";
            case SasMode::Retrograde:
                return "Retrograde";
            case SasMode::Normal:
                return "Normal";
            case SasMode::AntiNormal:
                return "Anti-normal";
            case SasMode::RadialOut:
                return "Radial out";
            case SasMode::RadialIn:
                return "Radial in";
            case SasMode::Target:
                return "Target";
            case SasMode::AntiTarget:
                return "Anti-target";
            }
            return "";
        }

        // Each mode's symbol, size across from its middle: the orbit and target directions as they show on the navball,
        // and a padlock for Stability
        void drawMarker(ImDrawList* pList, SasMode mode, ImVec2 at, float size, ImU32 color)
        {
            const float s = size;
            switch (mode)
            {
            case SasMode::Prograde:
                pList->AddCircle(at, s, color, 20, 2.0f);
                pList->AddLine(offset(at, 0.0f, -s), offset(at, 0.0f, -s * 2.0f), color, 2.0f);
                pList->AddLine(offset(at, -s, 0.0f), offset(at, -s * 2.0f, 0.0f), color, 2.0f);
                pList->AddLine(offset(at, s, 0.0f), offset(at, s * 2.0f, 0.0f), color, 2.0f);
                pList->AddCircleFilled(at, 1.5f, color);
                break;
            case SasMode::Retrograde:
            {
                // The prograde symbol turned over, with a cross in the circle
                pList->AddCircle(at, s, color, 20, 2.0f);
                pList->AddLine(offset(at, -s * 0.7f, -s * 0.7f), offset(at, s * 0.7f, s * 0.7f), color, 2.0f);
                pList->AddLine(offset(at, -s * 0.7f, s * 0.7f), offset(at, s * 0.7f, -s * 0.7f), color, 2.0f);
                for (float angle : { PI * 0.5f, PI * 1.25f, PI * 1.75f })
                {
                    const float c = std::cos(angle);
                    const float d = std::sin(angle);
                    pList->AddLine(offset(at, c * s, d * s), offset(at, c * s * 2.0f, d * s * 2.0f), color, 2.0f);
                }
                break;
            }
            case SasMode::Normal:
                pList->AddTriangle(offset(at, 0.0f, -s * 1.3f), offset(at, -s * 1.15f, s * 0.75f), offset(at, s * 1.15f, s * 0.75f), color, 2.0f);
                pList->AddCircleFilled(at, 1.5f, color);
                break;
            case SasMode::AntiNormal:
            {
                // The normal triangle turned over, with spokes out of its corners
                const ImVec2 corners[] = { offset(at, 0.0f, s * 1.1f), offset(at, -s * 0.95f, -s * 0.55f), offset(at, s * 0.95f, -s * 0.55f) };
                pList->AddTriangle(corners[0], corners[1], corners[2], color, 2.0f);
                for (const ImVec2& corner : corners)
                {
                    pList->AddLine(corner, { at.x + (corner.x - at.x) * 1.7f, at.y + (corner.y - at.y) * 1.7f }, color, 2.0f);
                }
                break;
            }
            case SasMode::RadialOut:
                pList->AddCircle(at, s, color, 20, 2.0f);
                pList->AddCircleFilled(at, 1.5f, color);
                for (int corner = 0; corner < 4; corner++)
                {
                    const float angle = PI * 0.25f + PI * 0.5f * corner;
                    const float c = std::cos(angle);
                    const float d = std::sin(angle);
                    pList->AddLine(offset(at, c * s, d * s), offset(at, c * s * 1.9f, d * s * 1.9f), color, 2.0f);
                }
                break;
            case SasMode::RadialIn:
                // Spokes pointing in toward the middle, square on so they don't read as retrograde's cross
                pList->AddCircle(at, s, color, 20, 2.0f);
                for (int side = 0; side < 4; side++)
                {
                    const float angle = PI * 0.5f * side;
                    const float c = std::cos(angle);
                    const float d = std::sin(angle);
                    pList->AddLine(offset(at, c * s * 1.7f, d * s * 1.7f), offset(at, c * s * 0.3f, d * s * 0.3f), color, 2.0f);
                }
                break;
            case SasMode::Target:
            case SasMode::AntiTarget:
            {
                // A ring with a dot for toward or a cross for away, and four ticks round it, turned for away
                const bool away = mode == SasMode::AntiTarget;
                pList->AddCircle(at, s * 0.85f, color, 20, 2.0f);
                if (away)
                {
                    pList->AddLine(offset(at, -s * 0.55f, -s * 0.55f), offset(at, s * 0.55f, s * 0.55f), color, 2.0f);
                    pList->AddLine(offset(at, -s * 0.55f, s * 0.55f), offset(at, s * 0.55f, -s * 0.55f), color, 2.0f);
                }
                else
                {
                    pList->AddCircleFilled(at, 2.0f, color);
                }
                for (int side = 0; side < 4; side++)
                {
                    const float angle = PI * 0.5f * side + (away ? PI * 0.25f : 0.0f);
                    const float c = std::cos(angle);
                    const float d = std::sin(angle);
                    pList->AddLine(offset(at, c * s * 1.15f, d * s * 1.15f), offset(at, c * s * 1.6f, d * s * 1.6f), color, 2.0f);
                }
                break;
            }
            case SasMode::Stability:
                pList->PathLineTo(offset(at, -s * 0.55f, s * 0.2f));
                pList->PathArcTo(offset(at, 0.0f, -s * 0.25f), s * 0.55f, PI, PI * 2.0f, 12);
                pList->PathLineTo(offset(at, s * 0.55f, s * 0.2f));
                pList->PathStroke(color, 0, 2.0f);
                pList->AddRectFilled(offset(at, -s, s * 0.1f), offset(at, s, s * 1.2f), color, 2.0f);
                pList->AddCircleFilled(offset(at, 0.0f, s * 0.6f), s * 0.2f, HUD_BOX);
                break;
            case SasMode::Off:
                break;
            }
        }

        // A rocket seen from the side, nose toward angle, for the SAS dial's middle
        void drawPointer(ImDrawList* pList, ImVec2 centre, float angle, float length, ImU32 body, ImU32 trim)
        {
            const float c = std::cos(angle);
            const float s = std::sin(angle);
            auto at = [&](float along, float across)
            {
                const float x = along * length;
                const float y = across * length;
                return ImVec2{ centre.x + x * c - y * s, centre.y + x * s + y * c };
            };
            const ImVec2 hull[] = { at(0.5f, 0.0f), at(0.3f, 0.1f), at(-0.32f, 0.1f), at(-0.32f, -0.1f), at(0.3f, -0.1f) };
            pList->AddConvexPolyFilled(hull, 5, body);
            pList->AddTriangleFilled(at(-0.12f, 0.1f), at(-0.4f, 0.24f), at(-0.36f, 0.1f), trim);
            pList->AddTriangleFilled(at(-0.12f, -0.1f), at(-0.36f, -0.1f), at(-0.4f, -0.24f), trim);
            pList->AddTriangleFilled(at(-0.32f, 0.06f), at(-0.44f, 0.09f), at(-0.44f, -0.09f), trim);
            pList->AddTriangleFilled(at(-0.32f, 0.06f), at(-0.44f, -0.09f), at(-0.32f, -0.06f), trim);
            pList->AddLine(at(0.3f, 0.1f), at(0.3f, -0.1f), trim, 1.5f);
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

    FlightScene::FlightScene(const RocketDesign& design, const SolarSystem& solarSystem)
        : mDesign(design), mSolarSystem(solarSystem)
    {
        mOrbit.minPitch = -1.5f;
        mOrbit.liftWithKeys = false;
        mOrbit.maxDistance = 3000.0f;
    }

    // The sky and the planet need the window's graphics context, so they're made on first entry
    void FlightScene::onLoad()
    {
        const BodyInfo& earth = mSolarSystem.body(mSolarSystem.findBody("Earth"));
        const SurfaceFrame site = SurfaceFrame::below(SurfaceFrame::direction(PAD_LATITUDE, PAD_LONGITUDE), earth.radius);
        mSky.init();
        mPlanet.init(site, earth.radius, earth.atmosphereHeight, GROUND_COLOR);
        mGlobes.init(mSolarSystem);
        mMap.init(mSolarSystem);
    }

    void FlightScene::onUnload()
    {
        mSky.unload();
        mPlanet.unload();
        mGlobes.unload();
    }

    // The rocket rolls out as built and stands on top of pad 39A with its tanks full
    void FlightScene::onEnter()
    {
        const BodyId earth = mSolarSystem.findBody("Earth");
        const double radius = mSolarSystem.body(earth).radius;
        const Vehicle vehicle = makeVehicle(mDesign);
        mTime = 0.0;
        mWarp.stop();
        mWarp.setPaused(false);
        mGForce = 1.0;
        mLaunched = false;
        mDebris.clear();
        mPuffs.clear();
        mMapOpen = false;
        mMapFresh = true;
        mTarget = NO_BODY;
        mRocket.clearTarget();
        const Vec3d site = SurfaceFrame::direction(PAD_LATITUDE, PAD_LONGITUDE);
        mPad = SurfaceFrame::below(site, radius);
        mLaunchPad.body = earth;
        mLaunchPad.frame = mPad;
        mLaunchPad.halfWidth = PAD_WIDTH * 0.5;
        mLaunchPad.halfLength = PAD_LENGTH * 0.5;
        mLaunchPad.height = PAD_HEIGHT;
        mLaunchPad.skirt = PAD_HEIGHT * PAD_SLOPE;
        mRocket.setVehicle(vehicle);
        mRocket.setPad(mLaunchPad);
        mRocket.place(earth, site * (radius + PAD_HEIGHT + vehicle.groundClearance), {});

        mPreviousNear = rlGetCullDistanceNear();
        mPreviousFar = rlGetCullDistanceFar();
        rlSetClipPlanes(NEAR_PLANE, FAR_PLANE);

        // Frames the rocket from a little way off, so a taller rocket starts further away
        const float height = (float)(mDesign.highestPoint() - mDesign.lowestPoint());
        mOrbit.distance = std::max(14.0f, height * 1.8f);
        mOrbit.yaw = 0.6f;
        mOrbit.pitch = 0.15f;
        mOrbit.frame = QuaternionIdentity();
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
        // A tank about to run dry holds time at 1x until it's empty, so staging isn't missed at speed
        mWarp.run(
            [&]
            {
                stepPhysics(dt);
                mWarp.setHeld(secondsOfFuel(mRocket.vehicle(), mRocket.pressure()) < FUEL_WARNING);
            },
            []
            {
                return GetTime();
            },
            dt * WARP_BUDGET);
    }

    void FlightScene::stepPhysics(double dt)
    {
        const Vec3d before = mRocket.velocity();
        const BodyId reference = mRocket.reference();
        updateTarget();
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
            if (IsKeyPressed(KEY_T))
            {
                mRocket.setSas(mRocket.sas() == SasMode::Off ? SasMode::Stability : SasMode::Off);
            }
            if (IsKeyPressed(KEY_M))
            {
                toggleMap();
            }

            // Period and comma speed time up and slow it down, slash returns to real time, and P pauses
            if (IsKeyPressed(KEY_PERIOD) && mLaunched)
            {
                mWarp.faster();
            }
            if (IsKeyPressed(KEY_COMMA))
            {
                mWarp.slower();
            }
            if (IsKeyPressed(KEY_SLASH))
            {
                mWarp.stop();
            }
            if (IsKeyPressed(KEY_P) && mLaunched)
            {
                mWarp.togglePause();
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

        updateTarget();
        mSun = sunDirection();

        // A body without its own planet drawing gets a fine patch of ground under the rocket, rebuilt as it moves on
        const BodyId around = mRocket.reference();
        if (!mGlobes.usesPlanet(around))
        {
            const double radius = mSolarSystem.body(around).radius;
            const Vec3d under = bodyAxes(mSolarSystem, around, mTime).toBody(mRocket.position());
            mGlobes.placeGround(around, under, radius, std::max(GROUND_SLACK, 0.25 * (under.length() - radius)));
        }

        const Vehicle& vehicle = mRocket.vehicle();
        const double middle = (vehicle.design.highestPoint() + vehicle.design.lowestPoint()) * 0.5;
        const Vector3 target = toRaylib(mPad.rotate(mRocket.toWorld({ 0.0, middle - vehicle.centerOfMass.y, 0.0 })));
        const float follow = mSnapCamera ? 1.0f : 1.0f - std::exp(-CAMERA_EASE * (float)frameDt);
        mSnapCamera = false;
        mFocus = { mFocus.x + (target.x - mFocus.x) * follow, mFocus.y + (target.y - mFocus.y) * follow,
                   mFocus.z + (target.z - mFocus.z) * follow };
        // The orbit's up turns with the rocket's own up as it travels round the planet, so the camera keeps circling
        // above the ground wherever it is. Turning by the least amount each frame keeps the view steady over the poles.
        const Vector3 up = toRaylib(mPad.rotate(mRocket.position().normalized()));
        const Vector3 orbitUp = Vector3RotateByQuaternion({ 0.0f, 1.0f, 0.0f }, mOrbit.frame);
        mOrbit.frame = QuaternionNormalize(QuaternionMultiply(QuaternionFromVector3ToVector3(orbitUp, up), mOrbit.frame));
        const Quaternion toOrbit = QuaternionInvert(mOrbit.frame);

        const Vector3 focus = Vector3RotateByQuaternion(mFocus, toOrbit);
        mOrbit.focus = focus;
        mOrbit.minFocusHeight = focus.y;
        mOrbit.maxFocusHeight = focus.y;
        // The ground right under the rocket, or the pad's top, since far downrange the planet curves away below it
        const BodyInfo& body = mSolarSystem.body(mRocket.reference());
        const double radius = body.radius;
        const Vec3d over = mPad.toLocal(mRocket.position());

        // Out of the air the camera can turn all the way round the rocket, since there's no ground nearby to stay above
        const double space = std::max(body.atmosphereHeight, radius * FREE_LOOK_SHARE);
        mOrbit.freeTilt = mRocket.position().length() - radius > space;
        const double ground = radius + (mRocket.reference() == mLaunchPad.body ? mLaunchPad.heightAt(over.x, over.z) : 0.0);
        mOrbit.minCameraHeight = Vector3RotateByQuaternion(toScene(mRocket.position().normalized() * ground), toOrbit).y + 0.5f;
        // The map has its own camera; this one waits, still following, to pick up again when the map closes
        if (mMapOpen)
        {
            mMap.update(getCamera(), mapFlight(), (float)frameDt);
            return;
        }
        if (mZoomTarget > 0.0f)
        {
            mOrbit.distance += (mZoomTarget - mOrbit.distance) * follow;
        }
        const float easedDistance = mOrbit.distance;
        mOrbit.update(getCamera(), (float)frameDt);

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

    Vector3 FlightScene::sunDirection() const
    {
        return sunFrom(mRocket.reference());
    }

    Vector3 FlightScene::sunFrom(BodyId id) const
    {
        BodyId star = id;
        while (mSolarSystem.body(star).parent != NO_BODY)
        {
            star = mSolarSystem.body(star).parent;
        }
        const Vec3d toward = mSolarSystem.positionAt(star, mTime) - mSolarSystem.positionAt(id, mTime);
        if (toward.lengthSquared() <= 0.0)
        {
            return toRaylib(mPad.rotate({ 0.0, 1.0, 0.0 }));
        }
        return toRaylib(mPad.rotate(toward.normalized()));
    }

    void FlightScene::toggleMap()
    {
        mMapOpen = !mMapOpen;
        if (mMapOpen && mMapFresh)
        {
            mMap.reset(mapFlight());
            mMapFresh = false;
        }
        if (!mMapOpen)
        {
            rlSetClipPlanes(NEAR_PLANE, FAR_PLANE);
        }
    }

    MapView::Flight FlightScene::mapFlight() const
    {
        MapView::Flight flight;
        flight.pSystem = &mSolarSystem;
        flight.time = mTime;
        flight.reference = mRocket.reference();
        flight.position = mRocket.position();
        flight.velocity = mRocket.velocity();
        for (const Debris& debris : mDebris)
        {
            flight.debris.push_back(debris.rocket.position());
        }
        return flight;
    }

    void FlightScene::updateTarget()
    {
        if (mTarget == mRocket.reference())
        {
            mTarget = NO_BODY;
        }
        if (mTarget == NO_BODY)
        {
            if (mRocket.hasTarget())
            {
                mRocket.clearTarget();
            }
            return;
        }
        const Vec3d target = mSolarSystem.positionAt(mTarget, mTime) - mSolarSystem.positionAt(mRocket.reference(), mTime);
        mRocket.setTarget(target - mRocket.position());
    }

    Vector3 FlightScene::toScene(const Vec3d& position) const
    {
        return toRaylib(mPad.rotate(position - mRocket.position()));
    }

    double FlightScene::cameraHeight() const
    {
        const Vector3 camera = getCamera().position;
        const Vec3d at = mRocket.position() + mPad.toBody({ camera.x, camera.y, camera.z });
        return at.length() - mSolarSystem.body(mRocket.reference()).radius;
    }

    // The stars and the nearest air behind everything, then every planet and moon from the farthest in. At these
    // distances depth can't tell them apart, so each hides what's behind it by being drawn later.
    void FlightScene::drawGround() const
    {
        const BodyId reference = mRocket.reference();
        const Vec3d home = mSolarSystem.positionAt(reference, mTime);
        const Vector3 camera = getCamera().position;
        const Vec3d eye = mRocket.position() + mPad.toBody({ camera.x, camera.y, camera.z });
        const auto centreOf = [&](BodyId id)
        {
            return mSolarSystem.positionAt(id, mTime) - home; // from the reference body, like the rocket
        };

        // The sky's air is the nearest that has any, so Earth's still glows seen from the Moon
        BodyId airBody = reference;
        double airDistance = DBL_MAX;
        for (BodyId id = 0; id < mSolarSystem.bodyCount(); id++)
        {
            const double distance = (centreOf(id) - eye).length();
            if (mSolarSystem.body(id).atmosphereHeight > 0.0 && distance < airDistance)
            {
                airBody = id;
                airDistance = distance;
            }
        }
        const BodyInfo& air = mSolarSystem.body(airBody);
        SpaceSky::View sky;
        sky.axes = mPad;
        sky.centre = toScene(centreOf(airBody));
        sky.camera = camera;
        sky.sun = sunFrom(airBody);
        sky.radius = air.radius;
        sky.atmosphereHeight = air.atmosphereHeight;
        sky.cameraHeight = (eye - centreOf(airBody)).length() - air.radius;

        // The Sun stays the sky's bright disc, tinted by the air, so only planets and moons are drawn
        std::vector<BodyId> order;
        for (BodyId id = 0; id < mSolarSystem.bodyCount(); id++)
        {
            if (mSolarSystem.body(id).parent != NO_BODY)
            {
                order.push_back(id);
            }
        }
        std::sort(order.begin(), order.end(),
                  [&](BodyId a, BodyId b) { return (centreOf(a) - eye).lengthSquared() > (centreOf(b) - eye).lengthSquared(); });

        // Bodies beyond the air's planet are drawn before its air, so they're seen through it like the stars
        mSky.drawStars(sky);
        bool airDrawn = false;
        for (BodyId id : order)
        {
            const BodyInfo& info = mSolarSystem.body(id);
            const Vec3d centre = centreOf(id);
            const Vector3 sun = sunFrom(id);
            const double distance = (centre - eye).length();
            const bool beyond = id != airBody && distance > airDistance;
            if (beyond && sky.discCount < SpaceSky::MAX_DISCS)
            {
                const Vector3 toward = Vector3Normalize(toScene(centre) - camera);
                const float across = (float)std::cos(std::asin(std::min(info.radius / distance, 1.0)));
                sky.discs[sky.discCount++] = { toward.x, toward.y, toward.z, across };
            }
            if (!beyond && !airDrawn)
            {
                rlEnableDepthTest();
                mSky.drawAir(sky);
                airDrawn = true;
            }
            rlDrawRenderBatchActive();
            rlDisableDepthTest();
            if (mGlobes.usesPlanet(id))
            {
                // Only the body the rocket is around gets its patch of ground near the pad
                PlanetSurface::View view;
                view.axes = mPad;
                view.centre = toScene(centre);
                view.camera = camera;
                view.sun = sun;
                view.cameraHeight = (eye - centre).length() - info.radius;
                view.ground = id == reference;
                mPlanet.draw(view);
                continue;
            }
            const BodyAxes axes = bodyAxes(mSolarSystem, id, mTime);
            const auto turn = [&](const Vec3d& v)
            {
                return mPad.rotate(axes.fromBody(v));
            };
            const double r = info.radius;
            mGlobes.drawGlobe(id, placedMatrix(turn({ r, 0.0, 0.0 }), turn({ 0.0, r, 0.0 }), turn({ 0.0, 0.0, r }), toScene(centre)), sun);
            if (id == reference && mGlobes.groundBody() == id)
            {
                // The patch under the rocket covers the coarse sphere, and the rocket and everything after it can stand on it
                rlEnableDepthTest();
                const SurfaceFrame& site = mGlobes.groundSite();
                const Vector3 at = toScene(axes.fromBody(site.origin));
                mGlobes.drawGround(placedMatrix(turn(site.east), turn(site.up), turn(site.south), at), sun);
            }
        }
        rlEnableDepthTest();
        if (!airDrawn)
        {
            mSky.drawAir(sky);
        }
    }

    void FlightScene::drawPad() const
    {
        if (mRocket.reference() != mLaunchPad.body)
        {
            return;
        }
        const Vector3 pad = toScene(mPad.origin);
        const float height = (float)mLaunchPad.height;
        const float halfWidth = (float)mLaunchPad.halfWidth;
        const float halfLength = (float)mLaunchPad.halfLength;
        const float footWidth = halfWidth + (float)mLaunchPad.skirt;
        const float footLength = halfLength + (float)mLaunchPad.skirt;

        // Corners going round, each sloped side shaded by how squarely it faces the sun
        const Vector3 top[4] = { pad + Vector3{ -halfWidth, height, -halfLength }, pad + Vector3{ -halfWidth, height, halfLength },
                                 pad + Vector3{ halfWidth, height, halfLength }, pad + Vector3{ halfWidth, height, -halfLength } };
        const Vector3 foot[4] = { pad + Vector3{ -footWidth, -PAD_BURY, -footLength }, pad + Vector3{ -footWidth, -PAD_BURY, footLength },
                                  pad + Vector3{ footWidth, -PAD_BURY, footLength }, pad + Vector3{ footWidth, -PAD_BURY, -footLength } };
        const Vector3 sun = Vector3Normalize(mSun);
        for (int i = 0; i < 4; i++)
        {
            const int j = (i + 1) % 4;
            const Vector3 facing = Vector3Normalize(Vector3CrossProduct(foot[j] - foot[i], top[j] - foot[i]));
            const Color side = ColorBrightness(PAD_SIDE_COLOR, 0.35f * std::max(Vector3DotProduct(facing, sun), 0.0f) - 0.25f);
            DrawTriangle3D(foot[i], foot[j], top[j], side);
            DrawTriangle3D(foot[i], top[j], top[i], side);
        }
        const Vector3 lift = { 0.0f, PAD_LIFT, 0.0f };
        for (int i = 0; i < 4; i++)
        {
            DrawLine3D(top[i] + lift, top[(i + 1) % 4] + lift, ColorBrightness(PAD_COLOR, -0.3f));
        }

        // The flame trench's cover runs north to south under the rocket. The concrete is laid round it rather than
        // under it, so the two never overlap to flicker against each other.
        const Vector3 deck = pad + Vector3{ 0.0f, height, 0.0f };
        const float trenchX = TRENCH_WIDTH * 0.5f;
        const float trenchZ = TRENCH_LENGTH * 0.5f;
        const auto slab = [&](float x0, float z0, float x1, float z1, Color color)
        {
            DrawTriangle3D(deck + Vector3{ x0, 0.0f, z0 }, deck + Vector3{ x0, 0.0f, z1 }, deck + Vector3{ x1, 0.0f, z1 }, color);
            DrawTriangle3D(deck + Vector3{ x0, 0.0f, z0 }, deck + Vector3{ x1, 0.0f, z1 }, deck + Vector3{ x1, 0.0f, z0 }, color);
        };
        slab(-halfWidth, -halfLength, -trenchX, halfLength, PAD_COLOR);
        slab(trenchX, -halfLength, halfWidth, halfLength, PAD_COLOR);
        slab(-trenchX, -halfLength, trenchX, -trenchZ, PAD_COLOR);
        slab(-trenchX, trenchZ, trenchX, halfLength, PAD_COLOR);
        slab(-trenchX, -trenchZ, trenchX, trenchZ, TRENCH_COLOR);

        // The service tower beside the rocket, a little taller than it
        const float towerHeight = (float)(mDesign.highestPoint() - mDesign.lowestPoint()) + 4.0f;
        const Vector3 tower = deck + Vector3{ TOWER_POSITION.x, towerHeight * 0.5f, TOWER_POSITION.z };
        DrawCube(tower, TOWER_WIDTH, towerHeight, TOWER_WIDTH, TOWER_COLOR);
        DrawCubeWires(tower, TOWER_WIDTH, towerHeight, TOWER_WIDTH, ColorBrightness(TOWER_COLOR, -0.5f));
    }

    void FlightScene::onDraw3D()
    {
        if (mMapOpen)
        {
            mMap.draw(getCamera(), mapFlight(), mPlanet, mSky, mGlobes);
            return;
        }
        drawGround();
        drawPad();

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
        if (mMapOpen)
        {
            mMap.drawOverlay(getCamera(), mapFlight(), mTarget);
            updateTarget();
        }
        drawStatusBar();
        drawClock();
        drawInstruments();
        drawSas();
        drawSasSwitch();
        drawSasControl();
        drawOrbit();
        drawGForce();
        drawStageStack();
        drawGameView();
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
                switchTo(scenes::BUILD);
            }
        }
        if (mTarget != NO_BODY)
        {
            // How far off the target is, and how fast that gap is closing
            const Vec3d targetPosition = mSolarSystem.positionAt(mTarget, mTime) - mSolarSystem.positionAt(mRocket.reference(), mTime);
            const Vec3d targetVelocity = mSolarSystem.velocityAt(mTarget, mTime) - mSolarSystem.velocityAt(mRocket.reference(), mTime);
            const Vec3d gap = targetPosition - mRocket.position();
            const double closing = (mRocket.velocity() - targetVelocity).dot(gap.normalized());
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(HUD_PURPLE), "Target");
            ImGui::SameLine();
            ImGui::Text("%s   %s km   %s %s m/s", mSolarSystem.body(mTarget).name.c_str(), grouped(gap.length() / 1000.0).c_str(),
                        closing >= 0.0 ? "closing" : "opening", grouped(std::abs(closing)).c_str());
        }
        ImGui::TextDisabled("Space stage   WASD/QE steer   T SAS   Shift/Ctrl throttle   Z full   X cut   , . / warp   P pause   M map   Right-drag look");
        ImGui::End();
    }

    // Bottom middle: the mission clock beside a pause button, and under it a row of arrows for time warp that light up
    // to the chosen rate. The caption shows the rate, and the rate really reached when the computer can't keep up.
    void FlightScene::drawClock()
    {
        const ImVec2 min = clockCorner();
        const ImVec2 max = offset(min, CLOCK_SIZE.x, CLOCK_SIZE.y);
        ImGui::SetNextWindowPos(min, ImGuiCond_Always);
        ImGui::SetNextWindowSize(CLOCK_SIZE, ImGuiCond_Always);
        ImGui::Begin("Clock", nullptr, OVERLAY_FLAGS);
        ImDrawList* pList = ImGui::GetWindowDrawList();
        const bool running = mLaunched && !mWarp.paused();
        panel(pList, min, max, !mLaunched ? HUD_EDGE : (mWarp.paused() ? HUD_YELLOW : HUD_GREEN));

        // The pause button shows two bars while time runs and a play arrow while it's stopped
        const ImVec2 pause = offset(min, 12.0f + PAUSE_RADIUS, 6.0f + PAUSE_RADIUS);
        const ImU32 pauseInk = !mLaunched ? HUD_DIM : (mWarp.paused() ? HUD_YELLOW : HUD_GREEN);
        pList->AddCircleFilled(pause, PAUSE_RADIUS, HUD_BOX, 24);
        pList->AddCircle(pause, PAUSE_RADIUS, pauseInk, 24, 1.5f);
        if (mWarp.paused())
        {
            pList->AddTriangleFilled(offset(pause, -4.0f, -6.0f), offset(pause, 6.0f, 0.0f), offset(pause, -4.0f, 6.0f), pauseInk);
        }
        else
        {
            pList->AddRectFilled(offset(pause, -5.0f, -6.0f), offset(pause, -1.5f, 6.0f), pauseInk);
            pList->AddRectFilled(offset(pause, 1.5f, -6.0f), offset(pause, 5.0f, 6.0f), pauseInk);
        }
        ImGui::SetCursorScreenPos(offset(pause, -PAUSE_RADIUS, -PAUSE_RADIUS));
        if (ImGui::InvisibleButton("pause", { PAUSE_RADIUS * 2.0f, PAUSE_RADIUS * 2.0f }) && mLaunched)
        {
            mWarp.togglePause();
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Pause (P)");
        }

        text(pList, offset(min, 24.0f + PAUSE_RADIUS * 2.0f, 9.0f), 16.0f, mLaunched ? HUD_GREEN : HUD_MUTED, "T+");
        textRight(pList, max.x - 12.0f, min.y + 6.0f, 26.0f, HUD_TEXT, clockText(mTime).c_str());

        // Each arrow sets its rate when clicked; those up to the chosen one are lit. While a tank is about to run dry,
        // all but real time are greyed out.
        const float row = min.y + 49.0f;
        for (int i = 0; i < (int)TimeWarp::RATES.size(); i++)
        {
            const float left = min.x + 14.0f + WARP_SPACING * i;
            const bool lit = running && i <= mWarp.level();
            const bool open = mLaunched && (i == 0 || !mWarp.held());
            const ImU32 ink = !open ? HUD_DIM : (lit ? HUD_GREEN : HUD_MUTED);
            pList->AddTriangleFilled({ left, row - WARP_ARROW }, { left + WARP_ARROW * 1.6f, row }, { left, row + WARP_ARROW }, lit ? ink : HUD_DIM);
            pList->AddTriangle({ left, row - WARP_ARROW }, { left + WARP_ARROW * 1.6f, row }, { left, row + WARP_ARROW }, ink, 1.0f);
            ImGui::SetCursorScreenPos({ left - 3.0f, row - WARP_ARROW - 3.0f });
            ImGui::PushID(i);
            if (ImGui::InvisibleButton("warp", { WARP_SPACING, WARP_ARROW * 2.0f + 6.0f }) && open)
            {
                mWarp.setLevel(i);
                mWarp.setPaused(false);
            }
            ImGui::PopID();
            if (ImGui::IsItemHovered())
            {
                if (mWarp.held() && i > 0)
                {
                    ImGui::SetTooltip("Held at 1x: a burning stage runs dry in under %.0f s", FUEL_WARNING);
                }
                else
                {
                    ImGui::SetTooltip("%dx   (. faster, , slower, / real time)", TimeWarp::RATES[i]);
                }
            }
        }

        char value[48];
        if (mWarp.paused())
        {
            std::snprintf(value, sizeof(value), "PAUSED");
        }
        else if (mWarp.achieved() < mWarp.rate() * 0.9)
        {
            std::snprintf(value, sizeof(value), "%dx (%.0fx)", mWarp.rate(), mWarp.achieved());
        }
        else
        {
            std::snprintf(value, sizeof(value), "%dx", mWarp.rate());
        }
        const ImU32 rateInk = mWarp.held() && running ? HUD_ORANGE : (mWarp.rate() > 1 && running ? HUD_YELLOW : HUD_TEXT);
        textRight(pList, max.x - 12.0f, row - 8.0f, 16.0f, rateInk, mWarp.held() && running ? "LOW FUEL  1x" : value);
        caption(pList, min, max, "TIME WARP");
        ImGui::End();
    }

    // Bottom left: the navball in a ring carrying the heading tape, the throttle arc to its left, the climb rate arc
    // to its right, and the speed and altitude boxed either side
    void FlightScene::drawInstruments()
    {
        const float radius = NAVBALL_RADIUS;
        const ImVec2 centre = navballCentre();
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

        // The orbit's directions and the target's, where the SAS can point the nose
        for (SasMode mode : { SasMode::Prograde, SasMode::Retrograde, SasMode::Normal, SasMode::AntiNormal, SasMode::RadialOut, SasMode::RadialIn,
                              SasMode::Target, SasMode::AntiTarget })
        {
            Vec3d toward;
            ImVec2 at;
            if (mRocket.holdDirection(mode, toward) && onBall(toward, at))
            {
                drawMarker(pList, mode, at, MARKER_SIZE, markerColor(mode));
            }
        }

        // A diamond round wherever the SAS is steering the nose
        Vec3d held;
        ImVec2 heldAt;
        if (mRocket.sas() != SasMode::Off && mRocket.holdDirection(mRocket.sas(), held) && onBall(held, heldAt))
        {
            const ImVec2 diamond[] = { offset(heldAt, 0.0f, -13.0f), offset(heldAt, 13.0f, 0.0f), offset(heldAt, 0.0f, 13.0f), offset(heldAt, -13.0f, 0.0f) };
            pList->AddPolyline(diamond, 4, HUD_GREEN, ImDrawFlags_Closed, 2.0f);
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

        // Three tapes share the ring: heading round the top, speed up the left, and altitude up the right
        auto whole = [](double mark)
        {
            char label[16];
            std::snprintf(label, sizeof(label), "%.0f", mark);
            return std::string(label);
        };
        if (hasHeading)
        {
            drawTape(pList, centre, radius, -PI * 0.5f, HEADING_SWEEP, heading, 20.0, 2, -1.0e9,
                     [&](double mark)
                     {
                         // The label under the heading box would hide behind its pointer
                         return std::abs(mark - heading) > 8.0 ? whole(std::fmod(std::fmod(mark, 360.0) + 360.0, 360.0)) : std::string();
                     });
        }
        const double speedNow = mRocket.velocity().length();
        const double heightNow = std::max(0.0, mRocket.altitude(mSolarSystem));
        drawTape(pList, centre, radius, PI, SIDE_SWEEP, speedNow, tapeStep(speedNow), 5, 0.0, whole);
        const double heightStep = tapeStep(heightNow);
        drawTape(pList, centre, radius, 0.0f, -SIDE_SWEEP, heightNow, heightStep, 5, 0.0,
                 [&](double mark)
                 {
                     return heightStep >= 1000.0 ? whole(mark / 1000.0) + "k" : whole(mark);
                 });
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
        const double speed = mRocket.velocity().length();
        std::snprintf(value, sizeof(value), "%.1f", speed);
        sideBox(pList, offset(centre, -100.0f, 0.0f), HUD_YELLOW, "SURFACE", value, "m/s");
        const char* unit = "m";
        writeHeight(value, sizeof(value), std::max(0.0, mRocket.altitude(mSolarSystem)), unit);
        const double atmosphere = mSolarSystem.body(mRocket.reference()).atmosphereHeight;
        const float air = atmosphere > 0.0 ? (float)(mRocket.altitude(mSolarSystem) / atmosphere) : 1.0f;
        sideBox(pList, offset(centre, 100.0f, 0.0f), HUD_PINK, "ALTITUDE", value, unit, air);
        ImGui::End();
    }

    // Right of the navball: a ring of the six orbit directions the SAS can hold, each opposite its reverse, round a
    // rocket that turns to point at the chosen one.
    void FlightScene::drawSas()
    {
        const ImVec2 centre = offset(navballCentre(), SAS_DIAL_OFFSET.x, SAS_DIAL_OFFSET.y);
        const ImVec2 min = offset(centre, -SAS_DIAL_RADIUS, -SAS_DIAL_RADIUS);
        const float size = SAS_DIAL_RADIUS * 2.0f;
        ImGui::SetNextWindowPos(min, ImGuiCond_Always);
        ImGui::SetNextWindowSize({ size, size }, ImGuiCond_Always);
        ImGui::Begin("SAS", nullptr, OVERLAY_FLAGS);
        ImDrawList* pList = ImGui::GetWindowDrawList();
        const SasMode chosen = mRocket.sas();
        const bool on = chosen != SasMode::Off;
        pList->AddCircleFilled(centre, SAS_DIAL_RADIUS, HUD_PANEL, 64);
        pList->AddCircle(centre, SAS_DIAL_RADIUS, HUD_EDGE, 64, 1.5f);
        pList->AddCircle(centre, SAS_RING, HUD_DIM, 64, 1.0f);

        struct Slot
        {
            SasMode mode;
            float angle; // round the ring from the right, downward
        };
        const Slot slots[] = { { SasMode::Prograde, -PI * 0.5f },       { SasMode::Retrograde, PI * 0.5f }, { SasMode::RadialOut, -PI / 6.0f },
                               { SasMode::RadialIn, PI * 5.0f / 6.0f }, { SasMode::Normal, -PI * 5.0f / 6.0f }, { SasMode::AntiNormal, PI / 6.0f } };
        float pointTo = -PI * 0.5f;
        for (const Slot& slot : slots)
        {
            const ImVec2 at = offset(centre, std::cos(slot.angle) * SAS_RING, std::sin(slot.angle) * SAS_RING);
            const bool picked = chosen == slot.mode;
            if (picked)
            {
                pointTo = slot.angle;
            }
            pList->AddCircleFilled(at, SAS_BUTTON, picked ? IM_COL32(30, 70, 45, 255) : HUD_BOX, 24);
            pList->AddCircle(at, SAS_BUTTON, picked ? HUD_GREEN : HUD_DIM, 24, picked ? 2.0f : 1.5f);
            drawMarker(pList, slot.mode, at, MARKER_SIZE, markerColor(slot.mode));
            ImGui::SetCursorScreenPos(offset(at, -SAS_BUTTON, -SAS_BUTTON));
            if (ImGui::InvisibleButton(modeName(slot.mode), { SAS_BUTTON * 2.0f, SAS_BUTTON * 2.0f }))
            {
                mRocket.setSas(slot.mode);
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%s", modeName(slot.mode));
            }
        }

        // The rocket swings the short way round to its new mode
        float swing = std::remainder(pointTo - mSasPointer, 2.0f * PI);
        mSasPointer += swing * std::min(1.0f, GetFrameTime() * SAS_POINTER_EASE);
        pList->AddCircleFilled(centre, SAS_HUB, HUD_BOX, 48);
        pList->AddCircle(centre, SAS_HUB, on ? HUD_GREEN : HUD_EDGE, 48, 1.5f);
        drawPointer(pList, centre, mSasPointer, SAS_HUB * 1.6f, on ? HUD_TEXT : HUD_MUTED, on ? HUD_GREEN : HUD_DIM);
        ImGui::End();
    }

    // Under the dial: Stability, toward and away from the target, and a mode that will aim at a planned maneuver once
    // those exist
    void FlightScene::drawSasControl()
    {
        const ImVec2 dial = offset(navballCentre(), SAS_DIAL_OFFSET.x, SAS_DIAL_OFFSET.y);
        const ImVec2 min = { dial.x - SAS_TAB_SIZE.x * 0.5f, dial.y + SAS_DIAL_RADIUS + 6.0f };
        const ImVec2 max = offset(min, SAS_TAB_SIZE.x, SAS_TAB_SIZE.y);
        ImGui::SetNextWindowPos(min, ImGuiCond_Always);
        ImGui::SetNextWindowSize(SAS_TAB_SIZE, ImGuiCond_Always);
        ImGui::Begin("SAS control", nullptr, OVERLAY_FLAGS);
        ImDrawList* pList = ImGui::GetWindowDrawList();
        const SasMode chosen = mRocket.sas();
        panel(pList, min, max, HUD_EDGE);
        caption(pList, min, max, chosen != SasMode::Off ? modeName(chosen) : "SAS OFF");

        // The symbol for aiming at a maneuver, drawn here since that isn't a mode yet
        auto maneuver = [&](ImVec2 at, ImU32 color)
        {
            pList->AddCircle(at, 6.0f, color, 20, 2.0f);
            for (float angle : { -PI * 0.5f, PI / 6.0f, PI * 5.0f / 6.0f })
            {
                pList->AddLine(offset(at, std::cos(angle) * 6.0f, std::sin(angle) * 6.0f), offset(at, std::cos(angle) * 11.0f, std::sin(angle) * 11.0f), color, 3.0f);
            }
        };
        const bool targeted = mRocket.hasTarget();
        const char* tips[] = { "Stability: hold the current heading", "Maneuver: needs a planned maneuver",
                               targeted ? "Target: point at the target" : "Target: pick a target on the map (M)",
                               targeted ? "Anti-target: point away from the target" : "Anti-target: pick a target on the map (M)" };
        const SasMode modes[] = { SasMode::Stability, SasMode::Off, SasMode::Target, SasMode::AntiTarget };
        const float width = (SAS_TAB_SIZE.x - 16.0f) / 4.0f;
        for (int i = 0; i < 4; i++)
        {
            const ImVec2 at = { min.x + 8.0f + width * (i + 0.5f), min.y + 19.0f };
            const bool picked = modes[i] != SasMode::Off && chosen == modes[i];
            pList->AddRectFilled(offset(at, -width * 0.5f + 2.0f, -14.0f), offset(at, width * 0.5f - 2.0f, 14.0f), picked ? IM_COL32(30, 70, 45, 255) : HUD_BOX, 4.0f);
            pList->AddRect(offset(at, -width * 0.5f + 2.0f, -14.0f), offset(at, width * 0.5f - 2.0f, 14.0f), picked ? HUD_GREEN : HUD_DIM, 4.0f, 0, picked ? 2.0f : 1.0f);
            if (i == 0)
            {
                drawMarker(pList, SasMode::Stability, at, MARKER_SIZE, HUD_GREEN);
            }
            else if (i == 1)
            {
                maneuver(at, HUD_DIM);
            }
            else
            {
                drawMarker(pList, modes[i], at, MARKER_SIZE, targeted ? HUD_PURPLE : HUD_DIM);
            }
            ImGui::SetCursorScreenPos(offset(at, -width * 0.5f + 2.0f, -14.0f));
            ImGui::PushID(i);
            if (ImGui::InvisibleButton("mode", { width - 4.0f, 28.0f }) && modes[i] != SasMode::Off)
            {
                mRocket.setSas(modes[i]);
            }
            ImGui::PopID();
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%s", tips[i]);
            }
        }
        ImGui::End();
    }

    // The SAS's on and off button, set into the navball's ring
    void FlightScene::drawSasSwitch()
    {
        const ImVec2 navball = navballCentre();
        const float ring = (NAVBALL_RADIUS + NAVBALL_BEZEL) * 0.5f;
        const ImVec2 centre = offset(navball, std::cos(SAS_SWITCH_ANGLE) * ring, std::sin(SAS_SWITCH_ANGLE) * ring);
        ImGui::SetNextWindowPos(offset(centre, -SAS_SWITCH, -SAS_SWITCH), ImGuiCond_Always);
        ImGui::SetNextWindowSize({ SAS_SWITCH * 2.0f, SAS_SWITCH * 2.0f }, ImGuiCond_Always);
        ImGui::Begin("SAS switch", nullptr, OVERLAY_FLAGS);
        ImDrawList* pList = ImGui::GetWindowDrawList();
        const bool on = mRocket.sas() != SasMode::Off;
        pList->AddCircleFilled(centre, SAS_SWITCH, on ? HUD_GO : HUD_BOX, 32);
        pList->AddCircle(centre, SAS_SWITCH, on ? HUD_GO_EDGE : HUD_EDGE, 32, 1.5f);
        textCentred(pList, centre, 11.0f, on ? HUD_GO_INK : HUD_MUTED, "SAS");
        ImGui::SetCursorScreenPos(offset(centre, -SAS_SWITCH, -SAS_SWITCH));
        if (ImGui::InvisibleButton("sas", { SAS_SWITCH * 2.0f, SAS_SWITCH * 2.0f }))
        {
            mRocket.setSas(on ? SasMode::Off : SasMode::Stability);
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("SAS on or off (T)");
        }
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

    // Right of the clock: a button for the view following the rocket and one for the map, the current one lit
    void FlightScene::drawGameView()
    {
        const ImVec2 clock = clockCorner();
        const ImVec2 min = { clock.x + CLOCK_SIZE.x + 8.0f, (float)GetScreenHeight() - GAME_VIEW_SIZE.y - HUD_MARGIN };
        const ImVec2 max = offset(min, GAME_VIEW_SIZE.x, GAME_VIEW_SIZE.y);
        ImGui::SetNextWindowPos(min, ImGuiCond_Always);
        ImGui::SetNextWindowSize(GAME_VIEW_SIZE, ImGuiCond_Always);
        ImGui::Begin("Game view", nullptr, OVERLAY_FLAGS);
        ImDrawList* pList = ImGui::GetWindowDrawList();
        panel(pList, min, max, HUD_EDGE);
        caption(pList, min, max, "GAME VIEW");
        const float width = (GAME_VIEW_SIZE.x - 16.0f) / 2.0f;
        for (int i = 0; i < 2; i++)
        {
            const bool map = i == 1;
            const bool picked = map == mMapOpen;
            const ImVec2 at = { min.x + 8.0f + width * (i + 0.5f), min.y + 19.0f };
            const ImVec2 corner = offset(at, -width * 0.5f + 2.0f, -14.0f);
            pList->AddRectFilled(corner, offset(at, width * 0.5f - 2.0f, 14.0f), picked ? IM_COL32(30, 70, 45, 255) : HUD_BOX, 4.0f);
            pList->AddRect(corner, offset(at, width * 0.5f - 2.0f, 14.0f), picked ? HUD_GREEN : HUD_DIM, 4.0f, 0, picked ? 2.0f : 1.0f);
            const ImU32 ink = picked ? HUD_GREEN : HUD_MUTED;
            if (map)
            {
                // A planet with an orbit round it
                pList->AddCircleFilled(at, 4.0f, ink, 16);
                pList->AddEllipse(at, { 11.0f, 5.0f }, ink, -0.35f, 24, 1.5f);
            }
            else
            {
                drawPointer(pList, at, -PI * 0.5f, 22.0f, ink, ink);
            }
            ImGui::SetCursorScreenPos(corner);
            ImGui::PushID(i);
            if (ImGui::InvisibleButton("view", { width - 4.0f, 28.0f }) && !picked)
            {
                toggleMap();
            }
            ImGui::PopID();
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%s", map ? "Map (M)" : "Flight (M)");
            }
        }
        ImGui::End();
    }
}
