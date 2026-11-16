#include "flight/PartDrawing.h"

#include <algorithm>
#include <cmath>

#include <rlgl.h>

namespace tenth
{
    namespace
    {
        constexpr int ROUND_SIDES = 32;
    }

    Color categoryColor(PartCategory category)
    {
        switch (category)
        {
        case PartCategory::Pod:
            return { 210, 214, 220, 255 };
        case PartCategory::Tank:
            return { 232, 140, 52, 255 };
        case PartCategory::Engine:
            return { 96, 100, 112, 255 };
        case PartCategory::Coupling:
            return { 226, 196, 64, 255 };
        case PartCategory::Aero:
            return { 70, 132, 214, 255 };
        case PartCategory::Utility:
            return { 96, 176, 112, 255 };
        }
        return GRAY;
    }

    void drawPart(const PartShape& shape, Vector3 position, float angle, Color color, Color outline)
    {
        const float halfHeight = (float)shape.height * 0.5f;
        if (shape.kind == ShapeKind::Round)
        {
            const Vector3 bottom = { position.x, position.y - halfHeight, position.z };
            const Vector3 top = { position.x, position.y + halfHeight, position.z };
            const float bottomRadius = (float)shape.bottomDiameter * 0.5f;
            const float topRadius = (float)shape.topDiameter * 0.5f;
            DrawCylinderEx(bottom, top, bottomRadius, topRadius, ROUND_SIDES, color);
            DrawCylinderWiresEx(bottom, top, bottomRadius, topRadius, ROUND_SIDES, outline);
            return;
        }

        // Box parts reach out along their local +Z, which the angle turns to face away from the parent
        rlPushMatrix();
        rlTranslatef(position.x, position.y, position.z);
        rlRotatef(angle * RAD2DEG, 0.0f, 1.0f, 0.0f);
        DrawCube({ 0.0f, 0.0f, 0.0f }, (float)shape.thickness, (float)shape.height, (float)shape.span, color);
        DrawCubeWires({ 0.0f, 0.0f, 0.0f }, (float)shape.thickness, (float)shape.height, (float)shape.span, outline);
        rlPopMatrix();
    }

    void drawDesign(const RocketDesign& design, Vector3 offset)
    {
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            const PlacedPart& placed = design.part(i);
            const Color color = categoryColor(placed.pDefinition->category);
            const Vector3 position = { offset.x + (float)placed.position.x, offset.y + (float)placed.position.y,
                                       offset.z + (float)placed.position.z };
            drawPart(placed.shape, position, (float)placed.angle, color, ColorBrightness(color, -0.5f));
        }
    }

    BoundingBox partBounds(const PartShape& shape, Vector3 position, float angle)
    {
        const float halfHeight = (float)shape.height * 0.5f;
        float halfX = 0.0f;
        float halfZ = 0.0f;
        if (shape.kind == ShapeKind::Round)
        {
            halfX = (float)std::max(shape.topDiameter, shape.bottomDiameter) * 0.5f;
            halfZ = halfX;
        }
        else
        {
            const float c = std::abs(std::cos(angle));
            const float s = std::abs(std::sin(angle));
            halfX = (c * (float)shape.thickness + s * (float)shape.span) * 0.5f;
            halfZ = (s * (float)shape.thickness + c * (float)shape.span) * 0.5f;
        }
        return { { position.x - halfX, position.y - halfHeight, position.z - halfZ },
                 { position.x + halfX, position.y + halfHeight, position.z + halfZ } };
    }
}
