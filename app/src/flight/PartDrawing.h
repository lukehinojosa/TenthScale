#pragma once

#include <raylib.h>

#include "flight/PartDefinition.h"
#include "flight/RocketDesign.h"
#include "shared/Vec3d.h"

namespace tenth
{
    inline Vector3 toRaylib(const Vec3d& v)
    {
        return { (float)v.x, (float)v.y, (float)v.z };
    }

    Color categoryColor(PartCategory category);

    // Draws a part centred on position; angle turns box parts to face out from the side they're on
    void drawPart(const PartShape& shape, Vector3 position, float angle, Color color, Color outline);

    // Draws a whole rocket in its parts' colours, its root at offset
    void drawDesign(const RocketDesign& design, Vector3 offset);

    // An axis-aligned box around the part, for picking it with the mouse
    BoundingBox partBounds(const PartShape& shape, Vector3 position, float angle);
}
