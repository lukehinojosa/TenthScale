#pragma once

#include <raylib.h>

#include "flight/PartDefinition.h"
#include "shared/Vec3d.h"

namespace tenth
{
    inline Vector3 toRaylib(const Vec3d& v)
    {
        return { (float)v.x, (float)v.y, (float)v.z };
    }

    Color categoryColor(PartCategory category);

    // Draws a part centred on position; angle turns box parts to face out from the side they're on
    void drawPart(const PartDefinition& definition, Vector3 position, float angle, Color color, Color outline);

    // An axis-aligned box around the part, for picking it with the mouse
    BoundingBox partBounds(const PartDefinition& definition, Vector3 position, float angle);
}
