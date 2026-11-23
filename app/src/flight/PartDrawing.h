#pragma once

#include <vector>

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

    // The part's shape as drawn: a shrouded engine is a cylinder as wide as its top
    PartShape drawnShape(const RocketDesign& design, PartIndex index);

    // Draws a whole rocket in its parts' colours, its root at offset. Engines lean by their gimbal, if given.
    void drawDesign(const RocketDesign& design, Vector3 offset, const std::vector<Vec3d>* pGimbal = nullptr);

    // An axis-aligned box around the part, for picking it with the mouse
    BoundingBox partBounds(const PartShape& shape, Vector3 position, float angle);
}
