#include "flight/StageSymbol.h"

#include <raylib.h>

namespace tenth
{
    void drawStageSymbol(ImDrawList* pList, ImVec2 min, float size, const PartDefinition& definition, ImU32 color)
    {
        auto at = [min, size](float x, float y)
        {
            return ImVec2(min.x + x * size, min.y + y * size);
        };
        if (definition.isEngine())
        {
            pList->AddRectFilled(at(0.38f, 0.14f), at(0.62f, 0.34f), color);
            pList->AddQuadFilled(at(0.40f, 0.34f), at(0.60f, 0.34f), at(0.78f, 0.84f), at(0.22f, 0.84f), color);
        }
        else if (definition.separates)
        {
            pList->AddRectFilled(at(0.16f, 0.28f), at(0.84f, 0.42f), color);
            pList->AddRectFilled(at(0.16f, 0.58f), at(0.84f, 0.72f), color);
        }
        else if (definition.hasParachute)
        {
            pList->PathArcTo(at(0.5f, 0.5f), size * 0.32f, PI, 2.0f * PI, 16);
            pList->PathFillConvex(color);
            pList->AddLine(at(0.18f, 0.5f), at(0.5f, 0.86f), color, 1.5f);
            pList->AddLine(at(0.82f, 0.5f), at(0.5f, 0.86f), color, 1.5f);
            pList->AddLine(at(0.5f, 0.5f), at(0.5f, 0.86f), color, 1.5f);
        }
    }
}
