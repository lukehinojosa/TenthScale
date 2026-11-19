#pragma once

#include <imgui.h>

#include "flight/PartDefinition.h"

namespace tenth
{
    // A symbol for what the part does when staged: an engine bell, a decoupler's split, or a parachute canopy.
    // Drawn in a square of the given size with its top left corner at min.
    void drawStageSymbol(ImDrawList* pList, ImVec2 min, float size, const PartDefinition& definition, ImU32 color);
}
