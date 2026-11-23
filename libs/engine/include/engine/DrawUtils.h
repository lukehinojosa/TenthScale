#pragma once

#include <raylib.h>

namespace engine
{
    // Stand-ins for raylib's DrawModelWires and DrawModelWiresEx that work in the web build too. raylib's versions
    // switch the GPU to outline mode, which WebGL lacks, so in the browser they draw solid shapes instead. These draw
    // every triangle's edges as lines, in the given colour, using the mesh data raylib keeps in memory.
    void drawModelWires(const Model& model, Vector3 position, float scale, Color color);
    void drawModelWiresEx(const Model& model, Vector3 position, Vector3 rotationAxis, float rotationAngle, Vector3 scale, Color color);

    // One mesh placed by a transform, such as a model's transform
    void drawMeshWires(const Mesh& mesh, const Matrix& transform, Color color);
}
