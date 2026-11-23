#include "engine/DrawUtils.h"

#include <raymath.h>

namespace engine
{
    void drawMeshWires(const Mesh& mesh, const Matrix& transform, Color color)
    {
        // Meshes freed from memory after upload have nothing left to read
        if (mesh.vertices == nullptr)
        {
            return;
        }
        const int cornerCount = mesh.indices != nullptr ? mesh.triangleCount * 3 : mesh.vertexCount;
        auto corner = [&](int i)
        {
            const int vertex = mesh.indices != nullptr ? mesh.indices[i] : i;
            const Vector3 local = { mesh.vertices[vertex * 3], mesh.vertices[vertex * 3 + 1], mesh.vertices[vertex * 3 + 2] };
            return Vector3Transform(local, transform);
        };
        for (int i = 0; i + 2 < cornerCount; i += 3)
        {
            const Vector3 a = corner(i);
            const Vector3 b = corner(i + 1);
            const Vector3 c = corner(i + 2);
            DrawLine3D(a, b, color);
            DrawLine3D(b, c, color);
            DrawLine3D(c, a, color);
        }
    }

    void drawModelWiresEx(const Model& model, Vector3 position, Vector3 rotationAxis, float rotationAngle, Vector3 scale, Color color)
    {
        // Placed the same way raylib's DrawModelEx places a model
        const Matrix scaling = MatrixScale(scale.x, scale.y, scale.z);
        const Matrix rotation = MatrixRotate(rotationAxis, rotationAngle * DEG2RAD);
        const Matrix translation = MatrixTranslate(position.x, position.y, position.z);
        const Matrix placement = MatrixMultiply(MatrixMultiply(scaling, rotation), translation);
        const Matrix transform = MatrixMultiply(model.transform, placement);
        for (int i = 0; i < model.meshCount; i++)
        {
            drawMeshWires(model.meshes[i], transform, color);
        }
    }

    void drawModelWires(const Model& model, Vector3 position, float scale, Color color)
    {
        drawModelWiresEx(model, position, { 0.0f, 1.0f, 0.0f }, 0.0f, { scale, scale, scale }, color);
    }
}
