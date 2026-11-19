#include "flight/Aerodynamics.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace tenth
{
    namespace
    {
        constexpr double FLAT_FACE_DRAG = 0.9; // a blunt end meeting the flow head on
        constexpr double SLOPED_FACE_DRAG = 0.3; // a tapered side, like a nose cone's, parting the flow
        constexpr double SIDE_DRAG = 1.0; // a cylinder crossways to the flow
        constexpr double PLATE_DRAG = 1.2; // a fin face square to the flow
        constexpr double PLATE_EDGE_DRAG = 0.05; // a fin edge-on, slicing through
        constexpr double PARACHUTE_DRAG = 1.5;

        double circle(double radius)
        {
            return std::numbers::pi * radius * radius;
        }

        // Drag from flow along one direction: half the density times speed squared, times area and coefficient
        Vec3d push(const Vec3d& velocity, double density, double dragArea)
        {
            return velocity * (-0.5 * density * velocity.length() * dragArea);
        }
    }

    Vec3d partDrag(const RocketDesign& design, PartIndex index, const Vec3d& velocity, double density)
    {
        if (density <= 0.0 || velocity.lengthSquared() == 0.0)
        {
            return {};
        }
        const PlacedPart& placed = design.part(index);
        const PartShape& shape = placed.shape;

        if (shape.kind == ShapeKind::Round)
        {
            const double topRadius = shape.topDiameter * 0.5;
            const double bottomRadius = (design.isShrouded(index) ? shape.topDiameter : shape.bottomDiameter) * 0.5;

            // The end the part moves toward meets the flow: its face if nothing covers it, plus any taper narrowing
            // that way. The trailing end is left out.
            double axialArea = 0.0;
            if (velocity.y > 0.0)
            {
                axialArea += design.isNodeFree(index, Node::Top) ? circle(topRadius) * FLAT_FACE_DRAG : 0.0;
                axialArea += std::max(0.0, circle(bottomRadius) - circle(topRadius)) * SLOPED_FACE_DRAG;
            }
            else
            {
                axialArea += design.isNodeFree(index, Node::Bottom) ? circle(bottomRadius) * FLAT_FACE_DRAG : 0.0;
                axialArea += std::max(0.0, circle(topRadius) - circle(bottomRadius)) * SLOPED_FACE_DRAG;
            }
            const double sideArea = shape.height * (topRadius + bottomRadius) * SIDE_DRAG;
            const Vec3d axial = { 0.0, velocity.y, 0.0 };
            const Vec3d across = { velocity.x, 0.0, velocity.z };
            return push(axial, density, axialArea) + push(across, density, sideArea);
        }

        // A plate's broad faces look along its thin axis, turned with the plate. Flow along the face barely pushes.
        const Vec3d normal = { std::cos(placed.angle), 0.0, -std::sin(placed.angle) };
        const double faceArea = shape.height * shape.span;
        const Vec3d square = normal * velocity.dot(normal);
        const Vec3d along = velocity - square;
        return push(square, density, faceArea * PLATE_DRAG) + push(along, density, faceArea * PLATE_EDGE_DRAG);
    }

    Vec3d parachuteDrag(double opening, const Vec3d& velocity, double density)
    {
        if (opening <= 0.0 || density <= 0.0)
        {
            return {};
        }
        return push(velocity, density, circle(PARACHUTE_DIAMETER * 0.5) * std::min(opening, 1.0) * PARACHUTE_DRAG);
    }
}
