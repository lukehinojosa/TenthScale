#include "flight/Vehicle.h"

#include <cmath>

namespace tenth
{
    Inertia& Inertia::operator+=(const Inertia& other)
    {
        xx += other.xx;
        yy += other.yy;
        zz += other.zz;
        xy += other.xy;
        xz += other.xz;
        yz += other.yz;
        return *this;
    }

    Inertia partInertia(const PlacedPart& placed)
    {
        const PartShape& shape = placed.shape;
        const double mass = placed.wetMass();
        const double h2 = shape.height * shape.height;
        Inertia inertia;
        if (shape.kind == ShapeKind::Round)
        {
            const double radius = (shape.topDiameter + shape.bottomDiameter) * 0.25;
            const double r2 = radius * radius;
            inertia.yy = mass * r2 * 0.5;
            inertia.xx = mass * (3.0 * r2 + h2) / 12.0;
            inertia.zz = inertia.xx;
            return inertia;
        }

        // A box is thin along its local x and reaches out along its local z, turned to face away from its parent
        const double t2 = shape.thickness * shape.thickness;
        const double s2 = shape.span * shape.span;
        const double localX = mass * (h2 + s2) / 12.0;
        const double localZ = mass * (t2 + h2) / 12.0;
        const double c = std::cos(placed.angle);
        const double s = std::sin(placed.angle);
        inertia.xx = c * c * localX + s * s * localZ;
        inertia.zz = s * s * localX + c * c * localZ;
        inertia.xz = c * s * (localZ - localX);
        inertia.yy = mass * (t2 + s2) / 12.0;
        return inertia;
    }

    Vehicle makeVehicle(const RocketDesign& design)
    {
        Vehicle vehicle;
        vehicle.design = design;
        if (design.empty())
        {
            return vehicle;
        }
        vehicle.mass = design.totalMass();
        vehicle.centerOfMass = design.centerOfMass();
        vehicle.groundClearance = vehicle.centerOfMass.y - design.lowestPoint();

        // Each part's own inertia, moved out to the center of mass by how far its centre sits from it
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            const PlacedPart& placed = design.part(i);
            const double mass = placed.wetMass();
            const Vec3d d = placed.position - vehicle.centerOfMass;
            Inertia shifted = partInertia(placed);
            shifted.xx += mass * (d.y * d.y + d.z * d.z);
            shifted.yy += mass * (d.x * d.x + d.z * d.z);
            shifted.zz += mass * (d.x * d.x + d.y * d.y);
            shifted.xy -= mass * d.x * d.y;
            shifted.xz -= mass * d.x * d.z;
            shifted.yz -= mass * d.y * d.z;
            vehicle.inertia += shifted;
        }
        return vehicle;
    }
}
