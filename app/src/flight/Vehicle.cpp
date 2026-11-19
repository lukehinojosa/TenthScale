#include "flight/Vehicle.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "flight/DesignStats.h"

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

    Inertia partInertia(const PlacedPart& placed, double mass)
    {
        const PartShape& shape = placed.shape;
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

    std::vector<Vec3d> partOutline(const RocketDesign& design, PartIndex index)
    {
        constexpr int RIM_POINTS = 8;
        const PlacedPart& placed = design.part(index);
        const PartShape& shape = placed.shape;
        const double halfHeight = shape.height * 0.5;
        std::vector<Vec3d> points;
        if (shape.kind == ShapeKind::Round)
        {
            // A shrouded engine is as wide at the bottom as at the top
            const double bottomRadius = (design.isShrouded(index) ? shape.topDiameter : shape.bottomDiameter) * 0.5;
            const double topRadius = shape.topDiameter * 0.5;
            for (int i = 0; i < RIM_POINTS; i++)
            {
                const double angle = 2.0 * std::numbers::pi * i / RIM_POINTS;
                const double c = std::cos(angle);
                const double s = std::sin(angle);
                points.push_back(placed.position + Vec3d{ bottomRadius * c, -halfHeight, bottomRadius * s });
                points.push_back(placed.position + Vec3d{ topRadius * c, halfHeight, topRadius * s });
            }
            return points;
        }

        // A box's corners, turned the same way it's drawn
        const double c = std::cos(placed.angle);
        const double s = std::sin(placed.angle);
        for (int corner = 0; corner < 8; corner++)
        {
            const double x = (corner & 1 ? 0.5 : -0.5) * shape.thickness;
            const double y = (corner & 2 ? 1.0 : -1.0) * halfHeight;
            const double z = (corner & 4 ? 0.5 : -0.5) * shape.span;
            points.push_back(placed.position + Vec3d{ x * c + z * s, y, z * c - x * s });
        }
        return points;
    }

    Vehicle makeVehicle(const RocketDesign& design)
    {
        Vehicle vehicle;
        vehicle.design = design;
        vehicle.sections = sectionStarts(design);
        vehicle.lit.assign(design.partCount(), false);
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            vehicle.fuel.push_back(design.part(i).pDefinition->fuelMass);
            const std::vector<Vec3d> points = partOutline(design, i);
            vehicle.outline.insert(vehicle.outline.end(), points.begin(), points.end());
        }
        updateMass(vehicle);
        if (!design.empty())
        {
            vehicle.groundClearance = vehicle.centerOfMass.y - design.lowestPoint();
        }
        return vehicle;
    }

    void updateMass(Vehicle& vehicle)
    {
        const RocketDesign& design = vehicle.design;
        vehicle.mass = 0.0;
        vehicle.inertia = {};
        Vec3d weighted;
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            const double mass = design.part(i).dryMass() + vehicle.fuel[i];
            vehicle.mass += mass;
            weighted += design.part(i).position * mass;
        }
        vehicle.centerOfMass = vehicle.mass > 0.0 ? weighted / vehicle.mass : Vec3d{};

        // Each part's own inertia, moved out to the center of mass by how far its centre sits from it
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            const PlacedPart& placed = design.part(i);
            const double mass = placed.dryMass() + vehicle.fuel[i];
            const Vec3d d = placed.position - vehicle.centerOfMass;
            Inertia shifted = partInertia(placed, mass);
            shifted.xx += mass * (d.y * d.y + d.z * d.z);
            shifted.yy += mass * (d.x * d.x + d.z * d.z);
            shifted.zz += mass * (d.x * d.x + d.y * d.y);
            shifted.xy -= mass * d.x * d.y;
            shifted.xz -= mass * d.x * d.z;
            shifted.yz -= mass * d.y * d.z;
            vehicle.inertia += shifted;
        }
    }

    std::vector<PartIndex> fireNextStage(Vehicle& vehicle)
    {
        if (vehicle.nextStage >= vehicle.design.stageCount())
        {
            return {};
        }
        const std::vector<PartIndex> fired = vehicle.design.partsInStage(vehicle.nextStage);
        for (PartIndex i : fired)
        {
            if (vehicle.design.part(i).pDefinition->isEngine())
            {
                vehicle.lit[i] = true;
            }
        }
        vehicle.nextStage++;
        return fired;
    }

    double sectionFuel(const Vehicle& vehicle, PartIndex section)
    {
        double total = 0.0;
        for (PartIndex i = 0; i < vehicle.design.partCount(); i++)
        {
            if (vehicle.sections[i] == section)
            {
                total += vehicle.fuel[i];
            }
        }
        return total;
    }

    double sectionCapacity(const Vehicle& vehicle, PartIndex section)
    {
        double total = 0.0;
        for (PartIndex i = 0; i < vehicle.design.partCount(); i++)
        {
            if (vehicle.sections[i] == section)
            {
                total += vehicle.design.part(i).pDefinition->fuelMass;
            }
        }
        return total;
    }

    bool isBurning(const Vehicle& vehicle, PartIndex engine)
    {
        return vehicle.lit[engine] && sectionFuel(vehicle, vehicle.sections[engine]) > 0.0;
    }

    EngineOutput engineOutput(const PartDefinition& engine, double pressure)
    {
        EngineOutput output;
        if (!engine.isEngine())
        {
            return output;
        }
        const double share = std::clamp(pressure, 0.0, 1.0);
        output.isp = engine.ispVacuum + (engine.ispSeaLevel - engine.ispVacuum) * share;
        output.thrust = engine.thrustVacuum * output.isp / engine.ispVacuum;
        return output;
    }

    void burnFuel(Vehicle& vehicle, double pressure, double dt)
    {
        const RocketDesign& design = vehicle.design;
        for (PartIndex engine = 0; engine < design.partCount(); engine++)
        {
            if (!isBurning(vehicle, engine))
            {
                continue;
            }
            const EngineOutput output = engineOutput(*design.part(engine).pDefinition, pressure);
            const PartIndex section = vehicle.sections[engine];
            const double available = sectionFuel(vehicle, section);
            const double burned = std::min(available, output.thrust * vehicle.throttle / (output.isp * STANDARD_GRAVITY) * dt);

            // Taking the same share from each tank empties them all together
            const double share = burned / available;
            for (PartIndex i = 0; i < design.partCount(); i++)
            {
                if (vehicle.sections[i] == section)
                {
                    vehicle.fuel[i] = std::max(0.0, vehicle.fuel[i] * (1.0 - share));
                }
            }
        }
    }
}
