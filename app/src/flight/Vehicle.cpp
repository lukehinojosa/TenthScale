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

    bool insidePart(const RocketDesign& design, PartIndex index, const Vec3d& point, double& depth, Vec3d& outward)
    {
        const PlacedPart& placed = design.part(index);
        const PartShape& shape = placed.shape;
        const Vec3d local = point - placed.position;
        const double halfHeight = shape.height * 0.5;
        if (std::abs(local.y) >= halfHeight)
        {
            return false;
        }

        // The nearest of the top, the bottom, and the sides is the way out
        depth = halfHeight - local.y;
        outward = { 0.0, 1.0, 0.0 };
        if (halfHeight + local.y < depth)
        {
            depth = halfHeight + local.y;
            outward = { 0.0, -1.0, 0.0 };
        }
        if (shape.kind == ShapeKind::Round)
        {
            const double bottomRadius = (design.isShrouded(index) ? shape.topDiameter : shape.bottomDiameter) * 0.5;
            const double topRadius = shape.topDiameter * 0.5;
            const double radius = bottomRadius + (topRadius - bottomRadius) * (local.y + halfHeight) / shape.height;
            const double across = std::hypot(local.x, local.z);
            if (across >= radius)
            {
                return false;
            }
            if (radius - across < depth)
            {
                depth = radius - across;
                outward = across > 0.0 ? Vec3d{ local.x / across, 0.0, local.z / across } : Vec3d{ 1.0, 0.0, 0.0 };
            }
            return true;
        }

        // A box is turned to face away from its parent, so the point is turned back into the box's own axes
        const double c = std::cos(placed.angle);
        const double s = std::sin(placed.angle);
        const double x = local.x * c - local.z * s;
        const double z = local.x * s + local.z * c;
        const double halfThickness = shape.thickness * 0.5;
        const double halfSpan = shape.span * 0.5;
        if (std::abs(x) >= halfThickness || std::abs(z) >= halfSpan)
        {
            return false;
        }
        const double sideX = halfThickness - std::abs(x);
        const double sideZ = halfSpan - std::abs(z);
        if (sideX < depth && sideX <= sideZ)
        {
            depth = sideX;
            const double sign = x < 0.0 ? -1.0 : 1.0;
            outward = { sign * c, 0.0, -sign * s };
        }
        else if (sideZ < depth)
        {
            depth = sideZ;
            const double sign = z < 0.0 ? -1.0 : 1.0;
            outward = { sign * s, 0.0, sign * c };
        }
        return true;
    }

    namespace
    {
        // Sections and the outline depend on how the parts connect, so they're redone whenever the rocket splits
        void rebuild(Vehicle& vehicle)
        {
            const RocketDesign& design = vehicle.design;
            vehicle.sections = sectionStarts(design);
            vehicle.outline.clear();
            for (PartIndex i = 0; i < design.partCount(); i++)
            {
                const std::vector<Vec3d> points = partOutline(design, i);
                vehicle.outline.insert(vehicle.outline.end(), points.begin(), points.end());
            }
            updateMass(vehicle);
            if (!design.empty())
            {
                vehicle.groundClearance = vehicle.centerOfMass.y - design.lowestPoint();
            }
        }
    }

    Vehicle makeVehicle(const RocketDesign& design)
    {
        Vehicle vehicle;
        vehicle.design = design;
        vehicle.lit.assign(design.partCount(), false);
        vehicle.deployed.assign(design.partCount(), false);
        vehicle.canopy.assign(design.partCount(), 0.0);
        vehicle.gimbal.assign(design.partCount(), {});
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            vehicle.fuel.push_back(design.part(i).pDefinition->fuelMass);
        }
        rebuild(vehicle);
        return vehicle;
    }

    Vec3d gimbalPivot(const Vehicle& vehicle, PartIndex engine)
    {
        const PlacedPart& placed = vehicle.design.part(engine);
        return placed.position + Vec3d{ 0.0, placed.shape.height * 0.5, 0.0 };
    }

    Vec3d thrustDirection(const Vehicle& vehicle, PartIndex engine)
    {
        const Vec3d lean = vehicle.gimbal[engine];
        const double angle = lean.length();
        if (angle <= 0.0)
        {
            return { 0.0, 1.0, 0.0 };
        }
        const Vec3d toward = lean / angle;
        return { toward.x * std::sin(angle), std::cos(angle), toward.z * std::sin(angle) };
    }

    double wheelTorque(const Vehicle& vehicle)
    {
        double total = 0.0;
        for (PartIndex i = 0; i < vehicle.design.partCount(); i++)
        {
            total += vehicle.design.part(i).pDefinition->wheelTorque;
        }
        return total;
    }

    Vehicle detachBranch(Vehicle& vehicle, PartIndex index, Vec3d& rootOffset)
    {
        // Parents come first, so one pass in order finds the whole branch; both halves keep their parts in order
        const int count = vehicle.design.partCount();
        std::vector<bool> inBranch(count, false);
        for (PartIndex i = index; i < count; i++)
        {
            const PartIndex parent = vehicle.design.part(i).parent;
            inBranch[i] = i == index || (parent != NO_PART && inBranch[parent]);
        }

        Vehicle branch;
        branch.throttle = vehicle.throttle;
        Vehicle kept = vehicle;
        kept.fuel.clear();
        kept.lit.clear();
        kept.deployed.clear();
        kept.canopy.clear();
        kept.gimbal.clear();
        for (PartIndex i = 0; i < count; i++)
        {
            Vehicle& side = inBranch[i] ? branch : kept;
            side.fuel.push_back(vehicle.fuel[i]);
            side.lit.push_back(vehicle.lit[i]);
            side.deployed.push_back(vehicle.deployed[i]);
            side.canopy.push_back(vehicle.canopy[i]);
            side.gimbal.push_back(vehicle.gimbal[i]);
        }
        rootOffset = vehicle.design.part(index).position;
        branch.design = kept.design.extract(index);
        rebuild(branch);
        rebuild(kept);

        // The ground clearance is kept from rollout, as it's only for standing on the pad
        kept.groundClearance = vehicle.groundClearance;
        vehicle = std::move(kept);
        return branch;
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
            const PartDefinition& definition = *vehicle.design.part(i).pDefinition;
            if (definition.isEngine())
            {
                vehicle.lit[i] = true;
            }
            if (definition.hasParachute)
            {
                vehicle.deployed[i] = true;
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
