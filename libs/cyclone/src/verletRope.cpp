#include "verletRope.h"
#include "forceGenerator.h"

using namespace cyclone;

VerletRope::VerletRope(Vector3 start, Vector3 end, int segments, real stiffness, real springDamping)
    : stiffness(stiffness), springDamping(springDamping)
{
    // Need at least one link.
    if (segments < 1)
    {
        segments = 1;
    }

    const int pointCount = segments + 1;
    Vector3 step = (end - start) * (((real)1.0) / segments);

    // The rest length of each spring is the initial even spacing.
    restLength = step.magnitude();

    points.reserve(pointCount);
    for (int i = 0; i < pointCount; i++)
    {
        VerletPoint point;
        point.position = start + step * (real)i;
        // Previous position equal to current means the rope starts at rest.
        point.previousPosition = point.position;
        points.push_back(point);
    }

    // The two endpoints drive the rope by default.
    if (!points.empty())
    {
        points.front().pinned = true;
        points.back().pinned = true;
    }
}

void VerletRope::integrate(real dt)
{
    if (dt <= 0)
    {
        return;
    }

    const size_t n = points.size();
    const real invDt = ((real)1.0) / dt;

    // Implied velocity of each particle from its last movement.
    std::vector<Vector3> velocities(n);
    for (size_t i = 0; i < n; i++)
    {
        velocities[i] = (points[i].position - points[i].previousPosition) * invDt;
    }

    // Spring force on each particle from its neighbours. Each link is worked out once and pushes both ends
    // equally in opposite directions.
    std::vector<Vector3> forces(n, Vector3(0, 0, 0));
    for (size_t i = 0; i + 1 < n; i++)
    {
        Vector3 force = ForceGenerator::AddSpringForce(points[i].position, points[i + 1].position, restLength, stiffness, springDamping,
                                                       velocities[i], velocities[i + 1]);
        forces[i] += force;
        forces[i + 1] -= force;
    }

    // Optional environment forces on each particle (e.g. a slack cord in water).
    if (buoyancyEnabled || dragEnabled)
    {
        for (size_t i = 0; i < n; i++)
        {
            if (buoyancyEnabled)
            {
                forces[i] += ForceGenerator::AddBuoyancyForce(points[i].position, buoyancyFluidHeight, buoyancyLiquidDensity,
                                                              buoyancyNodeVolume, buoyancyObjectHeight);
            }
            if (dragEnabled)
            {
                Vector3 drag = ForceGenerator::AddDragForce(points[i].position, velocities[i], dragLinearCoefficient,
                                                            dragExponentialCoefficient, dragUseMaxHeight, dragMaxHeight);

                // Limit the drag so it can only slow the motion, never flip it.
                real mass = (inverseMass <= 0) ? REAL_MAX : ((real)1.0) / inverseMass;
                real maxMagnitude = mass * velocities[i].magnitude() / dt;
                real magnitude = drag.magnitude();
                if (magnitude > maxMagnitude)
                {
                    drag = drag * (maxMagnitude / magnitude);
                }

                forces[i] += drag;
            }
        }
    }

    integratePoints(dt, forces);

    // Keep the rope taut by satisfying the rest-length distance constraints.
    solveConstraints();
}

void VerletRope::integratePoints(real dt, const std::vector<Vector3>& forces)
{
    for (size_t i = 0; i < points.size(); i++)
    {
        VerletPoint& point = points[i];

        // Pinned particles are moved from outside. Resetting the previous position stops a still anchor from
        // seeming to move next frame.
        if (point.pinned)
        {
            point.previousPosition = point.position;
            continue;
        }

        // Constant acceleration (gravity) plus acceleration from spring forces.
        Vector3 accel = acceleration + forces[i] * inverseMass;

        // Implied velocity from the last step, bled off by the global damping.
        Vector3 velocity = (point.position - point.previousPosition) * damping;

        Vector3 current = point.position;

        // Verlet update: carry on with the implied velocity, plus this step's acceleration
        point.position = point.position + velocity + accel * (dt * dt);

        point.previousPosition = current;
    }
}

void VerletRope::solveConstraints()
{
    // Iteratively relax each link back toward its rest length. Running several
    // passes lets the correction propagate along the rope for a stiffer result.
    for (int iteration = 0; iteration < constraintIterations; iteration++)
    {
        for (size_t i = 0; i + 1 < points.size(); i++)
        {
            VerletPoint& a = points[i];
            VerletPoint& b = points[i + 1];

            Vector3 delta = b.position - a.position;
            real distance = delta.magnitude();

            // Coincident points give no direction to correct along.
            if (distance <= 0)
            {
                continue;
            }

            // Pinned points don't move, so the free end takes the full share.
            real weightA = a.pinned ? (real)0.0 : (real)1.0;
            real weightB = b.pinned ? (real)0.0 : (real)1.0;
            real weightSum = weightA + weightB;
            if (weightSum <= 0)
            {
                continue;
            }

            real difference = (distance - restLength) / distance;
            Vector3 correction = delta * difference;

            a.position += correction * (weightA / weightSum);
            b.position -= correction * (weightB / weightSum);
        }
    }
}

void VerletRope::setPinned(int index, bool pinned)
{
    points[index].pinned = pinned;
}

void VerletRope::setPointPosition(int index, Vector3 position)
{
    // Leave previousPosition at the current position so the move shows up as a
    // velocity ((new - current) / dt) that the spring damping can transmit.
    points[index].previousPosition = points[index].position;
    points[index].position = position;
}

void VerletRope::setBuoyancy(bool enabled, real fluidHeight, real liquidDensity, real nodeVolume, real objectHeight)
{
    buoyancyEnabled = enabled;
    buoyancyFluidHeight = fluidHeight;
    buoyancyLiquidDensity = liquidDensity;
    buoyancyNodeVolume = nodeVolume;
    buoyancyObjectHeight = objectHeight;
}

void VerletRope::setDrag(bool enabled, real linearCoefficient, real exponentialCoefficient, bool useMaxHeight, real maxHeight)
{
    dragEnabled = enabled;
    dragLinearCoefficient = linearCoefficient;
    dragExponentialCoefficient = exponentialCoefficient;
    dragUseMaxHeight = useMaxHeight;
    dragMaxHeight = maxHeight;
}

void VerletRope::setMass(real mass)
{
    if (mass <= 0)
    {
        inverseMass = 0;
    }
    else
    {
        inverseMass = ((real)1.0) / mass;
    }
}
