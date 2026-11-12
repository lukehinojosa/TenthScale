#pragma once
#include <vector>
#include "core.h"

namespace cyclone
{
    // One node of a Verlet rope. Verlet keeps the previous position instead of a velocity; the velocity is
    // implied by how far it moved.
    struct VerletPoint
    {
        Vector3 position = Vector3(0, 0, 0);
        Vector3 previousPosition = Vector3(0, 0, 0);

        // A pinned point is not moved by integration; it acts as an endpoint
        // driver/anchor whose position (and thus velocity) is set externally.
        bool pinned = false;
    };

    // A rope or elastic band as a chain of particles joined by springs. Pin the two ends and move them, and the
    // rest follows; the spring settings decide how stretchy it is.
    class VerletRope
    {
    public:
        // Builds a rope of evenly spaced particles between start and end, at least one segment, with both ends
        // pinned. Each spring's rest length is the starting spacing.
        VerletRope(Vector3 start, Vector3 end, int segments, real stiffness = 100.0f, real springDamping = 0.5f);

        // Advances the simulation by one Verlet step driven by the spring forces.
        void integrate(real dt);

        // Pins/unpins a particle. Pinned particles are driven, not simulated.
        void setPinned(int index, bool pinned);

        // Moves a particle to a new position. The move implies a velocity, so dragging an end pulls the rope along.
        // Call before integrate().
        void setPointPosition(int index, Vector3 position);

        // Number of particles in the rope.
        int getPointCount() const { return (int)points.size(); }

        // Position of a particle, for the visualizer to draw.
        Vector3 getPointPosition(int index) const { return points[index].position; }

        // Read-only access to all particles.
        const std::vector<VerletPoint>& getPoints() const { return points; }

        // Natural spacing between neighbouring particles (each spring's rest length).
        real getRestLength() const { return restLength; }
        void setRestLength(real value) { restLength = value; }

        void setStiffness(real value) { stiffness = value; }
        real getStiffness() const { return stiffness; }

        void setSpringDamping(real value) { springDamping = value; }
        real getSpringDamping() const { return springDamping; }

        // Constant acceleration applied to every free particle (gravity by default).
        void setAcceleration(Vector3 a) { acceleration = a; }
        Vector3 getAcceleration() const { return acceleration; }

        // Global velocity damping per step, in [0,1]. 1 = no damping. Bleeds off
        // energy for stability, on top of the per-link spring damping.
        void setDamping(real value) { damping = value; }
        real getDamping() const { return damping; }

        // Mass of each particle. 0 is treated as immovable (infinite mass).
        void setMass(real mass);

        // Constraint passes after the springs: more makes a tauter rope, fewer a stretchier band, and none leaves
        // pure springs that sag.
        void setConstraintIterations(int value) { constraintIterations = value; }
        int getConstraintIterations() const { return constraintIterations; }

        // Optional buoyancy applied to every rope particle (e.g. a slack cord
        // resting on water). nodeVolume is the submerged volume of one particle.
        void setBuoyancy(bool enabled, real fluidHeight, real liquidDensity, real nodeVolume, real objectHeight);

        // Optional drag applied to every rope particle (e.g. water resistance
        // on a slack cord). Mirrors ForceGenerator::AddDragForce.
        void setDrag(bool enabled, real linearCoefficient, real exponentialCoefficient, bool useMaxHeight, real maxHeight);

    private:
        // Verlet update using an acceleration accumulated for each particle.
        void integratePoints(real dt, const std::vector<Vector3>& forces);

        // Pulls linked particles back toward the rest length so the rope stays
        // taut instead of sagging under the soft spring forces alone.
        void solveConstraints();

        std::vector<VerletPoint> points;

        // Rest length of each spring (the initial particle spacing).
        real restLength = 0;

        real stiffness;
        real springDamping;

        Vector3 acceleration = Vector3(0.0f, -9.81f, 0.0f);
        real damping = 0.99f;

        // Inverse mass of each particle (1/mass); 0 means infinite mass.
        real inverseMass = 1.0f;

        int constraintIterations = 15;

        // Optional per-particle environment forces (buoyancy / drag).
        bool buoyancyEnabled = false;
        real buoyancyFluidHeight = 0;
        real buoyancyLiquidDensity = 0;
        real buoyancyNodeVolume = 0;
        real buoyancyObjectHeight = 0;

        bool dragEnabled = false;
        real dragLinearCoefficient = 0;
        real dragExponentialCoefficient = 0;
        bool dragUseMaxHeight = false;
        real dragMaxHeight = 0;
    };
}
