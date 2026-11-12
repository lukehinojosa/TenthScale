#pragma once
#include <vector>
#include "core.h"

namespace cyclone
{
    // One rope particle. Velocity is implied by (position - prevPosition), so none is stored.
    // Damping lives on the rope (ConstraintRope::setDamping) and is shared by every particle.
    struct VerletParticle
    {
        Vector3 position = Vector3(0, 0, 0);
        Vector3 prevPosition = Vector3(0, 0, 0);

        // The inverse of the mass. Zero makes the particle immovable, which is how a grabbed particle is held.
        real inverseMass = 1.0f;
    };

    // A static sphere the rope collides with.
    struct RopeSphereCollider
    {
        Vector3 center = Vector3(0, 0, 0);
        real radius = 1.0f;
        // 0 = slides freely, 1 = no sliding.
        real friction = 0.5f;
    };

    // A static, optionally rotated box the rope collides with.
    struct RopeBoxCollider
    {
        Vector3 center = Vector3(0, 0, 0);
        Vector3 halfExtents = Vector3(1, 1, 1);
        Quaternion orientation;
        // 0 = slides freely, 1 = no sliding.
        real friction = 0.5f;

        // World-space direction of the box's local x (0), y (1) or z (2) axis.
        Vector3 getAxis(int index) const;
    };

    // Verlet particles joined by distance constraints, solved by iterative relaxation.
    // Each step: integrate, solve constraints (distance + collision), then apply friction.
    class ConstraintRope
    {
    public:
        // A straight rope from start along direction, with at least one segment.
        ConstraintRope(Vector3 start, Vector3 direction, real length, int segments, real radius);

        // Advances the rope by one fixed step.
        void step(real dt);

        int getParticleCount() const { return (int)particles.size(); }
        const VerletParticle& getParticle(int index) const { return particles[index]; }
        Vector3 getParticlePosition(int index) const { return particles[index].position; }

        // Teleports a particle: sets position and prevPosition, so it carries no velocity.
        void setParticlePosition(int index, Vector3 position);

        void setInverseMass(int index, real inverseMass) { particles[index].inverseMass = inverseMass; }
        real getInverseMass(int index) const { return particles[index].inverseMass; }

        // Returns position pushed out of the ground and every sphere, as a point of the rope's radius.
        Vector3 pushOutOfColliders(Vector3 position) const;

        // Moves a point from `from` toward `to`, sliding along any sphere it hits instead of passing through.
        // Used to place a dragged particle so a fast mouse can't pull it through the sphere.
        Vector3 sweepPointToward(Vector3 from, Vector3 to) const;

        // Point `distance` along a ray. If that lands inside a sphere or box, it moves back to where the ray enters it,
        // so a dragged particle stays under the cursor on the surface facing the camera.
        Vector3 constrainDragTarget(Vector3 origin, Vector3 direction, real distance) const;

        // Pulls a particle toward `target` with a force instead of placing it there. It stays a normal particle,
        // so collisions still stop it and it can't be dragged through anything. Call every frame while dragging.
        void setDragTarget(int index, Vector3 target);
        void releaseDrag() { dragIndex = -1; }
        int getDragIndex() const { return dragIndex; }
        Vector3 getDragTarget() const { return dragTarget; }

        // How hard the dragged particle is pulled toward the cursor: this strength at epsilon away, weaker closer in,
        // and growing quickly further out.
        void setDragStrength(real value) { dragStrength = value < 0 ? 0 : value; }
        real getDragStrength() const { return dragStrength; }
        void setDragEpsilon(real value) { dragEpsilon = value < (real)0.001 ? (real)0.001 : value; }
        real getDragEpsilon() const { return dragEpsilon; }

        real getLength() const { return segmentLength * (real)(particles.size() - 1); }
        real getSegmentLength() const { return segmentLength; }
        real getRadius() const { return radius; }

        void setGravity(Vector3 value) { gravity = value; }
        Vector3 getGravity() const { return gravity; }

        // Relaxation passes per step, at least one. More passes mean less stretch.
        void setConstraintIterations(int value) { constraintIterations = value < 1 ? 1 : value; }
        int getConstraintIterations() const { return constraintIterations; }

        // The fraction of velocity kept each second, from 0 to 1; 1 means no damping.
        void setDamping(real value) { damping = value; }
        real getDamping() const { return damping; }

        // Turns friction on or off. While a particle is held, sticky contacts still get dragged along without stretching.
        void setFrictionEnabled(bool enabled) { frictionEnabled = enabled; }
        bool getFrictionEnabled() const { return frictionEnabled; }

        // Max bend between neighbouring segments, in degrees: 0 = stays straight, 180 = unconstrained (default).
        void setMaxBendAngle(real degrees) { maxBendAngle = degrees < 0 ? 0 : (degrees > 180 ? 180 : degrees); }
        real getMaxBendAngle() const { return maxBendAngle; }

        void setSelfCollision(bool enabled) { selfCollision = enabled; }
        bool getSelfCollision() const { return selfCollision; }

        // Infinite flat ground at the given height.
        void setGround(real height, real friction)
        {
            groundHeight = height;
            groundFriction = friction;
        }
        void setGroundFriction(real friction) { groundFriction = friction; }
        real getGroundHeight() const { return groundHeight; }
        real getGroundFriction() const { return groundFriction; }

        void addSphere(const RopeSphereCollider& sphere);
        int getSphereCount() const { return (int)spheres.size(); }
        RopeSphereCollider& getSphere(int index) { return spheres[index]; }
        const RopeSphereCollider& getSphere(int index) const { return spheres[index]; }

        void addBox(const RopeBoxCollider& box);
        int getBoxCount() const { return (int)boxes.size(); }
        RopeBoxCollider& getBox(int index) { return boxes[index]; }
        const RopeBoxCollider& getBox(int index) const { return boxes[index]; }

    private:
        // One full step: integrate, constraints, friction, collision cleanup. step() may run several per call.
        void stepOnce(real dt);

        // Verlet step under gravity with damping. Free particles only.
        void integrate(real dt);

        // This step's move for the dragged particle: its velocity plus gravity and the pull toward the drag target.
        Vector3 dragMove(const VerletParticle& p, Vector3 velocity, real dt) const;

        // Held (inverse mass 0) or dragged: the rope is solved outward from these.
        bool isAnchor(size_t i) const;

        // Inverse mass used by the link and bend constraints (the dragged particle counts as heavier).
        real solveWeight(size_t i) const;

        // Relaxation loop over distance and collision constraints, ending on collisions.
        void solveConstraints();

        // Solves every link once, outward from a held particle if there is one.
        void solveDistanceConstraints(int iteration);
        // Mass-weighted distance constraint between particle i and i + 1.
        void solveDistanceConstraint(int i);

        // Angular constraint from the triangle at each joint: a min distance between particles i and i + 2
        // (law of cosines), and a max distance from i + 1 to their midpoint so the middle particle moves too.
        void solveBendConstraints();

        // Outward from a held particle, places each particle exactly one segment from the one before.
        // With a maxStretch of zero or more it only shortens links stretched past that fraction, back to that length.
        void solveFollowTheLeader(real maxStretch = -1);

        // Self, then ground, then spheres, so the environment gets the last word.
        void solveCollisionConstraints();
        void solveSelfCollisions();
        void solveGroundCollisions();

        // Spheres, boxes, then the ground fix-up, then records safe positions.
        void solveColliderCollisions();
        void solveSphereCollisions();
        void solveBoxCollisions();

        // Lifts particles pushed below ground beside a collider and frees any stuck in a sphere's crevice.
        void solveGroundCorners();

        // Deepest the cord (particles or links) overlaps any sphere or box.
        real worstColliderOverlap() const;

        // Stores positions that are outside every sphere and box; hits are swept from here.
        void updateSafePositions();

        // Kinetic friction for every particle pushed into the ground, a sphere or a box this step.
        void applyFriction();

        // Sliding friction against one surface; true if it stopped the particle. With keepPosition only the velocity
        // changes, otherwise this step's slide is undone too.
        bool applyContactFriction(VerletParticle& particle, Vector3 normal, real penetration, real friction, bool keepPosition);

        // Pins the particles friction stopped and re-solves the rest, so friction can't stretch the rope.
        void holdFrictionContacts(const std::vector<size_t>& held);

        // Moves the point at t on segment (a, b) by delta, split by inverse mass. Optionally returns each end's move.
        void moveSegmentPoint(int a, int b, real t, Vector3 delta, real* movedA = nullptr, real* movedB = nullptr);

        // Index into sphereContactDepth for particle i and sphere s.
        size_t sphereDepthIndex(size_t i, size_t s) const { return i * spheres.size() + s; }
        // Index into boxContactDepth for particle i and box b.
        size_t boxDepthIndex(size_t i, size_t b) const { return i * boxes.size() + b; }

        std::vector<VerletParticle> particles;
        real segmentLength = 0;
        real radius = 0.05f;

        Vector3 gravity = Vector3(0.0f, -9.81f, 0.0f);
        int constraintIterations = 20;
        real damping = 0.99f;
        real maxBendAngle = 180.0f;

        int dragIndex = -1;
        Vector3 dragTarget = Vector3(0, 0, 0);
        Vector3 lastDragTarget = Vector3(0, 0, 0);
        real dragStrength = 200.0f;
        real dragEpsilon = 0.25f;
        bool selfCollision = true;
        bool frictionEnabled = true;

        real groundHeight = 0;
        real groundFriction = 0.5f;
        std::vector<RopeSphereCollider> spheres;
        std::vector<RopeBoxCollider> boxes;

        // How far collisions pushed each particle out of the ground / each sphere / each box this step.
        std::vector<real> groundContactDepth;
        std::vector<real> sphereContactDepth;
        std::vector<real> boxContactDepth;

        // Last position of each particle known to be outside every sphere and box.
        // Hits are resolved along the path from here, so a particle can't tunnel out the far side.
        std::vector<Vector3> safePositions;

        // Every particle's position at the end of the last step, to tell when a held particle is being dragged.
        std::vector<Vector3> lastPositions;
    };
}
