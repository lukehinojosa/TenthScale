#include "constraintRope.h"
#include <cmath>

using namespace cyclone;

namespace
{
    // Segment-sphere contacts this close to a segment end are left to the particle test.
    const real kSegmentEndEpsilon = (real)0.0001;

    // End-of-step sphere and box passes, repeated until the overlap is within tolerance.
    const int kMaxColliderCleanupPasses = 16;
    const real kColliderOverlapTolerance = (real)0.0005;

    // Follow-the-leader + collision passes run after the iterations.
    const int kFollowTheLeaderPasses = 3;

    // Fraction of each follow-the-leader correction removed from the leader's velocity (paper: 0.9 to 1).
    const real kFollowTheLeaderVelocityCorrection = (real)0.9;

    // Golden-section steps used to find a link's deepest point in a box, and how close to an end counts as the end.
    const int kBoxSearchSteps = 20;
    const real kBoxSegmentEndEpsilon = (real)0.001;

    // Distance + follow-the-leader + collision passes run after friction pins the particles it stopped.
    const int kFrictionHoldPasses = 3;

    // While dragging, link stretch (as a fraction of the segment length) left after friction before the rope is pulled taut.
    const real kDragStretchTolerance = (real)0.002;

    // Mouse drag force: velocity damping of the dragged particle (per second), and a cap on the pull's exponent
    // so a far-away cursor can't overflow it.
    const real kDragDamping = (real)20.0;

    // Furthest the dragged particle moves in one sub-step (m), and the most sub-steps a step is split into.
    const real kMaxDragSubstepMove = (real)0.15;
    const int kMaxDragSubsteps = 32;
    const real kMaxDragExponent = (real)30.0;

    // The dragged particle counts as 50 times heavier in the link and bend constraints, so the rest of the rope gives way.
    const real kDragInverseMassScale = (real)0.02;

    real Clamp01(real value)
    {
        if (value < 0)
        {
            return 0;
        }
        if (value > 1)
        {
            return 1;
        }
        return value;
    }

    // Contact between link (a, b) of thickness ropeRadius and a sphere: closest point at t, outward normal, depth.
    // Returns false if the closest point is an end particle or the link doesn't touch the sphere.
    bool FindSegmentSphereContact(Vector3 a, Vector3 b, const RopeSphereCollider& sphere, real ropeRadius, real& t, Vector3& normal,
                                  real& depth)
    {
        const Vector3 ab = b - a;
        const real lengthSq = ab.squareMagnitude();
        if (lengthSq <= 0)
        {
            return false;
        }

        t = ((sphere.center - a) * ab) / lengthSq;
        if (t <= kSegmentEndEpsilon || t >= (real)1.0 - kSegmentEndEpsilon)
        {
            return false; // closest point is an end particle
        }

        const Vector3 offset = (a + ab * t) - sphere.center;
        const real distance = offset.magnitude();
        const real r = sphere.radius + ropeRadius;
        if (distance >= r)
        {
            return false;
        }

        if (distance > 0)
        {
            normal = offset * ((real)1.0 / distance);
        }
        else
        {
            // Link passes through the centre: push out perpendicular to it.
            normal = ab % Vector3(0, 1, 0);
            if (normal.squareMagnitude() <= 0)
            {
                normal = ab % Vector3(1, 0, 0);
            }
            normal.normalise();
        }
        depth = r - distance;
        return true;
    }

    // Sweeps a point from `from` to `to` against a sphere of radius r: stops at the surface, then slides along it.
    // Can't tunnel. depth = motion removed along the normal. False if it starts inside or never enters.
    bool SweepSlideSphere(Vector3 from, Vector3 to, Vector3 center, real r, Vector3& out, real& depth)
    {
        const Vector3 move = to - from;
        const Vector3 fromCenter = from - center;

        const real c = fromCenter.squareMagnitude() - r * r;
        if (c < 0)
        {
            return false; // starts inside: no side to come in from
        }
        const real a = move.squareMagnitude();
        if (a <= 0)
        {
            return false;
        }
        const real b = (real)2.0 * (fromCenter * move);
        if (b >= 0)
        {
            return false; // moving away from the sphere
        }

        // First s in [0,1] where |from + s * move - center| = r.
        const real discriminant = b * b - (real)4.0 * a * c;
        if (discriminant < 0)
        {
            return false; // path misses the sphere
        }
        const real s = (-b - real_sqrt(discriminant)) / ((real)2.0 * a);
        if (s < 0 || s > 1)
        {
            return false;
        }

        const Vector3 hit = from + move * s;
        Vector3 normal = hit - center;
        normal.normalise();

        // Drop the motion into the sphere, keep the tangential part.
        Vector3 remaining = to - hit;
        const real into = remaining * normal;
        depth = 0;
        if (into < 0)
        {
            remaining -= normal * into;
            depth = -into;
        }

        // Project the slide back onto the sphere, keeping it on the side it entered from.
        Vector3 slid = (hit + remaining) - center;
        const real slidLength = slid.magnitude();
        out = (slidLength > 0) ? center + slid * (r / slidLength) : hit;
        return true;
    }

    // Moves a point stuck in the crevice between sphere and ground (at y = minY) horizontally onto
    // the ring where the sphere meets that height. Returns how far it moved.
    real PushOutOfGroundCorner(Vector3& position, real minY, Vector3 center, real r)
    {
        if (position.y > minY + (real)0.0001)
        {
            return 0; // not at ground level: the sphere projection handles it
        }
        const Vector3 offset = position - center;
        if (offset.squareMagnitude() >= r * r)
        {
            return 0;
        }

        const real height = minY - center.y;
        const real ringRadiusSq = r * r - height * height;
        if (ringRadiusSq <= 0)
        {
            return 0; // the sphere doesn't reach ground level
        }
        const real ringRadius = real_sqrt(ringRadiusSq);

        Vector3 horizontal(offset.x, 0, offset.z);
        real horizontalLength = horizontal.magnitude();
        if (horizontalLength <= 0)
        {
            horizontal = Vector3(1, 0, 0);
            horizontalLength = 1;
        }

        const Vector3 before = position;
        position = Vector3(center.x, minY, center.z) + horizontal * (ringRadius / horizontalLength);
        return (position - before).magnitude();
    }

    // Horizontal move that takes `point` out of a sphere of radius r at its current height, away from the sphere's
    // vertical axis on the side it's already on. `along` is the link direction, used when it's right on the axis.
    Vector3 SidewaysOutOfSphere(Vector3 point, Vector3 along, Vector3 center, real r)
    {
        const real height = point.y - center.y;
        const real ringRadiusSq = r * r - height * height;
        if (ringRadiusSq <= 0)
        {
            return Vector3(0, 0, 0); // above or below the sphere at this height
        }

        Vector3 horizontal(point.x - center.x, 0, point.z - center.z);
        real horizontalLength = horizontal.magnitude();
        if (horizontalLength <= (real)1e-6)
        {
            horizontal = Vector3(-along.z, 0, along.x); // perpendicular to the link
            horizontalLength = horizontal.magnitude();
            if (horizontalLength <= (real)1e-6)
            {
                horizontal = Vector3(1, 0, 0);
                horizontalLength = 1;
            }
        }

        const real ringRadius = real_sqrt(ringRadiusSq);
        if (horizontalLength >= ringRadius)
        {
            return Vector3(0, 0, 0);
        }
        return horizontal * ((ringRadius - horizontalLength) / horizontalLength);
    }

    // Closest points p1 + (q1 - p1) * s and p2 + (q2 - p2) * t between two segments.
    // From Ericson, Real-Time Collision Detection 5.1.9.
    void ClosestPointsSegmentSegment(Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2, real& s, real& t)
    {
        const real epsilon = (real)1e-8;
        const Vector3 d1 = q1 - p1;
        const Vector3 d2 = q2 - p2;
        const Vector3 r = p1 - p2;
        const real a = d1 * d1;
        const real e = d2 * d2;
        const real f = d2 * r;

        if (a <= epsilon && e <= epsilon)
        {
            s = t = 0;
            return;
        }
        if (a <= epsilon)
        {
            s = 0;
            t = Clamp01(f / e);
            return;
        }

        const real c = d1 * r;
        if (e <= epsilon)
        {
            t = 0;
            s = Clamp01(-c / a);
            return;
        }

        const real b = d1 * d2;
        const real denominator = a * e - b * b;
        // Parallel segments (denominator 0): any s works, pick 0.
        s = (denominator != 0) ? Clamp01((b * f - c * e) / denominator) : 0;
        t = (b * s + f) / e;
        if (t < 0)
        {
            t = 0;
            s = Clamp01(-c / a);
        }
        else if (t > 1)
        {
            t = 1;
            s = Clamp01((b - c) / a);
        }
    }

    // A box's center, world axes and half extents, for moving points in and out of its local space.
    struct BoxFrame
    {
        Vector3 center;
        Vector3 axis[3];
        Vector3 half;

        explicit BoxFrame(const RopeBoxCollider& box) : center(box.center), half(box.halfExtents)
        {
            for (int i = 0; i < 3; i++)
            {
                axis[i] = box.getAxis(i);
            }
        }

        Vector3 toLocal(Vector3 point) const
        {
            const Vector3 d = point - center;
            return Vector3(d * axis[0], d * axis[1], d * axis[2]);
        }
        Vector3 directionToWorld(Vector3 v) const { return axis[0] * v.x + axis[1] * v.y + axis[2] * v.z; }
        Vector3 toWorld(Vector3 point) const { return center + directionToWorld(point); }
    };

    // Signed distance from a local point to an axis-aligned box's surface (negative inside), with the outward normal.
    real BoxSignedDistanceLocal(Vector3 p, Vector3 half, Vector3& normal)
    {
        Vector3 q(0, 0, 0);
        bool outside = false;
        for (unsigned i = 0; i < 3; i++)
        {
            q[i] = real_abs(p[i]) - half[i];
            if (q[i] > 0)
            {
                outside = true;
            }
        }

        if (outside)
        {
            Vector3 d(0, 0, 0);
            for (unsigned i = 0; i < 3; i++)
            {
                if (q[i] > 0)
                {
                    d[i] = (p[i] < 0) ? -q[i] : q[i];
                }
            }
            const real distance = d.magnitude();
            normal = d * ((real)1.0 / distance);
            return distance;
        }

        // Inside: the nearest face is the one with the least penetration.
        unsigned k = 0;
        for (unsigned i = 1; i < 3; i++)
        {
            if (q[i] > q[k])
            {
                k = i;
            }
        }
        normal = Vector3(0, 0, 0);
        normal[k] = (p[k] < 0) ? (real)-1.0 : (real)1.0;
        return q[k];
    }

    // World-space signed distance to a box's surface, with the outward world normal.
    real BoxSignedDistance(const BoxFrame& box, Vector3 point, Vector3& normal)
    {
        Vector3 localNormal;
        const real distance = BoxSignedDistanceLocal(box.toLocal(point), box.half, localNormal);
        normal = box.directionToWorld(localNormal);
        return distance;
    }

    // Where the local path from + move * s first enters an axis-aligned box from outside (slab test).
    // False if it starts inside, misses, or enters outside s in [0,1].
    bool PathEntersBox(Vector3 from, Vector3 move, Vector3 half, real& sEnter, unsigned& axis)
    {
        real sMin = (real)-1e30, sMax = (real)1e30;
        bool inside = true, found = false;
        for (unsigned i = 0; i < 3; i++)
        {
            if (real_abs(from[i]) >= half[i])
            {
                inside = false;
            }
            if (real_abs(move[i]) < (real)1e-12)
            {
                if (real_abs(from[i]) > half[i])
                {
                    return false; // parallel to and outside this slab
                }
                continue;
            }
            real s1 = (-half[i] - from[i]) / move[i];
            real s2 = (half[i] - from[i]) / move[i];
            if (s1 > s2)
            {
                const real swap = s1;
                s1 = s2;
                s2 = swap;
            }
            if (s1 > sMin)
            {
                sMin = s1;
                axis = i;
                found = true;
            }
            if (s2 < sMax)
            {
                sMax = s2;
            }
            if (sMin > sMax)
            {
                return false;
            }
        }
        if (inside || !found || sMin < 0 || sMin > 1)
        {
            return false;
        }
        sEnter = sMin;
        return true;
    }

    // Box version of SweepSlideSphere, against the box grown by r: stops at the face it enters, then slides along it.
    // depth = motion removed along the face normal. False if it starts inside the grown box or never enters.
    bool SweepSlideBox(Vector3 from, Vector3 to, const BoxFrame& box, real r, Vector3& out, real& depth)
    {
        const Vector3 localFrom = box.toLocal(from);
        const Vector3 localMove = box.toLocal(to) - localFrom;
        const Vector3 grown = box.half + Vector3(r, r, r);

        real s;
        unsigned axis = 0;
        if (!PathEntersBox(localFrom, localMove, grown, s, axis))
        {
            return false;
        }

        // Snap onto the entered face, on the side the point came from.
        Vector3 hit = localFrom + localMove * s;
        const real side = (localFrom[axis] < 0) ? (real)-1.0 : (real)1.0;
        hit[axis] = grown[axis] * side;

        // Drop the motion into the face, keep the sliding part.
        Vector3 remaining = (localFrom + localMove) - hit;
        const real into = remaining[axis] * side;
        depth = 0;
        if (into < 0)
        {
            remaining[axis] = 0;
            depth = -into;
        }
        out = box.toWorld(hit + remaining);
        return true;
    }

    // Shortest push that takes `point` out of the box grown by r through a face that doesn't point down.
    // Used where pushing out through the bottom would go into the ground, which would just push it back.
    Vector3 PushOutOfBoxNotDown(Vector3 point, const BoxFrame& box, real r)
    {
        const Vector3 local = box.toLocal(point);
        real best = (real)1e30;
        Vector3 push(0, 0, 0);
        for (unsigned i = 0; i < 3; i++)
        {
            for (int side = -1; side <= 1; side += 2)
            {
                const Vector3 normal = box.axis[i] * (real)side;
                if (normal.y < (real)-0.5)
                {
                    continue; // a bottom face
                }
                const real amount = (box.half[i] + r) - local[i] * (real)side;
                if (amount > 0 && amount < best)
                {
                    best = amount;
                    push = normal * amount;
                }
            }
        }
        return push;
    }

    // Moves a point of the given radius from a safe spot toward a box, sweeping so it can't tunnel and rounding
    // the box's edges. False if they don't touch; minY is the ground.
    bool CollidePointBox(Vector3 from, Vector3 to, const BoxFrame& box, real r, real minY, Vector3& out, real& depth)
    {
        Vector3 normal;
        const real distance = BoxSignedDistance(box, to, normal);

        const Vector3 localFrom = box.toLocal(from);
        real s;
        unsigned axis = 0;
        const bool reachesBox = distance < 0 || PathEntersBox(localFrom, box.toLocal(to) - localFrom, box.half, s, axis);
        if (reachesBox && SweepSlideBox(from, to, box, r, out, depth))
        {
            return true;
        }

        if (distance >= r)
        {
            return false;
        }
        Vector3 push = normal * (r - distance);
        if (to.y + push.y < minY)
        {
            push = PushOutOfBoxNotDown(to, box, r);
        }
        out = to + push;
        depth = push.magnitude();
        return true;
    }

    // Interval [tEnter, tExit] of the ray origin + direction * t (unit direction) inside a sphere of radius r.
    bool RaySphereInterval(Vector3 origin, Vector3 direction, Vector3 center, real r, real& tEnter, real& tExit)
    {
        const Vector3 toOrigin = origin - center;
        const real b = toOrigin * direction;
        const real discriminant = b * b - (toOrigin.squareMagnitude() - r * r);
        if (discriminant <= 0)
        {
            return false;
        }
        const real root = real_sqrt(discriminant);
        tEnter = -b - root;
        tExit = -b + root;
        return true;
    }

    // Interval [tEnter, tExit] of a world ray inside a box with the given half extents (slab test in its local space).
    bool RayBoxInterval(const BoxFrame& box, Vector3 half, Vector3 origin, Vector3 direction, real& tEnter, real& tExit)
    {
        const Vector3 localOrigin = box.toLocal(origin);
        const Vector3 localDirection(direction * box.axis[0], direction * box.axis[1], direction * box.axis[2]);
        tEnter = (real)-1e30;
        tExit = (real)1e30;
        for (unsigned i = 0; i < 3; i++)
        {
            if (real_abs(localDirection[i]) < (real)1e-12)
            {
                if (real_abs(localOrigin[i]) >= half[i])
                {
                    return false; // parallel to and outside this slab
                }
                continue;
            }
            real t1 = (-half[i] - localOrigin[i]) / localDirection[i];
            real t2 = (half[i] - localOrigin[i]) / localDirection[i];
            if (t1 > t2)
            {
                const real swap = t1;
                t1 = t2;
                t2 = swap;
            }
            if (t1 > tEnter)
            {
                tEnter = t1;
            }
            if (t2 < tExit)
            {
                tExit = t2;
            }
            if (tEnter >= tExit)
            {
                return false;
            }
        }
        return true;
    }

    // Deepest point of link (a, b) against a box rounded by r: parameter t, outward normal and depth.
    // False if the deepest point is an end particle or the link doesn't touch the box.
    bool FindSegmentBoxContact(Vector3 a, Vector3 b, const BoxFrame& box, real r, real& t, Vector3& normal, real& depth)
    {
        // Cheap reject against the box's bounding sphere.
        const Vector3 ab = b - a;
        const Vector3 mid = (a + b) * (real)0.5;
        const real reach = ab.magnitude() * (real)0.5 + box.half.magnitude() + r;
        if ((mid - box.center).squareMagnitude() >= reach * reach)
        {
            return false;
        }

        // Signed distance to a convex box is convex along the link, so golden-section search finds its minimum.
        Vector3 n;
        auto distanceAt = [&](real u) { return BoxSignedDistance(box, a + ab * u, n); };
        const real g = (real)0.6180339887;
        real lo = 0, hi = 1;
        real x1 = hi - g * (hi - lo), x2 = lo + g * (hi - lo);
        real f1 = distanceAt(x1), f2 = distanceAt(x2);
        for (int k = 0; k < kBoxSearchSteps; k++)
        {
            if (f1 < f2)
            {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - g * (hi - lo);
                f1 = distanceAt(x1);
            }
            else
            {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + g * (hi - lo);
                f2 = distanceAt(x2);
            }
        }

        t = (lo + hi) * (real)0.5;
        if (t <= kBoxSegmentEndEpsilon || t >= (real)1.0 - kBoxSegmentEndEpsilon)
        {
            return false; // deepest point is an end particle
        }
        depth = r - distanceAt(t);
        if (depth <= 0)
        {
            return false;
        }
        normal = n;
        return true;
    }
}

Vector3 RopeBoxCollider::getAxis(int index) const
{
    // Rotate the unit axis by the orientation: v + 2r(u x v) + 2u x (u x v), with u = (i, j, k).
    Quaternion q = orientation;
    q.normalise();
    Vector3 v(0, 0, 0);
    v[(unsigned)index] = 1;
    const Vector3 u(q.i, q.j, q.k);
    const Vector3 c = u % v;
    return v + c * ((real)2.0 * q.r) + (u % c) * (real)2.0;
}

ConstraintRope::ConstraintRope(Vector3 start, Vector3 direction, real length, int segments, real radius) : radius(radius)
{
    if (segments < 1)
    {
        segments = 1;
    }

    direction.normalise();
    segmentLength = length / (real)segments;

    particles.reserve(segments + 1);
    for (int i = 0; i <= segments; i++)
    {
        VerletParticle particle;
        particle.position = start + direction * (segmentLength * (real)i);
        // The previous position matches the current one, so the rope starts at rest.
        particle.prevPosition = particle.position;
        particles.push_back(particle);
    }
}

void ConstraintRope::addSphere(const RopeSphereCollider& sphere)
{
    spheres.push_back(sphere);
}

void ConstraintRope::addBox(const RopeBoxCollider& box)
{
    boxes.push_back(box);
}

void ConstraintRope::step(real dt)
{
    if (dt <= 0)
    {
        return;
    }

    // The dragged particle has no speed limit, so a far cursor could swing a link straight through a collider in
    // one step. Splitting the step lets collisions see the whole swing.
    int substeps = 1;
    if (dragIndex >= 0 && particles[dragIndex].inverseMass > 0)
    {
        const real distance = (dragTarget - particles[dragIndex].position).magnitude();
        substeps = (int)std::ceil(distance / kMaxDragSubstepMove);
        if (substeps < 1)
        {
            substeps = 1;
        }
        if (substeps > kMaxDragSubsteps)
        {
            substeps = kMaxDragSubsteps;
        }
    }
    for (int s = 0; s < substeps; s++)
    {
        stepOnce(dt / (real)substeps);
    }
}

void ConstraintRope::stepOnce(real dt)
{
    integrate(dt);
    solveConstraints();
    applyFriction();

    // Friction can nudge the cord into a collider, so clean up.
    solveCollisionConstraints();

    // Stretched links can dip into a sphere or box; repeat the collider pass until nothing overlaps,
    // so "no clipping" holds at any iteration count.
    for (int pass = 0; pass < kMaxColliderCleanupPasses; pass++)
    {
        if (worstColliderOverlap() <= kColliderOverlapTolerance)
        {
            break;
        }
        solveColliderCollisions();
    }

    // Remembered so friction can tell whether a held particle is being dragged.
    lastPositions.resize(particles.size());
    for (size_t i = 0; i < particles.size(); i++)
    {
        lastPositions[i] = particles[i].position;
    }
    lastDragTarget = dragTarget;
}

real ConstraintRope::worstColliderOverlap() const
{
    real worst = 0;
    for (const RopeBoxCollider& collider : boxes)
    {
        const BoxFrame box(collider);
        for (size_t i = 0; i < particles.size(); i++)
        {
            Vector3 normal;
            const real overlap = radius - BoxSignedDistance(box, particles[i].position, normal);
            if (overlap > worst)
            {
                worst = overlap;
            }

            if (i + 1 < particles.size())
            {
                real t, depth;
                if (FindSegmentBoxContact(particles[i].position, particles[i + 1].position, box, radius, t, normal, depth) && depth > worst)
                {
                    worst = depth;
                }
            }
        }
    }
    for (const RopeSphereCollider& sphere : spheres)
    {
        const real r = sphere.radius + radius;
        for (size_t i = 0; i < particles.size(); i++)
        {
            const real overlap = r - (particles[i].position - sphere.center).magnitude();
            if (overlap > worst)
            {
                worst = overlap;
            }

            if (i + 1 < particles.size())
            {
                real t, depth;
                Vector3 normal;
                if (FindSegmentSphereContact(particles[i].position, particles[i + 1].position, sphere, radius, t, normal, depth) &&
                    depth > worst)
                {
                    worst = depth;
                }
            }
        }
    }
    return worst;
}

void ConstraintRope::integrate(real dt)
{
    // Damping is the fraction of velocity kept per second.
    const real velocityScale = real_pow(Clamp01(damping), dt);
    const Vector3 gravityStep = gravity * (dt * dt);

    for (size_t i = 0; i < particles.size(); i++)
    {
        VerletParticle& p = particles[i];
        if (p.inverseMass <= 0)
        {
            // Immovable: no implied velocity carried forward.
            p.prevPosition = p.position;
            continue;
        }

        // Verlet step: the velocity is implied by the last move, damped, then carried on with gravity added
        const Vector3 velocity = (p.position - p.prevPosition) * velocityScale;
        Vector3 move = velocity + gravityStep;
        if ((int)i == dragIndex)
        {
            move = dragMove(p, velocity, dt);
        }
        p.prevPosition = p.position;
        p.position += move;
    }
}

Vector3 ConstraintRope::dragMove(const VerletParticle& p, Vector3 velocity, real dt) const
{
    Vector3 toTarget = dragTarget - p.position;
    const real distance = toTarget.magnitude();

    // Damped, so the pull settles on the cursor instead of orbiting it like a spring.
    velocity *= real_exp(-kDragDamping * dt);

    // The pull fades to nothing at the cursor, is the set strength at epsilon away, and grows quickly beyond that
    Vector3 move = velocity + gravity * (dt * dt);
    if (distance > 0)
    {
        const real exponent = distance / dragEpsilon;
        const real growth = real_exp(exponent < kMaxDragExponent ? exponent : kMaxDragExponent) - (real)1.0;
        const real pull = dragStrength * growth / (real)(2.718281828459045 - 1.0);
        toTarget *= (real)1.0 / distance;
        // The pull alone never carries it past the cursor. Capping it here, before adding, also keeps a huge
        // far-away pull from swamping the rest of the move in floating point.
        real pullStep = pull * dt * dt;
        if (pullStep > distance)
        {
            pullStep = distance;
        }
        move += toTarget * pullStep;
    }

    // Never overshoot the cursor in one step (velocity carried from the last step included).
    if (distance > 0)
    {
        const real along = move * toTarget;
        if (along > distance)
        {
            move -= toTarget * (along - distance);
        }
    }
    // Per sub-step only (step() adds sub-steps to keep up), so collisions catch a fast swing.
    const real moveLength = move.magnitude();
    if (moveLength > kMaxDragSubstepMove)
    {
        move *= kMaxDragSubstepMove / moveLength;
    }
    return move;
}

void ConstraintRope::setDragTarget(int index, Vector3 target)
{
    if (index != dragIndex)
    {
        lastDragTarget = target; // a new grab hasn't moved yet
    }
    dragIndex = index;
    dragTarget = target;
}

real ConstraintRope::solveWeight(size_t i) const
{
    // The dragged particle acts like a hand holding the rope: heavy, so the rope's weight barely pulls it off
    // the cursor. Collisions still push it with its real mass.
    return (int)i == dragIndex ? particles[i].inverseMass * kDragInverseMassScale : particles[i].inverseMass;
}

bool ConstraintRope::isAnchor(size_t i) const
{
    return particles[i].inverseMass <= 0 || (int)i == dragIndex;
}

void ConstraintRope::solveConstraints()
{
    // Reset this step's record of how hard each particle hit each collider.
    groundContactDepth.assign(particles.size(), 0);
    sphereContactDepth.assign(particles.size() * spheres.size(), 0);
    boxContactDepth.assign(particles.size() * boxes.size(), 0);

    // Pre-integration positions are outside every sphere and box (last step ended on collisions).
    safePositions.resize(particles.size());
    for (size_t i = 0; i < particles.size(); i++)
    {
        safePositions[i] = particles[i].prevPosition;
    }

    // Relaxation, with collisions interleaved so links can't drag particles back into colliders.
    for (int i = 0; i < constraintIterations; i++)
    {
        solveDistanceConstraints(i);
        solveBendConstraints();
        solveCollisionConstraints();
    }

    // Remove leftover stretch, then collide. Collisions can re-stretch a link, so alternate a few times.
    for (int pass = 0; pass < kFollowTheLeaderPasses; pass++)
    {
        solveFollowTheLeader();
        solveCollisionConstraints();
    }
}

void ConstraintRope::solveFollowTheLeader(real maxStretch)
{
    // Placing each particle one segment out from its leader makes every link exact in one pass. It only fixes
    // lengths, since straightening bends here would ignore collisions and make pushed rope run away.
    const int last = (int)particles.size() - 1;

    // The free ends of the rope hang off the first and last held (or dragged) particles; between them is left
    // to relaxation.
    int firstAnchor = -1, lastAnchor = -1;
    for (int i = 0; i <= last; i++)
    {
        if (isAnchor((size_t)i))
        {
            if (firstAnchor < 0)
            {
                firstAnchor = i;
            }
            lastAnchor = i;
        }
    }
    if (firstAnchor < 0)
    {
        return; // nothing held: nothing to follow
    }

    // Dynamic FTL (Muller et al. 2012): take most of each correction back off the leader's velocity,
    // or Verlet turns it into energy and flings the rope. The held particle's velocity is left alone.
    auto follow = [&](int leader, int follower)
    {
        VerletParticle& p = particles[follower];
        VerletParticle& lead = particles[leader];
        const Vector3 delta = p.position - lead.position;
        const real length = delta.magnitude();
        if (length <= 0)
        {
            return;
        }
        real target = segmentLength;
        if (maxStretch >= 0)
        {
            // Only take out stretch beyond the tolerance, and never push a link apart.
            target = segmentLength * ((real)1.0 + maxStretch);
            if (length <= target)
            {
                return;
            }
        }
        const Vector3 correction = lead.position + delta * (target / length) - p.position;
        p.position += correction;
        if (lead.inverseMass > 0 && leader != dragIndex)
        {
            lead.prevPosition += correction * kFollowTheLeaderVelocityCorrection;
        }
    };

    for (int i = lastAnchor; i < last; i++)
    {
        follow(i, i + 1);
    }
    for (int i = firstAnchor; i > 0; i--)
    {
        follow(i, i - 1);
    }
}

void ConstraintRope::solveDistanceConstraints(int iteration)
{
    const int linkCount = (int)particles.size() - 1;

    // Solving outward from a held particle carries its pull down the whole rope in one pass;
    // solving toward it only moves the pull one link per pass.
    int anchor = -1;
    for (int i = 0; i <= linkCount; i++)
    {
        if (isAnchor((size_t)i))
        {
            anchor = i;
            break;
        }
    }

    if (anchor >= 0)
    {
        for (int i = anchor; i < linkCount; i++)
        {
            solveDistanceConstraint(i); // anchor toward the far end
        }
        for (int i = anchor - 1; i >= 0; i--)
        {
            solveDistanceConstraint(i); // anchor toward particle 0
        }
    }
    else
    {
        // Nothing held: alternate directions so neither end is favoured.
        for (int link = 0; link < linkCount; link++)
        {
            solveDistanceConstraint((iteration % 2 == 0) ? link : linkCount - 1 - link);
        }
    }
}

void ConstraintRope::solveDistanceConstraint(int i)
{
    VerletParticle& a = particles[i];
    VerletParticle& b = particles[i + 1];
    const real wa = solveWeight((size_t)i), wb = solveWeight((size_t)i + 1);

    const real inverseMassSum = wa + wb;
    if (inverseMassSum <= 0)
    {
        return;
    }

    const Vector3 delta = b.position - a.position;
    const real deltaLength = delta.magnitude();
    if (deltaLength <= 0)
    {
        return;
    }

    // Move both ends along the link, each by its share of the inverse mass.
    const real diff = (deltaLength - segmentLength) / (deltaLength * inverseMassSum);
    a.position += delta * (wa * diff);
    b.position -= delta * (wb * diff);
}

void ConstraintRope::solveBendConstraints()
{
    if (maxBendAngle >= 180)
    {
        return; // unconstrained
    }

    // Both limits come from the triangle two links make at the largest allowed bend: how close their far ends
    // can get, and how far the middle particle can sit from the line between them.
    const real halfBend = maxBendAngle * (real)(3.14159265358979 / 180.0) * (real)0.5;
    const real minDistance = (real)2.0 * segmentLength * real_cos(halfBend);
    const real maxOffset = segmentLength * real_sin(halfBend);

    // b to the midpoint of ac. The a-c constraint alone never moves b, so a stiff rope has to rely on the links
    // to drag b back into line, which converges slowly and leaves it sagging and wiggling.
    for (size_t i = 0; i + 2 < particles.size(); i++)
    {
        VerletParticle& a = particles[i];
        VerletParticle& b = particles[i + 1];
        VerletParticle& c = particles[i + 2];
        const real wa = solveWeight(i), wb = solveWeight(i + 1), wc = solveWeight(i + 2);

        const Vector3 offset = b.position - (a.position + c.position) * (real)0.5;
        const real offsetLength = offset.magnitude();
        if (offsetLength <= maxOffset || offsetLength <= 0)
        {
            continue;
        }

        // Gradients: n for b, -n/2 for a and c, so the weight is wb + (wa + wc) / 4.
        const real weightSum = wb + (wa + wc) * (real)0.25;
        if (weightSum <= 0)
        {
            continue;
        }
        const Vector3 n = offset * ((real)1.0 / offsetLength);
        const real lambda = (offsetLength - maxOffset) / weightSum;
        b.position -= n * (lambda * wb);
        a.position += n * (lambda * wa * (real)0.5);
        c.position += n * (lambda * wc * (real)0.5);
    }

    // a to c: one-sided distance constraint that only pushes them apart, mass-weighted like the links.
    for (size_t i = 0; i + 2 < particles.size(); i++)
    {
        VerletParticle& a = particles[i];
        VerletParticle& c = particles[i + 2];
        const real wa = solveWeight(i), wc = solveWeight(i + 2);

        const real inverseMassSum = wa + wc;
        if (inverseMassSum <= 0)
        {
            continue;
        }

        const Vector3 delta = c.position - a.position;
        const real deltaLength = delta.magnitude();
        if (deltaLength >= minDistance || deltaLength <= 0)
        {
            continue; // bent less than the limit (or folded exactly flat, with no direction to push)
        }

        const real diff = (deltaLength - minDistance) / (deltaLength * inverseMassSum);
        a.position += delta * (wa * diff);
        c.position -= delta * (wc * diff);
    }
}

void ConstraintRope::solveCollisionConstraints()
{
    // Self first, then the environment, so the cord is never left inside the ground, a sphere or a box.
    if (selfCollision)
    {
        solveSelfCollisions();
    }
    solveGroundCollisions();
    solveColliderCollisions();
}

void ConstraintRope::solveColliderCollisions()
{
    solveSphereCollisions();
    solveBoxCollisions();
    solveGroundCorners();
    updateSafePositions();
}

void ConstraintRope::solveSelfCollisions()
{
    // Links are capsules of the rope's radius. Links closer along the rope than the cord is thick always
    // touch (adjacent ones share a particle), so skip them.
    const real minDistance = radius * (real)2.0;
    const int segmentCount = (int)particles.size() - 1;
    int skip = 1;
    while (skip * segmentLength < minDistance && skip < segmentCount)
    {
        skip++;
    }

    for (int i = 0; i < segmentCount; i++)
    {
        for (int j = i + 1 + skip; j < segmentCount; j++)
        {
            VerletParticle& a = particles[i];
            VerletParticle& b = particles[i + 1];
            VerletParticle& c = particles[j];
            VerletParticle& d = particles[j + 1];

            // Cheap reject: the links' midpoints are too far apart to touch.
            const Vector3 midA = (a.position + b.position) * (real)0.5;
            const Vector3 midB = (c.position + d.position) * (real)0.5;
            const real reach = (real)0.5 * ((b.position - a.position).magnitude() + (d.position - c.position).magnitude()) + minDistance;
            if ((midA - midB).squareMagnitude() >= reach * reach)
            {
                continue;
            }

            real s, t;
            ClosestPointsSegmentSegment(a.position, b.position, c.position, d.position, s, t);
            const Vector3 p = a.position + (b.position - a.position) * s;
            const Vector3 q = c.position + (d.position - c.position) * t;
            const Vector3 offset = p - q;
            const real distance = offset.magnitude();
            if (distance >= minDistance)
            {
                continue;
            }

            Vector3 normal;
            if (distance > 0)
            {
                normal = offset * ((real)1.0 / distance);
            }
            else
            {
                // Segments cross exactly: separate along their common normal.
                normal = (b.position - a.position) % (d.position - c.position);
                if (normal.squareMagnitude() <= 0)
                {
                    normal = Vector3(0, 1, 0);
                }
                normal.normalise();
            }

            // Push the closest points apart, split over the four particles by
            // inverse mass and closeness to the contact (generalised inverse mass).
            const real wa = ((real)1.0 - s) * ((real)1.0 - s) * a.inverseMass;
            const real wb = s * s * b.inverseMass;
            const real wc = ((real)1.0 - t) * ((real)1.0 - t) * c.inverseMass;
            const real wd = t * t * d.inverseMass;
            const real weightSum = wa + wb + wc + wd;
            if (weightSum <= 0)
            {
                continue;
            }

            const Vector3 push = normal * ((minDistance - distance) / weightSum);
            a.position += push * (((real)1.0 - s) * a.inverseMass);
            b.position += push * (s * b.inverseMass);
            c.position -= push * (((real)1.0 - t) * c.inverseMass);
            d.position -= push * (t * d.inverseMass);
        }
    }
}

void ConstraintRope::solveGroundCollisions()
{
    // Project onto the plane. A link between two particles above a plane is above it too.
    const real minY = groundHeight + radius;
    for (size_t i = 0; i < particles.size(); i++)
    {
        VerletParticle& p = particles[i];
        if (p.inverseMass > 0 && p.position.y < minY)
        {
            groundContactDepth[i] += minY - p.position.y;
            p.position.y = minY;
        }
    }
}

void ConstraintRope::solveSphereCollisions()
{
    // Particles: sweep from the last safe position, so each is pushed out on the side it came from.
    for (size_t i = 0; i < particles.size(); i++)
    {
        VerletParticle& p = particles[i];
        if (p.inverseMass <= 0)
        {
            continue;
        }
        for (size_t s = 0; s < spheres.size(); s++)
        {
            const RopeSphereCollider& sphere = spheres[s];
            const real r = sphere.radius + radius;

            Vector3 slid;
            real depth;
            if (SweepSlideSphere(safePositions[i], p.position, sphere.center, r, slid, depth))
            {
                p.position = slid;
                sphereContactDepth[sphereDepthIndex(i, s)] += depth;
                continue;
            }

            // No usable path (e.g. started inside after a reset): use the nearest surface point.
            const Vector3 offset = p.position - sphere.center;
            const real distance = offset.magnitude();
            if (distance < r)
            {
                const Vector3 normal = (distance > 0) ? offset * ((real)1.0 / distance) : Vector3(0, 1, 0);
                p.position = sphere.center + normal * r;
                sphereContactDepth[sphereDepthIndex(i, s)] += r - distance;
            }
        }
    }

    // Links: a link can cut through a sphere with both ends outside, so treat each as a capsule.
    for (int i = 0; i + 1 < (int)particles.size(); i++)
    {
        for (size_t s = 0; s < spheres.size(); s++)
        {
            real t, depth;
            Vector3 normal;
            if (!FindSegmentSphereContact(particles[i].position, particles[i + 1].position, spheres[s], radius, t, normal, depth))
            {
                continue;
            }

            Vector3 push = normal * depth;
            const Vector3 a = particles[i].position, b = particles[i + 1].position;
            const Vector3 contact = a + (b - a) * t;
            if (contact.y + push.y < groundHeight + radius)
            {
                // Under the sphere the push points into the ground, which lifts the link straight back, so it stays
                // stuck under the sphere. Push it sideways instead, onto the ring where the sphere meets its height.
                push = SidewaysOutOfSphere(contact, b - a, spheres[s].center, spheres[s].radius + radius);
            }

            real movedA = 0, movedB = 0;
            moveSegmentPoint(i, i + 1, t, push, &movedA, &movedB);
            sphereContactDepth[sphereDepthIndex(i, s)] += movedA;
            sphereContactDepth[sphereDepthIndex(i + 1, s)] += movedB;
        }
    }
}

void ConstraintRope::solveBoxCollisions()
{
    for (size_t b = 0; b < boxes.size(); b++)
    {
        const BoxFrame box(boxes[b]);

        // Particles: resolved along the path from the last safe position, as with spheres.
        for (size_t i = 0; i < particles.size(); i++)
        {
            VerletParticle& p = particles[i];
            if (p.inverseMass <= 0)
            {
                continue;
            }
            Vector3 resolved;
            real depth;
            if (CollidePointBox(safePositions[i], p.position, box, radius, groundHeight + radius, resolved, depth))
            {
                p.position = resolved;
                boxContactDepth[boxDepthIndex(i, b)] += depth;
            }
        }

        // Links: a link can cut across an edge or corner with both ends outside.
        for (int i = 0; i + 1 < (int)particles.size(); i++)
        {
            real t, depth;
            Vector3 normal;
            if (!FindSegmentBoxContact(particles[i].position, particles[i + 1].position, box, radius, t, normal, depth))
            {
                continue;
            }

            Vector3 push = normal * depth;
            const Vector3 contact = particles[i].position + (particles[i + 1].position - particles[i].position) * t;
            if (contact.y + push.y < groundHeight + radius)
            {
                // Pushing out through the bottom would go into the ground, which lifts it straight back.
                push = PushOutOfBoxNotDown(contact, box, radius);
            }

            real movedA = 0, movedB = 0;
            moveSegmentPoint(i, i + 1, t, push, &movedA, &movedB);
            boxContactDepth[boxDepthIndex(i, b)] += movedA;
            boxContactDepth[boxDepthIndex(i + 1, b)] += movedB;
        }
    }
}

void ConstraintRope::solveGroundCorners()
{
    // Lift particles pushed below ground beside a collider, then free any stuck in a sphere's crevice.
    const real minY = groundHeight + radius;
    for (size_t i = 0; i < particles.size(); i++)
    {
        VerletParticle& p = particles[i];
        if (p.inverseMass <= 0 || p.position.y >= minY + (real)0.0001)
        {
            continue;
        }
        if (p.position.y < minY)
        {
            groundContactDepth[i] += minY - p.position.y;
            p.position.y = minY;
        }
        for (size_t s = 0; s < spheres.size(); s++)
        {
            sphereContactDepth[sphereDepthIndex(i, s)] +=
                PushOutOfGroundCorner(p.position, minY, spheres[s].center, spheres[s].radius + radius);
        }
    }
}

void ConstraintRope::updateSafePositions()
{
    // Only remember positions that really are outside every sphere and box.
    for (size_t i = 0; i < particles.size(); i++)
    {
        bool outside = true;
        for (const RopeSphereCollider& sphere : spheres)
        {
            const real r = sphere.radius + radius;
            if ((particles[i].position - sphere.center).squareMagnitude() < r * r * (real)0.999)
            {
                outside = false;
                break;
            }
        }
        for (size_t b = 0; outside && b < boxes.size(); b++)
        {
            Vector3 normal;
            if (BoxSignedDistance(BoxFrame(boxes[b]), particles[i].position, normal) < radius * (real)0.999)
            {
                outside = false;
            }
        }
        if (outside)
        {
            safePositions[i] = particles[i].position;
        }
    }
}

void ConstraintRope::applyFriction()
{
    if (!frictionEnabled)
    {
        return;
    }

    // Is a held or dragged particle moving (being dragged), or is it held still?
    bool anyHeld = false, dragging = false;
    const bool havePositions = lastPositions.size() == particles.size();
    for (size_t i = 0; i < particles.size(); i++)
    {
        if (isAnchor(i))
        {
            anyHeld = true;
            if (havePositions && (particles[i].position - lastPositions[i]).squareMagnitude() > (real)1e-8)
            {
                dragging = true;
            }
        }
    }
    if (dragIndex >= 0 && (dragTarget - lastDragTarget).squareMagnitude() > (real)1e-10)
    {
        dragging = true; // the cursor moved
    }

    // While dragging, friction only takes away velocity: undoing this step's slide would fight the drag and
    // stretch the rope. Otherwise it also undoes the slide, so sticky contacts stay exactly in place.
    const bool keepPositions = dragging;

    std::vector<size_t> held;
    for (size_t i = 0; i < particles.size(); i++)
    {
        VerletParticle& p = particles[i];
        if (isAnchor(i))
        {
            continue; // held, or pulled by the mouse
        }

        bool stopped = applyContactFriction(p, Vector3(0, 1, 0), groundContactDepth[i], groundFriction, keepPositions);

        for (size_t s = 0; s < spheres.size(); s++)
        {
            const Vector3 offset = p.position - spheres[s].center;
            const real distance = offset.magnitude();
            if (distance > 0)
            {
                stopped |= applyContactFriction(p, offset * ((real)1.0 / distance), sphereContactDepth[sphereDepthIndex(i, s)],
                                                spheres[s].friction, keepPositions);
            }
        }

        for (size_t b = 0; b < boxes.size(); b++)
        {
            const real penetration = boxContactDepth[boxDepthIndex(i, b)];
            if (penetration <= 0)
            {
                continue;
            }
            // Normal of the nearest face, edge or corner of the box.
            Vector3 normal;
            BoxSignedDistance(BoxFrame(boxes[b]), p.position, normal);
            stopped |= applyContactFriction(p, normal, penetration, boxes[b].friction, keepPositions);
        }

        if (stopped)
        {
            held.push_back(i);
        }
    }

    if (dragging)
    {
        return; // positions untouched, so nothing to re-solve
    }

    // Keep the stopped particles in place and re-solve the rest against them.
    holdFrictionContacts(held);

    if (anyHeld)
    {
        // Held still: the pins can leave the rope stretched toward the hand from the last drag. Pull it taut behind
        // the held particle, ignoring small leftover stretch so the rope doesn't slowly slide toward the hand.
        for (int pass = 0; pass < kFrictionHoldPasses; pass++)
        {
            solveFollowTheLeader(kDragStretchTolerance);
            solveCollisionConstraints();
        }
    }
}

void ConstraintRope::holdFrictionContacts(const std::vector<size_t>& held)
{
    if (held.empty())
    {
        return;
    }

    // Friction pulled the stopped particles back after the links were solved, which stretches them.
    // Pin those particles and re-solve, so the free rope hangs off them at rest length.
    std::vector<real> savedInverseMass(held.size());
    for (size_t k = 0; k < held.size(); k++)
    {
        savedInverseMass[k] = particles[held[k]].inverseMass;
        particles[held[k]].inverseMass = 0;
    }

    for (int pass = 0; pass < kFrictionHoldPasses; pass++)
    {
        solveDistanceConstraints(pass);
        solveBendConstraints();
        solveFollowTheLeader();
        solveCollisionConstraints();
    }

    for (size_t k = 0; k < held.size(); k++)
    {
        particles[held[k]].inverseMass = savedInverseMass[k];
    }
}

bool ConstraintRope::applyContactFriction(VerletParticle& p, Vector3 normal, real penetration, real friction, bool keepPosition)
{
    // Sliding friction that grows with how deep the contact is, slowing the sideways motion but never reversing it
    if (penetration <= 0 || friction <= 0)
    {
        return false;
    }

    const Vector3 velocity = p.position - p.prevPosition;
    const Vector3 normalVelocity = normal * (velocity * normal);
    const Vector3 tangentVelocity = velocity - normalVelocity;
    const real tangentSpeed = tangentVelocity.magnitude();
    if (tangentSpeed <= 0)
    {
        return true; // at rest in contact
    }

    // The slider runs from frictionless at 0, through ordinary friction at a half, to no sliding at all at 1
    real impulse = tangentSpeed;
    if (friction < 1)
    {
        const real mu = friction / ((real)1.0 - friction);
        const real coulomb = mu * penetration;
        if (coulomb < impulse)
        {
            impulse = coulomb;
        }
    }
    const Vector3 newTangentVelocity = tangentVelocity * ((real)1.0 - impulse / tangentSpeed);

    if (keepPosition)
    {
        // Velocity only: keep where the constraints put it, carry less speed into the next step.
        p.prevPosition = p.position - (normalVelocity + newTangentVelocity);
    }
    else
    {
        // Write back through position (not prevPosition) so a sticky contact undoes this step's slide too.
        p.position = p.prevPosition + normalVelocity + newTangentVelocity;
    }

    // Stopped means friction held it in place (static contact).
    return impulse >= tangentSpeed;
}

void ConstraintRope::moveSegmentPoint(int a, int b, real t, Vector3 delta, real* movedA, real* movedB)
{
    VerletParticle& pa = particles[a];
    VerletParticle& pb = particles[b];

    const real wa = ((real)1.0 - t) * pa.inverseMass;
    const real wb = t * pb.inverseMass;

    // The point lies between a and b; this weighs how much each should move so the point shifts by delta
    const real denominator = ((real)1.0 - t) * wa + t * wb;
    if (denominator <= 0)
    {
        return;
    }

    const Vector3 scaled = delta * ((real)1.0 / denominator);
    pa.position += scaled * wa;
    pb.position += scaled * wb;

    const real scaledLength = scaled.magnitude();
    if (movedA)
    {
        *movedA = scaledLength * wa;
    }
    if (movedB)
    {
        *movedB = scaledLength * wb;
    }
}

void ConstraintRope::setParticlePosition(int index, Vector3 position)
{
    particles[index].position = position;
    particles[index].prevPosition = position;
}

Vector3 ConstraintRope::pushOutOfColliders(Vector3 position) const
{
    for (const RopeSphereCollider& sphere : spheres)
    {
        const real r = sphere.radius + radius;
        const Vector3 offset = position - sphere.center;
        const real distance = offset.magnitude();
        if (distance < r)
        {
            const Vector3 normal = (distance > 0) ? offset * ((real)1.0 / distance) : Vector3(0, 1, 0);
            position = sphere.center + normal * r;
        }
    }

    for (const RopeBoxCollider& box : boxes)
    {
        Vector3 normal;
        const real distance = BoxSignedDistance(BoxFrame(box), position, normal);
        if (distance < radius)
        {
            position += normal * (radius - distance);
        }
    }

    const real minY = groundHeight + radius;
    if (position.y < minY)
    {
        position.y = minY;
    }

    // Lifting off the ground can put it back in a sphere's crevice.
    for (const RopeSphereCollider& sphere : spheres)
    {
        PushOutOfGroundCorner(position, minY, sphere.center, sphere.radius + radius);
    }
    return position;
}

Vector3 ConstraintRope::sweepPointToward(Vector3 from, Vector3 to) const
{
    from = pushOutOfColliders(from);

    // Follow the path, stopping and sliding wherever it enters a sphere or box.
    Vector3 position = to;
    for (const RopeSphereCollider& sphere : spheres)
    {
        Vector3 slid;
        real depth;
        if (SweepSlideSphere(from, position, sphere.center, sphere.radius + radius, slid, depth))
        {
            position = slid;
        }
    }
    for (const RopeBoxCollider& box : boxes)
    {
        Vector3 slid;
        real depth;
        if (CollidePointBox(from, position, BoxFrame(box), radius, groundHeight + radius, slid, depth))
        {
            position = slid;
        }
    }

    // Ground, plus the corner where the ground meets a sphere.
    return pushOutOfColliders(position);
}

Vector3 ConstraintRope::constrainDragTarget(Vector3 origin, Vector3 direction, real distance) const
{
    direction.normalise();
    real t = distance;

    // A target inside a collider moves back along the ray to where the ray enters it: the surface under the cursor.
    for (const RopeSphereCollider& sphere : spheres)
    {
        real tEnter, tExit;
        if (RaySphereInterval(origin, direction, sphere.center, sphere.radius + radius, tEnter, tExit) && t > tEnter && t < tExit &&
            tEnter > 0)
        {
            t = tEnter;
        }
    }
    for (const RopeBoxCollider& collider : boxes)
    {
        const BoxFrame box(collider);
        real tEnter, tExit;
        if (RayBoxInterval(box, box.half + Vector3(radius, radius, radius), origin, direction, tEnter, tExit) && t > tEnter && t < tExit &&
            tEnter > 0)
        {
            t = tEnter;
        }
    }

    return origin + direction * t;
}
