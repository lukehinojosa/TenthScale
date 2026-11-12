#pragma once
#include <vector>
#include "core.h"
#include "rigidBody.h"
#include "threadPool.h"

namespace cyclone
{
    enum class ColliderType
    {
        Sphere = 0,
        Plane = 1,
        Box = 2,
        Convex = 3
    };

    // Base class for collision shapes. Attach one to a rigid body, or leave it unattached as static scenery.
    class Collider
    {
    public:
        ColliderType type; // Used to identify the type of collider easily
        RigidBody* pBody = nullptr; // Optional: a static collider does not need a rigidbody
        Matrix4 offset; // Transform of this collider relative to the center of the rigidbody's local space
        Matrix4 transform; // Total local to world transform, accounting for rigidbody transform and offset
        int overlapCount = 0; // How many colliders this collider is currently overlapping

        virtual ~Collider() = default;

        // Recalculates the world transform. Call this once per frame, after
        // the rigidbody has been integrated and before any overlap tests.
        void updateInternals();

        // Gets a column of the transform matrix.
        // 0 = x axis, 1 = y axis, 2 = z axis, 3 = position
        Vector3 getAxis(int index) const { return transform.getAxisVector(index); }

    protected:
        explicit Collider(ColliderType type, RigidBody* body = nullptr) : type(type), pBody(body) {}
    };

    class SphereCollider : public Collider
    {
    public:
        real radius;

        explicit SphereCollider(real radius = 1, RigidBody* body = nullptr) : Collider(ColliderType::Sphere, body), radius(radius) {}
    };

    // An infinite plane, fixed in world space. The defaults give flat ground through the origin, facing up.
    class PlaneCollider : public Collider
    {
    public:
        Vector3 normal; // Direction of the plane (unit length)
        real offset; // Offset along the normal from the origin

        explicit PlaneCollider(Vector3 normal = Vector3(0, 1, 0), real offset = 0)
            : Collider(ColliderType::Plane), normal(normal), offset(offset)
        {
        }
    };

    // An oriented box; its orientation comes from its transform.
    class BoxCollider : public Collider
    {
    public:
        Vector3 halfSize; // Half the size of the box on the x, y, z axes

        explicit BoxCollider(Vector3 halfSize = Vector3(0.5f, 0.5f, 0.5f), RigidBody* body = nullptr)
            : Collider(ColliderType::Box, body), halfSize(halfSize)
        {
        }
    };

    // A convex polyhedron from vertices and faces in its own space (chapter 13.4.3). Centre the vertices on the
    // centre of mass, since that's where the body's centre is.
    class ConvexCollider : public Collider
    {
    public:
        struct Face
        {
            std::vector<int> indices; // Counter-clockwise seen from outside
            Vector3 normal; // Outward, unit length
            real offset = 0; // How far the face is from the shape's centre along its normal
        };

        explicit ConvexCollider(RigidBody* body = nullptr) : Collider(ColliderType::Convex, body) {}

        // Any convex shape: each face lists its corners. Winding is fixed up to face outward.
        void setShape(const std::vector<Vector3>& vertices, const std::vector<std::vector<int>>& faces);

        static ConvexCollider box(const Vector3& halfSize, RigidBody* body = nullptr);
        // A triangular prism: a ramp rising along +x, as deep as halfSize.z
        static ConvexCollider wedge(const Vector3& halfSize, RigidBody* body = nullptr);
        static ConvexCollider tetrahedron(real size, RigidBody* body = nullptr);
        static ConvexCollider octahedron(real radius, RigidBody* body = nullptr);

        // The exact inertia tensor of the solid shape with this mass, about its centroid
        Matrix3 computeInertiaTensor(real mass) const;
        Vector3 getCentroid() const;
        real getVolume() const;
        real getBoundingRadius() const { return boundingRadius; }

        const std::vector<Vector3>& getVertices() const { return vertices; }
        const std::vector<Face>& getFaces() const { return faces; }
        const std::vector<std::pair<int, int>>& getEdges() const { return edges; }
        const std::vector<Vector3>& getEdgeDirections() const { return edgeDirections; }

    private:
        std::vector<Vector3> vertices;
        std::vector<Face> faces;
        std::vector<std::pair<int, int>> edges;
        std::vector<Vector3> edgeDirections; // Unique up to sign, for the separating axis test
        real boundingRadius = 0;
    };

    // The axis that last separated a pair of shapes (chapter 13.5). Shapes move little between frames, so it
    // usually still separates them and the full test is skipped.
    struct SatCache
    {
        int separatingAxis = -1;
        bool lastTestHit = false; // Whether the last test was settled by the cached axis alone
    };

    // Yes or no overlap tests between pairs of shapes. Call each collider's updateInternals() first.
    class IntersectionTests
    {
    public:
        static bool SphereSphere(const SphereCollider& a, const SphereCollider& b);
        static bool SpherePlane(const SphereCollider& sphere, const PlaneCollider& plane);
        static bool SphereBox(const SphereCollider& sphere, const BoxCollider& box);
        static bool BoxBox(const BoxCollider& a, const BoxCollider& b);
        static bool BoxPlane(const BoxCollider& box, const PlaneCollider& plane);

        // Checks the types of both colliders and calls the matching test above.
        // Pairs with no test (plane-plane) return false.
        static bool Overlap(const Collider& a, const Collider& b);

        // Updates every collider's transform, then tests every unique pair and
        // sets each collider's overlapCount.
        static void UpdateOverlaps(const std::vector<Collider*>& colliders);
    };

    // A single point where two bodies touch.
    class Contact
    {
    public:
        Vector3 point; // The point of contact (or an approximation)
        Vector3 normal; // The direction, pointing from B to A
        real penetration; // How deeply the objects overlap along the normal
        RigidBody* body[2]; // The two bodies involved. Either may be null for static geometry.
        real friction = 0; // Used by RigidContactResolver; ContactResolver ignores it
        real restitution = 0; // Used by RigidContactResolver; ContactResolver has its own
    };

    // The contact array the collision tests write into. Point contactArray at enough contacts and call reset()
    // at the start of each frame.
    class CollisionData
    {
    public:
        Contact* contactArray = nullptr; // Start of contact array
        Contact* contacts = nullptr; // Current spot in contact array
        int contactCount = 0; // Amount of contacts found so far
        int contactsLeft = 0; // Amount of contacts remaining
        real friction = 0; // Written into every contact a test adds
        real restitution = 0;

        // Points contacts back at the start of the array and clears the count.
        void reset(int maxContacts);

        // Records that num contacts were written at the current spot, and
        // moves the current spot past them.
        void addContacts(int num);

        // Fills in the next contact (bodies, friction, restitution) and adds it; null when the array is full
        Contact* addContact(RigidBody* a, RigidBody* b, const Vector3& point, const Vector3& normal, real penetration);
    };

    // Contact generation between pairs of shapes. Each test returns how many contacts it added; call each
    // collider's updateInternals() first.
    class CollisionTests
    {
    public:
        static int SphereSphere(const SphereCollider& a, const SphereCollider& b, CollisionData* data);

        // The same test from each sphere's centre, radius and body, for a broad phase that keeps centres packed
        // together
        static int SphereSphere(const Vector3& positionA, real radiusA, RigidBody* bodyA, const Vector3& positionB, real radiusB,
                                RigidBody* bodyB, CollisionData* data);

        // Treats the plane as a half-space: everything behind it is solid, so
        // the contact normal is always the plane normal.
        static int SphereHalfspace(const SphereCollider& sphere, const PlaneCollider& plane, CollisionData* data);

        // The same test from just the sphere's center, radius and body
        static int SphereHalfspace(const Vector3& position, real radius, RigidBody* body, const PlaneCollider& plane, CollisionData* data);

        // A plane solid on neither side: the sphere is pushed back to whichever side its center is on
        static int SphereTruePlane(const SphereCollider& sphere, const PlaneCollider& plane, CollisionData* data);

        // One contact for each corner of the box behind or just above the plane
        static int BoxHalfspace(const BoxCollider& box, const PlaneCollider& plane, CollisionData* data);
        static int BoxSphere(const BoxCollider& box, const SphereCollider& sphere, CollisionData* data);

        // Separating axis test. A face contact adds one at each corner of where the faces overlap, which keeps stacks
        // steady; an edge contact adds one.
        static int BoxBox(const BoxCollider& a, const BoxCollider& b, CollisionData* data, SatCache* cache = nullptr);

        static int ConvexHalfspace(const ConvexCollider& convex, const PlaneCollider& plane, CollisionData* data);
        static int ConvexSphere(const ConvexCollider& convex, const SphereCollider& sphere, CollisionData* data);
        static int ConvexConvex(const ConvexCollider& a, const ConvexCollider& b, CollisionData* data, SatCache* cache = nullptr);
        static int ConvexBox(const ConvexCollider& convex, const BoxCollider& box, CollisionData* data, SatCache* cache = nullptr);

        // Checks both types and calls the matching test. Planes count as half-spaces; plane-plane adds nothing.
        static int Generate(const Collider& a, const Collider& b, CollisionData* data, SatCache* cache = nullptr);
    };

    // Anything besides collision tests that creates contacts each frame, such as joints. Chapter 17.
    class ContactGenerator
    {
    public:
        virtual ~ContactGenerator() = default;

        // Writes up to limit contacts and returns how many it wrote
        virtual unsigned addContact(Contact* contact, unsigned limit) const = 0;
    };

    // Fixes velocities with impulses, then moves bodies apart. Linear only, for spheres that don't need to spin;
    // RigidContactResolver handles rotation and friction.
    class ContactResolver
    {
    public:
        // Coefficient of restitution, from 0 (objects stick) to 1 (purely elastic)
        real restitution;

        // Passes over the contacts per call. Pushing one contact apart can push a body into another, so piles need
        // several.
        int iterations;

        explicit ContactResolver(real restitution = 0.5f, int iterations = 1) : restitution(restitution), iterations(iterations) {}

        void resolveContacts(Contact* contacts, int contactCount);

        // The same, with contacts split into batches resolved side by side on threadPool. Batches in a phase must not
        // share bodies; the result is the same whatever the thread count, or with no pool at all.
        void resolveContacts(Contact* contacts, const std::vector<int>& batchBounds, const std::vector<int>& phaseBounds,
                             ThreadPool* threadPool);

    private:
        // Resolves contacts[i], using its separation from the start of resolving
        void resolveContact(Contact* contacts, int i);
        void resolveVelocity(const Contact& contact);
        void resolveInterpenetration(const Contact& contact);

        // Each contact's separation along its normal when resolving started.
        // Kept between calls so it doesn't reallocate every frame.
        std::vector<real> m_startSeparation;
    };
}
