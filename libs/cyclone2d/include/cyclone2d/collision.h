#pragma once

#include <vector>

#include "core.h"
#include "rigidBody.h"

namespace cyclone2d
{
    enum class ColliderType
    {
        Circle = 0,
        Polygon = 1,
        Plane = 2
    };

    // Base class for collision shapes. Attach one to a rigid body, or leave it unattached as static scenery placed by
    // its offset.
    class Collider
    {
    public:
        ColliderType type;
        RigidBody* pBody = nullptr;
        Transform offset; // Relative to the body, or to the world when there is no body
        Transform transform; // Local to world, worked out by updateInternals()

        virtual ~Collider() = default;

        // Call once per frame, after integration and before any collision tests
        void updateInternals();

        Vector2 getPosition() const { return transform.position; }

        // How far the shape reaches from its centre; negative for planes, which reach everywhere
        real getBoundingRadius() const;

        // Whether a world point is inside the shape, using the transform from the last updateInternals()
        bool containsPoint(const Vector2& worldPoint) const;

    protected:
        explicit Collider(ColliderType type, RigidBody* body = nullptr) : type(type), pBody(body) {}
    };

    class CircleCollider : public Collider
    {
    public:
        real radius;

        explicit CircleCollider(real radius = (real)0.5, RigidBody* body = nullptr) : Collider(ColliderType::Circle, body), radius(radius) {}
    };

    // A convex polygon from vertices in its own space. Centre them on the centre of mass, since that's where the body's
    // centre is.
    class PolygonCollider : public Collider
    {
    public:
        explicit PolygonCollider(RigidBody* body = nullptr) : Collider(ColliderType::Polygon, body) {}
        PolygonCollider(const std::vector<Vector2>& vertices, RigidBody* body = nullptr);

        // Takes the convex hull of the points, so any order works, and duplicates or points inside are dropped
        void setVertices(const std::vector<Vector2>& points);

        // n corners spread evenly around a circle of the given radius
        static PolygonCollider regular(int sides, real radius, RigidBody* body = nullptr);

        const std::vector<Vector2>& getVertices() const { return vertices; }
        const std::vector<Vector2>& getNormals() const { return normals; } // Outward, one per edge from vertex i to i+1
        int getVertexCount() const { return (int)vertices.size(); }

        Vector2 getWorldVertex(int index) const { return transform.transform(vertices[(std::size_t)index]); }
        Vector2 getWorldNormal(int index) const { return transform.transformDirection(normals[(std::size_t)index]); }

        real getArea() const;
        Vector2 getCentroid() const;

        // The moment of inertia of the solid shape with this mass, about its own origin
        real computeInertia(real mass) const;

        real getBoundingRadius() const { return boundingRadius; }

    private:
        std::vector<Vector2> vertices; // Counter-clockwise
        std::vector<Vector2> normals;
        real boundingRadius = 0;
    };

    // A rectangle; it collides as a four-sided polygon
    class BoxCollider : public PolygonCollider
    {
    public:
        explicit BoxCollider(Vector2 halfSize = Vector2((real)0.5, (real)0.5), RigidBody* body = nullptr);

        Vector2 getHalfSize() const { return halfSize; }
        void setHalfSize(Vector2 value);

    private:
        Vector2 halfSize;
    };

    // An infinite line, solid behind it. The defaults give flat ground through the origin, facing up.
    class PlaneCollider : public Collider
    {
    public:
        Vector2 normal; // Unit length, pointing out of the solid side
        real planeOffset; // How far the line is from the origin along its normal

        explicit PlaneCollider(Vector2 normal = Vector2::UP, real planeOffset = 0)
            : Collider(ColliderType::Plane), normal(normal.unit()), planeOffset(planeOffset)
        {
        }
    };

    // A single point where two bodies touch
    class Contact
    {
    public:
        Vector2 point;
        Vector2 normal; // Pointing from body[1] toward body[0]
        real penetration = 0;
        RigidBody* body[2] = { nullptr, nullptr }; // Either may be null for static scenery
        real friction = 0;
        real restitution = 0;
    };

    // The contact array the collision tests write into. Point contactArray at enough contacts and call reset() at the
    // start of each frame.
    class CollisionData
    {
    public:
        Contact* contactArray = nullptr;
        Contact* contacts = nullptr; // The next free contact
        int contactCount = 0;
        int contactsLeft = 0;
        real friction = 0; // Written into every contact a test adds
        real restitution = 0;

        void reset(int maxContacts);
        void addContacts(int num);

        // Fills in the next contact and adds it; null when the array is full
        Contact* addContact(RigidBody* a, RigidBody* b, const Vector2& point, const Vector2& normal, real penetration);
    };

    // Contact generation between pairs of shapes. Each test returns how many contacts it added; call each collider's
    // updateInternals() first. Contact normals point from the second shape toward the first.
    class CollisionTests
    {
    public:
        static int CircleCircle(const CircleCollider& a, const CircleCollider& b, CollisionData* data);
        static int CircleHalfspace(const CircleCollider& circle, const PlaneCollider& plane, CollisionData* data);
        static int CirclePolygon(const CircleCollider& circle, const PolygonCollider& polygon, CollisionData* data);

        // One contact for each corner behind the plane
        static int PolygonHalfspace(const PolygonCollider& polygon, const PlaneCollider& plane, CollisionData* data);

        // Separating axis test. Edges lying against each other give two contacts, one at each end of the overlap,
        // which keeps stacks steady.
        static int PolygonPolygon(const PolygonCollider& a, const PolygonCollider& b, CollisionData* data);

        // Checks both types and calls the matching test; plane-plane adds nothing
        static int Generate(const Collider& a, const Collider& b, CollisionData* data);
    };

    // Anything besides collision tests that creates contacts each frame, such as joints. Chapter 17.
    class ContactGenerator
    {
    public:
        virtual ~ContactGenerator() = default;

        // Writes up to limit contacts and returns how many it wrote
        virtual unsigned addContact(Contact* contact, unsigned limit) const = 0;
    };

    // Pins a point on one body to a point on another, or to a fixed point when body[1] is null; the bodies turn
    // freely about it. Add it to World::joints, which holds the points together exactly.
    class Joint : public ContactGenerator
    {
    public:
        RigidBody* body[2] = { nullptr, nullptr };
        Vector2 position[2]; // Each in its own body's space; position[1] is in world space when body[1] is null
        real error = 0; // How far apart the points may drift before the joint pulls

        void set(RigidBody* a, const Vector2& aPosition, RigidBody* b, const Vector2& bPosition, real allowedError = 0);

        // The book's version (chapter 17.2.1): one contact pulling the points together. Added to contactGenerators
        // it lets them slide sideways, so chains wobble and spin.
        unsigned addContact(Contact* contact, unsigned limit) const override;

        Vector2 getWorldPosition(int index) const;

        // Gives both points the same velocity, as one impulse in any direction
        void solveVelocity() const;

        // Moves and turns the bodies so the points meet; returns how far apart they were
        real solvePosition() const;

        // A sleeping body on an awake one is woken; returns false when nothing on the joint can move
        bool prepare() const;
    };
}
