#include "collision.h"

#include <cmath>

using namespace cyclone;

// How far a box reaches along an axis, shared by box-plane and box-box
static real sizeAlongAxis(const BoxCollider& box, Vector3 axis)
{
    const Vector3& halfSize = box.halfSize;
    return halfSize.x * real_abs(axis * box.getAxis(0)) + halfSize.y * real_abs(axis * box.getAxis(1)) +
           halfSize.z * real_abs(axis * box.getAxis(2));
}

// One separating axis test: do the two boxes overlap along this axis?
static bool overlapOnAxis(const BoxCollider& a, const BoxCollider& b, Vector3 axis, Vector3 toCenter)
{
    // A cross product of two (nearly) parallel edges gives a near zero axis,
    // which can't separate anything, so skip it.
    if (axis.squareMagnitude() < 0.01)
    {
        return true;
    }

    real aRadius = sizeAlongAxis(a, axis);
    real bRadius = sizeAlongAxis(b, axis);
    real distance = real_abs(toCenter * axis);
    return distance <= aRadius + bRadius;
}

static real clamp(real value, real min, real max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

void Collider::updateInternals()
{
    // A collider without a rigidbody is static, so its offset is its world transform.
    if (pBody)
    {
        transform = pBody->getTransformationMatrix() * offset;
    }
    else
    {
        transform = offset;
    }
}

bool IntersectionTests::SphereSphere(const SphereCollider& a, const SphereCollider& b)
{
    // Only comparing distances, so squared lengths are enough
    real distanceSquared = (a.getAxis(3) - b.getAxis(3)).squareMagnitude();
    real radiiSum = a.radius + b.radius;
    return distanceSquared < radiiSum * radiiSum;
}

bool IntersectionTests::SpherePlane(const SphereCollider& sphere, const PlaneCollider& plane)
{
    // Signed distance from the plane to the sphere's center. The plane is
    // treated as a half-space, so anything behind it counts as overlapping.
    real distance = plane.normal * sphere.getAxis(3) - plane.offset;
    return distance < sphere.radius;
}

bool IntersectionTests::SphereBox(const SphereCollider& sphere, const BoxCollider& box)
{
    // Transform the sphere's center into the box's local space, where the
    // box is an AABB centered on the origin.
    Vector3 p = box.transform.transformInverse(sphere.getAxis(3));

    // Component-wise clamp gives the nearest point on the box to the center.
    Vector3 nearest(clamp(p.x, -box.halfSize.x, box.halfSize.x), clamp(p.y, -box.halfSize.y, box.halfSize.y),
                    clamp(p.z, -box.halfSize.z, box.halfSize.z));

    real distanceSquared = (nearest - p).squareMagnitude();
    return distanceSquared < sphere.radius * sphere.radius;
}

bool IntersectionTests::BoxBox(const BoxCollider& a, const BoxCollider& b)
{
    const Vector3 toCenter = a.getAxis(3) - b.getAxis(3);

#define TEST_OVERLAP(axis) overlapOnAxis(a, b, (axis), toCenter)
    bool result =
        // a's face normals
        TEST_OVERLAP(a.getAxis(0)) && TEST_OVERLAP(a.getAxis(1)) && TEST_OVERLAP(a.getAxis(2)) &&
        // b's face normals
        TEST_OVERLAP(b.getAxis(0)) && TEST_OVERLAP(b.getAxis(1)) && TEST_OVERLAP(b.getAxis(2)) &&
        // Cross products between edges. A box has 3 unique edge directions,
        // which are the same as its face normals, so 9 tests.
        TEST_OVERLAP(a.getAxis(0) % b.getAxis(0)) && TEST_OVERLAP(a.getAxis(0) % b.getAxis(1)) &&
        TEST_OVERLAP(a.getAxis(0) % b.getAxis(2)) && TEST_OVERLAP(a.getAxis(1) % b.getAxis(0)) &&
        TEST_OVERLAP(a.getAxis(1) % b.getAxis(1)) && TEST_OVERLAP(a.getAxis(1) % b.getAxis(2)) &&
        TEST_OVERLAP(a.getAxis(2) % b.getAxis(0)) && TEST_OVERLAP(a.getAxis(2) % b.getAxis(1)) && TEST_OVERLAP(a.getAxis(2) % b.getAxis(2));
#undef TEST_OVERLAP

    return result;
}

bool IntersectionTests::BoxPlane(const BoxCollider& box, const PlaneCollider& plane)
{
    // The box's size along the plane normal acts like a sphere's radius,
    // so the rest is the same as sphere-plane.
    real projectedRadius = sizeAlongAxis(box, plane.normal);
    real distance = plane.normal * box.getAxis(3) - plane.offset;
    return distance < projectedRadius;
}

bool IntersectionTests::Overlap(const Collider& a, const Collider& b)
{
    // Convex shapes have no separate yes/no test: ask the contact test whether it finds anything
    if (a.type == ColliderType::Convex || b.type == ColliderType::Convex)
    {
        Contact scratch[16];
        CollisionData data;
        data.contactArray = scratch;
        data.reset(16);
        return CollisionTests::Generate(a, b, &data) > 0;
    }

    switch (a.type)
    {
    case ColliderType::Sphere:
    {
        const SphereCollider& sphere = static_cast<const SphereCollider&>(a);

        switch (b.type)
        {
        case ColliderType::Sphere:
            return SphereSphere(sphere, static_cast<const SphereCollider&>(b));
        case ColliderType::Plane:
            return SpherePlane(sphere, static_cast<const PlaneCollider&>(b));
        case ColliderType::Box:
            return SphereBox(sphere, static_cast<const BoxCollider&>(b));
        case ColliderType::Convex:
            break;
        }
        break;
    }
    case ColliderType::Plane:
    {
        const PlaneCollider& plane = static_cast<const PlaneCollider&>(a);

        switch (b.type)
        {
        case ColliderType::Sphere:
            return SpherePlane(static_cast<const SphereCollider&>(b), plane);
        case ColliderType::Plane:
            return false;
        case ColliderType::Box:
            return BoxPlane(static_cast<const BoxCollider&>(b), plane);
        case ColliderType::Convex:
            break;
        }
        break;
    }
    case ColliderType::Box:
    {
        const BoxCollider& box = static_cast<const BoxCollider&>(a);

        switch (b.type)
        {
        case ColliderType::Sphere:
            return SphereBox(static_cast<const SphereCollider&>(b), box);
        case ColliderType::Plane:
            return BoxPlane(box, static_cast<const PlaneCollider&>(b));
        case ColliderType::Box:
            return BoxBox(box, static_cast<const BoxCollider&>(b));
        case ColliderType::Convex:
            break;
        }
        break;
    }
    case ColliderType::Convex:
        break;
    }
    return false;
}

void IntersectionTests::UpdateOverlaps(const std::vector<Collider*>& colliders)
{
    for (Collider* collider : colliders)
    {
        collider->updateInternals();
        collider->overlapCount = 0;
    }

    // Start j at i + 1 so each pair is only tested once, and nothing is
    // tested against itself.
    for (size_t i = 0; i < colliders.size(); i++)
    {
        for (size_t j = i + 1; j < colliders.size(); j++)
        {
            if (Overlap(*colliders[i], *colliders[j]))
            {
                colliders[i]->overlapCount++;
                colliders[j]->overlapCount++;
            }
        }
    }
}

void CollisionData::reset(int maxContacts)
{
    contacts = contactArray;
    contactCount = 0;
    contactsLeft = maxContacts;
}

void CollisionData::addContacts(int num)
{
    contacts += num;
    contactCount += num;
    contactsLeft -= num;
}

Contact* CollisionData::addContact(RigidBody* a, RigidBody* b, const Vector3& point, const Vector3& normal, real penetration)
{
    if (contactsLeft <= 0)
    {
        return nullptr;
    }
    Contact* contact = contacts;
    contact->body[0] = a;
    contact->body[1] = b;
    contact->point = point;
    contact->normal = normal;
    contact->penetration = penetration;
    contact->friction = friction;
    contact->restitution = restitution;
    addContacts(1);
    return contact;
}

int CollisionTests::SphereSphere(const SphereCollider& a, const SphereCollider& b, CollisionData* data)
{
    return SphereSphere(a.getAxis(3), a.radius, a.pBody, b.getAxis(3), b.radius, b.pBody, data);
}

int CollisionTests::SphereSphere(const Vector3& positionA, real radiusA, RigidBody* bodyA, const Vector3& positionB, real radiusB,
                                 RigidBody* bodyB, CollisionData* data)
{
    if (data->contactsLeft <= 0)
    {
        return 0;
    }

    // 1D Axis Check. If they don't overlap on the axis, impossible to collide
    const real radiusSum = radiusA + radiusB;
    if (std::abs(positionA.x - positionB.x) >= radiusSum)
    {
        return 0;
    }
    if (std::abs(positionA.y - positionB.y) >= radiusSum)
    {
        return 0;
    }
    if (std::abs(positionA.z - positionB.z) >= radiusSum)
    {
        return 0;
    }

    const Vector3 midline = positionA - positionB;
    const real distanceSq = (midline.x * midline.x) + (midline.y * midline.y) + (midline.z * midline.z);
    const real radiusSumSq = radiusSum * radiusSum;

    if (distanceSq >= radiusSumSq || distanceSq <= 0)
    {
        return 0; // Missed the corner, or perfectly identical centers
    }

    // Collision confirmed, calculate true distance
    const real distance = std::sqrt(distanceSq);

    Contact* contact = data->contacts;
    contact->normal = midline * (((real)1.0) / distance); // Points from b to a
    contact->point = positionB + midline * (real)0.5; // Halfway between the centers
    contact->penetration = radiusSum - distance;
    contact->body[0] = bodyA;
    contact->body[1] = bodyB;
    contact->friction = data->friction;
    contact->restitution = data->restitution;
    data->addContacts(1);
    return 1;
}

int CollisionTests::SphereHalfspace(const SphereCollider& sphere, const PlaneCollider& plane, CollisionData* data)
{
    return SphereHalfspace(sphere.getAxis(3), sphere.radius, sphere.pBody, plane, data);
}

int CollisionTests::SphereHalfspace(const Vector3& position, real radius, RigidBody* body, const PlaneCollider& plane, CollisionData* data)
{
    if (data->contactsLeft <= 0)
    {
        return 0;
    }

    // Signed distance from the sphere's surface to the plane. Positive means
    // the sphere is fully in front of the plane.
    const real distance = plane.normal * position - plane.offset - radius;
    if (distance >= 0)
    {
        return 0;
    }

    Contact* contact = data->contacts;
    contact->normal = plane.normal; // A half-space always pushes along its normal
    // Project the sphere's center onto the plane
    contact->point = position - plane.normal * (distance + radius);
    contact->penetration = -distance;
    contact->body[0] = body;
    contact->body[1] = plane.pBody;
    contact->friction = data->friction;
    contact->restitution = data->restitution;
    data->addContacts(1);
    return 1;
}

// A missing body is static geometry, which acts as if it has infinite mass
static real inverseMassOf(const RigidBody* body)
{
    return body ? body->getInverseMass() : 0;
}

// How far body[0] is from body[1] along the contact normal. A missing body is
// static, so it never moves and can count as being at the origin.
static real separationAlongNormal(const Contact& contact)
{
    Vector3 offset = contact.body[0] ? contact.body[0]->getPosition() : Vector3(0, 0, 0);
    if (contact.body[1])
    {
        offset -= contact.body[1]->getPosition();
    }
    return offset * contact.normal;
}

void ContactResolver::resolveContacts(Contact* contacts, int contactCount)
{
    // Resolving moves bodies, which changes every other contact's depth, so record each contact's separation now
    // and measure later changes against it.
    m_startSeparation.resize(contactCount);
    for (int i = 0; i < contactCount; i++)
    {
        m_startSeparation[i] = separationAlongNormal(contacts[i]);
    }

    for (int pass = 0; pass < iterations; pass++)
    {
        for (int i = 0; i < contactCount; i++)
        {
            resolveContact(contacts, i);
        }
    }
}

void ContactResolver::resolveContacts(Contact* contacts, const std::vector<int>& batchBounds, const std::vector<int>& phaseBounds,
                                      ThreadPool* threadPool)
{
    const int batchCount = (int)batchBounds.size() - 1;
    if (batchCount <= 0)
    {
        return;
    }

    // Runs job(b) for batches first up to (not including) last, on the
    // threads if there are any
    auto runBatches = [&](int first, int last, const std::function<void(int)>& job)
    {
        if (threadPool)
        {
            threadPool->run(last - first, [&](int k) { job(first + k); });
        }
        else
        {
            for (int b = first; b < last; b++)
            {
                job(b);
            }
        }
    };

    // As in the single list version, record every separation before anything moves. It only reads, so every
    // batch can do it at once.
    m_startSeparation.resize(batchBounds[batchCount]);
    runBatches(0, batchCount, [&](int b)
    {
        for (int i = batchBounds[b]; i < batchBounds[b + 1]; i++)
        {
            m_startSeparation[i] = separationAlongNormal(contacts[i]);
        }
    });

    for (int pass = 0; pass < iterations; pass++)
    {
        for (size_t p = 0; p + 1 < phaseBounds.size(); p++)
        {
            // No two batches in a phase touch the same body, so they can't
            // see each other's changes, and their order doesn't matter
            runBatches(phaseBounds[p], phaseBounds[p + 1], [&](int b)
            {
                for (int i = batchBounds[b]; i < batchBounds[b + 1]; i++)
                {
                    resolveContact(contacts, i);
                }
            });
        }
    }
}

void ContactResolver::resolveContact(Contact* contacts, int i)
{
    resolveVelocity(contacts[i]);

    Contact current = contacts[i];
    current.penetration -= separationAlongNormal(current) - m_startSeparation[i];
    resolveInterpenetration(current);
}

void ContactResolver::resolveVelocity(const Contact& contact)
{
    RigidBody* a = contact.body[0];
    RigidBody* b = contact.body[1];

    // Separating velocity: the relative velocity of a with respect to b,
    // along the contact normal.
    Vector3 relativeVelocity = a ? a->getLinearVelocity() : Vector3(0, 0, 0);
    if (b)
    {
        relativeVelocity -= b->getLinearVelocity();
    }
    const real separatingVelocity = relativeVelocity * contact.normal;

    // Already moving apart (or at rest), so no impulse is needed
    if (separatingVelocity > 0)
    {
        return;
    }

    const real newSeparatingVelocity = -separatingVelocity * restitution;
    const real deltaVelocity = newSeparatingVelocity - separatingVelocity;

    // If both bodies have infinite mass, neither can be moved
    const real totalInverseMass = inverseMassOf(a) + inverseMassOf(b);
    if (totalInverseMass <= 0)
    {
        return;
    }

    // Total impulse j, applied along the normal. Each body's share of the
    // change in velocity scales with its inverse mass.
    const real impulse = deltaVelocity / totalInverseMass;
    const Vector3 impulsePerInverseMass = contact.normal * impulse;

    if (a)
    {
        a->setLinearVelocity(a->getLinearVelocity() + impulsePerInverseMass * a->getInverseMass());
    }
    // b moves backwards along the normal, since the normal points from b to a
    if (b)
    {
        b->setLinearVelocity(b->getLinearVelocity() - impulsePerInverseMass * b->getInverseMass());
    }
}

void ContactResolver::resolveInterpenetration(const Contact& contact)
{
    if (contact.penetration <= 0)
    {
        return;
    }

    RigidBody* a = contact.body[0];
    RigidBody* b = contact.body[1];

    const real totalInverseMass = inverseMassOf(a) + inverseMassOf(b);
    if (totalInverseMass <= 0)
    {
        return;
    }

    // Each body moves a share of the penetration proportional to its inverse
    // mass, so the two displacements add up to the full penetration depth.
    const Vector3 movePerInverseMass = contact.normal * (contact.penetration / totalInverseMass);

    if (a)
    {
        a->setPosition(a->getPosition() + movePerInverseMass * a->getInverseMass());
    }
    if (b)
    {
        b->setPosition(b->getPosition() - movePerInverseMass * b->getInverseMass());
    }
}
