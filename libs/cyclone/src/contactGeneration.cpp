#include "collision.h"

// Contact generation for planes and boxes, chapter 13

using namespace cyclone;

namespace
{
    real& component(Vector3& v, int index)
    {
        return index == 0 ? v.x : (index == 1 ? v.y : v.z);
    }

    real component(const Vector3& v, int index)
    {
        return index == 0 ? v.x : (index == 1 ? v.y : v.z);
    }

    real clampReal(real value, real min, real max)
    {
        return value < min ? min : (value > max ? max : value);
    }

    // Half the box's extent along an axis: its own half sizes projected onto it
    real transformToAxis(const BoxCollider& box, const Vector3& axis)
    {
        return box.halfSize.x * real_abs(axis * box.getAxis(0)) + box.halfSize.y * real_abs(axis * box.getAxis(1)) +
               box.halfSize.z * real_abs(axis * box.getAxis(2));
    }

    real penetrationOnAxis(const BoxCollider& one, const BoxCollider& two, const Vector3& axis, const Vector3& toCentre)
    {
        return transformToAxis(one, axis) + transformToAxis(two, axis) - real_abs(toCentre * axis);
    }

    // Keeps the axis with the smallest overlap; false as soon as one separates the boxes
    bool tryAxis(const BoxCollider& one, const BoxCollider& two, Vector3 axis, const Vector3& toCentre, unsigned index,
                 real& smallestPenetration, unsigned& smallestCase, real preference = 1)
    {
        // Nearly parallel edges give a near-zero cross product, which can't separate anything
        if (axis.squareMagnitude() < (real)0.0001)
        {
            return true;
        }
        axis.normalise();
        const real penetration = penetrationOnAxis(one, two, axis, toCentre);
        if (penetration < 0)
        {
            return false;
        }
        if (penetration < smallestPenetration * preference)
        {
            smallestPenetration = penetration;
            smallestCase = index;
        }
        return true;
    }

    // The point between the two closest points of two edges, each given by a point, a direction and a half length
    Vector3 edgeContactPoint(const Vector3& pOne, const Vector3& dOne, real oneSize, const Vector3& pTwo, const Vector3& dTwo, real twoSize,
                             bool useOne)
    {
        const real smOne = dOne.squareMagnitude();
        const real smTwo = dTwo.squareMagnitude();
        const real dpOneTwo = dTwo * dOne;
        const Vector3 toSt = pOne - pTwo;
        const real dpStaOne = dOne * toSt;
        const real dpStaTwo = dTwo * toSt;
        const real denom = smOne * smTwo - dpOneTwo * dpOneTwo;

        // Parallel edges have no single closest point
        if (real_abs(denom) < (real)0.0001)
        {
            return useOne ? pOne : pTwo;
        }
        const real mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
        const real mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

        // Off the end of either edge means it's really a vertex-face contact
        if (mua > oneSize || mua < -oneSize || mub > twoSize || mub < -twoSize)
        {
            return useOne ? pOne : pTwo;
        }
        const Vector3 cOne = pOne + dOne * mua;
        const Vector3 cTwo = pTwo + dTwo * mub;
        return cOne * (real)0.5 + cTwo * (real)0.5;
    }

    // Box two's facing face, cut down to the outline of box one's face on the given axis: a contact at every corner
    // of the overlap, including where the two outlines cross, so an offset box rests flat instead of on a diagonal
    int fillPointFace(const BoxCollider& one, const BoxCollider& two, const Vector3& toCentre, CollisionData* data, unsigned best,
                      real penetration)
    {
        // The normal points from two toward one
        Vector3 normal = one.getAxis((int)best);
        if (normal * toCentre > 0)
        {
            normal = normal * -1;
        }
        const real slack = (real)0.01;
        const Vector3 oneCentre = one.getAxis(3);
        const Vector3 oneOutward = normal * -1;
        const real oneExtent = component(one.halfSize, (int)best);

        // Two's face that points most toward one
        int twoAxis = 0;
        real bestDot = 0;
        for (int axis = 0; axis < 3; axis++)
        {
            const real d = real_abs(two.getAxis(axis) * normal);
            if (d > bestDot)
            {
                bestDot = d;
                twoAxis = axis;
            }
        }
        const Vector3 twoOutward = two.getAxis(twoAxis) * (two.getAxis(twoAxis) * normal > 0 ? (real)1 : (real)-1);
        const Vector3 faceCentre = two.getAxis(3) + twoOutward * component(two.halfSize, twoAxis);
        const Vector3 u = two.getAxis((twoAxis + 1) % 3) * component(two.halfSize, (twoAxis + 1) % 3);
        const Vector3 v = two.getAxis((twoAxis + 2) % 3) * component(two.halfSize, (twoAxis + 2) % 3);

        // Each cut can add a corner, so four cuts take the quad to at most eight
        Vector3 polygon[8] = { faceCentre + u + v, faceCentre - u + v, faceCentre - u - v, faceCentre + u - v };
        int corners = 4;
        auto cut = [&](const Vector3& axis, real limit)
        {
            Vector3 kept[8];
            int keptCount = 0;
            for (int i = 0; i < corners && keptCount < 8; i++)
            {
                const Vector3& a = polygon[i];
                const Vector3& b = polygon[(i + 1) % corners];
                const real outA = (a - oneCentre) * axis - limit;
                const real outB = (b - oneCentre) * axis - limit;
                if (outA <= 0)
                {
                    kept[keptCount++] = a;
                }
                if (((outA < 0 && outB > 0) || (outA > 0 && outB < 0)) && keptCount < 8)
                {
                    kept[keptCount++] = a + (b - a) * (outA / (outA - outB));
                }
            }
            for (int i = 0; i < keptCount; i++)
            {
                polygon[i] = kept[i];
            }
            corners = keptCount;
        };
        // A little past the outline still counts, or an edge lying along it gets cut at some arbitrary point
        for (int side = 1; side <= 2; side++)
        {
            const int axis = ((int)best + side) % 3;
            const real limit = component(one.halfSize, axis) + slack;
            cut(one.getAxis(axis), limit);
            cut(one.getAxis(axis) * -1, limit);
        }

        Contact* first = data->contacts;
        int added = 0;
        for (int i = 0; i < corners; i++)
        {
            // Points a little apart are kept with their gap as a negative depth, so a corner lifting while the box
            // rocks still holds it instead of leaving the box balanced on what remains
            const real depth = oneExtent - (polygon[i] - oneCentre) * oneOutward;
            if (depth < -slack)
            {
                continue;
            }
            const Vector3 point = polygon[i] - normal * (depth * (real)0.5);
            bool duplicate = false;
            for (int j = 0; j < added; j++)
            {
                duplicate = duplicate || (first[j].point - point).squareMagnitude() < slack * slack * 4;
            }
            if (duplicate)
            {
                continue;
            }
            if (data->addContact(one.pBody, two.pBody, point, normal, depth) == nullptr)
            {
                break;
            }
            added++;
        }
        if (added > 0)
        {
            return added;
        }

        // No corner inside the outline (a small box resting on a big face's edge): fall back to the deepest corner
        Vector3 vertex = two.halfSize;
        for (int axis = 0; axis < 3; axis++)
        {
            if (two.getAxis(axis) * normal < 0)
            {
                component(vertex, axis) = -component(vertex, axis);
            }
        }
        return data->addContact(one.pBody, two.pBody, two.transform * vertex, normal, penetration) != nullptr ? 1 : 0;
    }
}

int CollisionTests::SphereTruePlane(const SphereCollider& sphere, const PlaneCollider& plane, CollisionData* data)
{
    const Vector3 position = sphere.getAxis(3);
    const real centreDistance = plane.normal * position - plane.offset;
    if (centreDistance * centreDistance > sphere.radius * sphere.radius)
    {
        return 0;
    }

    // Pushed out on whichever side the center is
    Vector3 normal = plane.normal;
    real penetration = -centreDistance;
    if (centreDistance < 0)
    {
        normal = normal * -1;
        penetration = -penetration;
    }
    penetration += sphere.radius;
    const Vector3 point = position - plane.normal * centreDistance;
    return data->addContact(sphere.pBody, plane.pBody, point, normal, penetration) != nullptr ? 1 : 0;
}

int CollisionTests::BoxHalfspace(const BoxCollider& box, const PlaneCollider& plane, CollisionData* data)
{
    if (data->contactsLeft <= 0 || !IntersectionTests::BoxPlane(box, plane))
    {
        return 0;
    }

    int added = 0;
    for (int corner = 0; corner < 8; corner++)
    {
        Vector3 local((corner & 1) ? box.halfSize.x : -box.halfSize.x, (corner & 2) ? box.halfSize.y : -box.halfSize.y,
                      (corner & 4) ? box.halfSize.z : -box.halfSize.z);
        const Vector3 vertex = box.transform * local;
        const real distance = vertex * plane.normal;

        // Corners just above the plane count too, with a negative depth, like box faces
        if (distance > plane.offset + (real)0.01)
        {
            continue;
        }
        // Halfway between the corner and the plane
        const real depth = plane.offset - distance;
        const Vector3 point = vertex + plane.normal * (depth * (real)0.5);
        if (data->addContact(box.pBody, plane.pBody, point, plane.normal, depth) == nullptr)
        {
            break;
        }
        added++;
    }
    return added;
}

int CollisionTests::BoxSphere(const BoxCollider& box, const SphereCollider& sphere, CollisionData* data)
{
    if (data->contactsLeft <= 0)
    {
        return 0;
    }
    const Vector3 centre = sphere.getAxis(3);
    const Vector3 relCentre = box.transform.transformInverse(centre);
    const real radius = sphere.radius;
    if (real_abs(relCentre.x) - radius > box.halfSize.x || real_abs(relCentre.y) - radius > box.halfSize.y ||
        real_abs(relCentre.z) - radius > box.halfSize.z)
    {
        return 0;
    }

    Vector3 closest(clampReal(relCentre.x, -box.halfSize.x, box.halfSize.x), clampReal(relCentre.y, -box.halfSize.y, box.halfSize.y),
                    clampReal(relCentre.z, -box.halfSize.z, box.halfSize.z));
    const real distanceSquared = (closest - relCentre).squareMagnitude();
    if (distanceSquared > radius * radius)
    {
        return 0;
    }

    // The normal points from the sphere toward the box
    if (distanceSquared > (real)1e-10)
    {
        const Vector3 closestWorld = box.transform.transform(closest);
        const Vector3 normal = (closestWorld - centre).unit();
        const real penetration = radius - real_sqrt(distanceSquared);
        return data->addContact(box.pBody, sphere.pBody, closestWorld, normal, penetration) != nullptr ? 1 : 0;
    }

    // The centre is inside the box: push the sphere out through the nearest face
    int axis = 0;
    real smallest = REAL_MAX;
    for (int i = 0; i < 3; i++)
    {
        const real gap = component(box.halfSize, i) - real_abs(component(relCentre, i));
        if (gap < smallest)
        {
            smallest = gap;
            axis = i;
        }
    }
    const real side = component(relCentre, axis) < 0 ? (real)-1 : (real)1;
    const Vector3 normal = box.getAxis(axis) * -side;
    return data->addContact(box.pBody, sphere.pBody, centre, normal, smallest + radius) != nullptr ? 1 : 0;
}

int CollisionTests::BoxBox(const BoxCollider& one, const BoxCollider& two, CollisionData* data, SatCache* cache)
{
    if (data->contactsLeft <= 0)
    {
        return 0;
    }
    const Vector3 toCentre = two.getAxis(3) - one.getAxis(3);
    real penetration = REAL_MAX;
    unsigned best = 0xffffff;

    // Axes 0 to 2 are one's faces, 3 to 5 two's, then the 9 edge-edge cross products
    auto axisFor = [&](unsigned index)
    {
        if (index < 3)
        {
            return one.getAxis((int)index);
        }
        if (index < 6)
        {
            return two.getAxis((int)index - 3);
        }
        return one.getAxis((int)(index - 6) / 3) % two.getAxis((int)(index - 6) % 3);
    };

    // Whatever separated them last frame probably still does
    if (cache != nullptr)
    {
        cache->lastTestHit = false;
        if (cache->separatingAxis >= 0)
        {
            real ignoredPenetration = REAL_MAX;
            unsigned ignoredCase = 0;
            if (!tryAxis(one, two, axisFor((unsigned)cache->separatingAxis), toCentre, 0, ignoredPenetration, ignoredCase))
            {
                cache->lastTestHit = true;
                return 0;
            }
        }
    }

    for (unsigned index = 0; index < 15; index++)
    {
        // An edge axis has to be clearly shallower to win: for boxes resting face on face, rounding can make an
        // edge axis look a hair shallower and swap a steady face contact for a single edge contact
        real preference = index < 6 ? (real)1 : (real)0.95;

        // Crossed edges of two faces turned slightly against each other give an axis almost along a face normal.
        // That's really the face contact, so it can still separate the boxes but never wins.
        const Vector3 axis = axisFor(index);
        if (index >= 6 && axis.squareMagnitude() > (real)0.0001)
        {
            const Vector3 unit = axis.unit();
            for (int face = 0; face < 3; face++)
            {
                if (real_abs(unit * one.getAxis(face)) > (real)0.999 || real_abs(unit * two.getAxis(face)) > (real)0.999)
                {
                    preference = 0;
                }
            }
        }
        if (!tryAxis(one, two, axis, toCentre, index, penetration, best, preference))
        {
            if (cache != nullptr)
            {
                cache->separatingAxis = (int)index;
            }
            return 0;
        }
    }
    if (cache != nullptr)
    {
        cache->separatingAxis = -1;
    }
    if (best == 0xffffff)
    {
        return 0;
    }

    if (best < 3)
    {
        return fillPointFace(one, two, toCentre, data, best, penetration);
    }
    if (best < 6)
    {
        return fillPointFace(two, one, toCentre * -1, data, best - 3, penetration);
    }

    // Edge against edge. Which kind of face axis came closest decides the fallback point for parallel edges.
    real faceOnly = REAL_MAX;
    unsigned bestSingleAxis = 0;
    for (unsigned index = 0; index < 6; index++)
    {
        tryAxis(one, two, axisFor(index), toCentre, index, faceOnly, bestSingleAxis);
    }
    best -= 6;
    const int oneAxisIndex = (int)(best / 3);
    const int twoAxisIndex = (int)(best % 3);
    const Vector3 oneAxis = one.getAxis(oneAxisIndex);
    const Vector3 twoAxis = two.getAxis(twoAxisIndex);
    Vector3 axis = (oneAxis % twoAxis).unit();
    if (axis * toCentre > 0)
    {
        axis = axis * -1;
    }

    // A point on each edge: the corner on the side facing the other box, with the edge's own axis zeroed
    Vector3 ptOnOneEdge = one.halfSize;
    Vector3 ptOnTwoEdge = two.halfSize;
    for (int i = 0; i < 3; i++)
    {
        if (i == oneAxisIndex)
        {
            component(ptOnOneEdge, i) = 0;
        }
        else if (one.getAxis(i) * axis > 0)
        {
            component(ptOnOneEdge, i) = -component(ptOnOneEdge, i);
        }

        if (i == twoAxisIndex)
        {
            component(ptOnTwoEdge, i) = 0;
        }
        else if (two.getAxis(i) * axis < 0)
        {
            component(ptOnTwoEdge, i) = -component(ptOnTwoEdge, i);
        }
    }
    ptOnOneEdge = one.transform * ptOnOneEdge;
    ptOnTwoEdge = two.transform * ptOnTwoEdge;

    const Vector3 vertex = edgeContactPoint(ptOnOneEdge, oneAxis, component(one.halfSize, oneAxisIndex), ptOnTwoEdge, twoAxis,
                                            component(two.halfSize, twoAxisIndex), bestSingleAxis > 2);
    return data->addContact(one.pBody, two.pBody, vertex, axis, penetration) != nullptr ? 1 : 0;
}

int CollisionTests::Generate(const Collider& a, const Collider& b, CollisionData* data, SatCache* cache)
{
    const ColliderType ta = a.type;
    const ColliderType tb = b.type;
    if (ta == ColliderType::Convex || tb == ColliderType::Convex)
    {
        const bool aConvex = ta == ColliderType::Convex;
        const ConvexCollider& convex = static_cast<const ConvexCollider&>(aConvex ? a : b);
        const Collider& other = aConvex ? b : a;
        switch (other.type)
        {
        case ColliderType::Plane:
            return ConvexHalfspace(convex, static_cast<const PlaneCollider&>(other), data);
        case ColliderType::Sphere:
            return ConvexSphere(convex, static_cast<const SphereCollider&>(other), data);
        case ColliderType::Box:
            return ConvexBox(convex, static_cast<const BoxCollider&>(other), data, cache);
        case ColliderType::Convex:
            return ConvexConvex(static_cast<const ConvexCollider&>(a), static_cast<const ConvexCollider&>(b), data, cache);
        }
        return 0;
    }
    if (ta == ColliderType::Sphere && tb == ColliderType::Sphere)
    {
        return SphereSphere(static_cast<const SphereCollider&>(a), static_cast<const SphereCollider&>(b), data);
    }
    if (ta == ColliderType::Sphere && tb == ColliderType::Plane)
    {
        return SphereHalfspace(static_cast<const SphereCollider&>(a), static_cast<const PlaneCollider&>(b), data);
    }
    if (ta == ColliderType::Plane && tb == ColliderType::Sphere)
    {
        return SphereHalfspace(static_cast<const SphereCollider&>(b), static_cast<const PlaneCollider&>(a), data);
    }
    if (ta == ColliderType::Box && tb == ColliderType::Plane)
    {
        return BoxHalfspace(static_cast<const BoxCollider&>(a), static_cast<const PlaneCollider&>(b), data);
    }
    if (ta == ColliderType::Plane && tb == ColliderType::Box)
    {
        return BoxHalfspace(static_cast<const BoxCollider&>(b), static_cast<const PlaneCollider&>(a), data);
    }
    if (ta == ColliderType::Box && tb == ColliderType::Sphere)
    {
        return BoxSphere(static_cast<const BoxCollider&>(a), static_cast<const SphereCollider&>(b), data);
    }
    if (ta == ColliderType::Sphere && tb == ColliderType::Box)
    {
        return BoxSphere(static_cast<const BoxCollider&>(b), static_cast<const SphereCollider&>(a), data);
    }
    if (ta == ColliderType::Box && tb == ColliderType::Box)
    {
        return BoxBox(static_cast<const BoxCollider&>(a), static_cast<const BoxCollider&>(b), data, cache);
    }
    return 0;
}
