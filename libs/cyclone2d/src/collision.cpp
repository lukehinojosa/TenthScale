#include "collision.h"

#include <algorithm>
#include <utility>

using namespace cyclone2d;

namespace
{
    // Shapes closer than this count as touching, and edges this close to parallel count as lying flat
    constexpr real kLinearSlop = (real)0.005;

    // Keeps the part of the segment in front of the line through the origin offset along normal by offset
    int clipSegmentToLine(Vector2 out[2], const Vector2 in[2], const Vector2& normal, real offset)
    {
        int count = 0;
        const real distance0 = normal.dot(in[0]) - offset;
        const real distance1 = normal.dot(in[1]) - offset;
        if (distance0 <= 0)
        {
            out[count++] = in[0];
        }
        if (distance1 <= 0)
        {
            out[count++] = in[1];
        }
        if (distance0 * distance1 < 0)
        {
            const real t = distance0 / (distance0 - distance1);
            out[count] = in[0] + (in[1] - in[0]) * t;
            count++;
        }
        return count;
    }

    // The face of a whose normal leaves b furthest outside it, and how far; positive means they are apart
    real findMaxSeparation(const PolygonCollider& a, const PolygonCollider& b, int* edgeIndex)
    {
        real best = -REAL_MAX;
        int bestIndex = 0;
        const int countA = a.getVertexCount();
        const int countB = b.getVertexCount();
        for (int i = 0; i < countA; i++)
        {
            const Vector2 normal = a.getWorldNormal(i);
            const Vector2 vertex = a.getWorldVertex(i);
            real deepest = REAL_MAX;
            for (int j = 0; j < countB; j++)
            {
                deepest = std::min(deepest, normal.dot(b.getWorldVertex(j) - vertex));
            }
            if (deepest > best)
            {
                best = deepest;
                bestIndex = i;
            }
        }
        *edgeIndex = bestIndex;
        return best;
    }

    // The edge of incident facing most directly against the reference normal
    int findIncidentEdge(const PolygonCollider& incident, const Vector2& referenceNormal)
    {
        int index = 0;
        real lowest = REAL_MAX;
        for (int i = 0; i < incident.getVertexCount(); i++)
        {
            const real d = referenceNormal.dot(incident.getWorldNormal(i));
            if (d < lowest)
            {
                lowest = d;
                index = i;
            }
        }
        return index;
    }
}

void Collider::updateInternals()
{
    transform = pBody != nullptr ? pBody->getTransform() * offset : offset;
}

real Collider::getBoundingRadius() const
{
    switch (type)
    {
    case ColliderType::Circle:
        return static_cast<const CircleCollider*>(this)->radius;
    case ColliderType::Polygon:
        return static_cast<const PolygonCollider*>(this)->getBoundingRadius();
    case ColliderType::Plane:
        return -1;
    }
    return -1;
}

bool Collider::containsPoint(const Vector2& worldPoint) const
{
    switch (type)
    {
    case ColliderType::Circle:
    {
        const real radius = static_cast<const CircleCollider*>(this)->radius;
        return (worldPoint - transform.position).squareMagnitude() <= radius * radius;
    }
    case ColliderType::Polygon:
    {
        const PolygonCollider& polygon = *static_cast<const PolygonCollider*>(this);
        const Vector2 local = transform.transformInverse(worldPoint);
        for (int i = 0; i < polygon.getVertexCount(); i++)
        {
            if (polygon.getNormals()[(std::size_t)i].dot(local - polygon.getVertices()[(std::size_t)i]) > 0)
            {
                return false;
            }
        }
        return polygon.getVertexCount() > 0;
    }
    case ColliderType::Plane:
    {
        const PlaneCollider& plane = *static_cast<const PlaneCollider*>(this);
        return plane.normal.dot(worldPoint) <= plane.planeOffset;
    }
    }
    return false;
}

PolygonCollider::PolygonCollider(const std::vector<Vector2>& points, RigidBody* body) : Collider(ColliderType::Polygon, body)
{
    setVertices(points);
}

void PolygonCollider::setVertices(const std::vector<Vector2>& points)
{
    // Andrew's monotone chain: sort, then build the lower and upper hulls, which come out counter-clockwise
    std::vector<Vector2> sorted = points;
    std::sort(sorted.begin(), sorted.end(), [](const Vector2& a, const Vector2& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());

    vertices.clear();
    if (sorted.size() >= 3)
    {
        std::vector<Vector2> hull(sorted.size() * 2);
        std::size_t k = 0;
        auto turn = [](const Vector2& o, const Vector2& a, const Vector2& b) { return (a - o).cross(b - o); };
        for (const Vector2& p : sorted)
        {
            while (k >= 2 && turn(hull[k - 2], hull[k - 1], p) <= 0)
            {
                k--;
            }
            hull[k++] = p;
        }
        const std::size_t lowerSize = k + 1;
        for (std::size_t i = sorted.size() - 1; i > 0; i--)
        {
            const Vector2& p = sorted[i - 1];
            while (k >= lowerSize && turn(hull[k - 2], hull[k - 1], p) <= 0)
            {
                k--;
            }
            hull[k++] = p;
        }
        hull.resize(k - 1);
        vertices = hull;
    }

    normals.clear();
    boundingRadius = 0;
    const std::size_t count = vertices.size();
    for (std::size_t i = 0; i < count; i++)
    {
        const Vector2 edge = vertices[(i + 1) % count] - vertices[i];
        normals.push_back(Vector2(edge.y, -edge.x).unit());
        boundingRadius = std::max(boundingRadius, vertices[i].magnitude());
    }
}

PolygonCollider PolygonCollider::regular(int sides, real radius, RigidBody* body)
{
    std::vector<Vector2> points;
    for (int i = 0; i < sides; i++)
    {
        const real angle = (real)(2 * R_PI) * (real)i / (real)sides;
        points.push_back(Vector2(real_cos(angle) * radius, real_sin(angle) * radius));
    }
    return PolygonCollider(points, body);
}

real PolygonCollider::getArea() const
{
    real twiceArea = 0;
    const std::size_t count = vertices.size();
    for (std::size_t i = 0; i < count; i++)
    {
        twiceArea += vertices[i].cross(vertices[(i + 1) % count]);
    }
    return twiceArea / 2;
}

Vector2 PolygonCollider::getCentroid() const
{
    // Each edge and the origin make a triangle; the centroid is their area-weighted average
    Vector2 sum;
    real twiceArea = 0;
    const std::size_t count = vertices.size();
    for (std::size_t i = 0; i < count; i++)
    {
        const Vector2& a = vertices[i];
        const Vector2& b = vertices[(i + 1) % count];
        const real c = a.cross(b);
        twiceArea += c;
        sum += (a + b) * c;
    }
    if (twiceArea == 0)
    {
        return Vector2();
    }
    return sum * (1 / (3 * twiceArea));
}

real PolygonCollider::computeInertia(real mass) const
{
    // Sums each origin-edge triangle's second moment of area, then scales from area to mass
    real numerator = 0;
    real twiceArea = 0;
    const std::size_t count = vertices.size();
    for (std::size_t i = 0; i < count; i++)
    {
        const Vector2& a = vertices[i];
        const Vector2& b = vertices[(i + 1) % count];
        const real c = a.cross(b);
        twiceArea += c;
        numerator += c * (a.dot(a) + a.dot(b) + b.dot(b));
    }
    if (twiceArea == 0)
    {
        return 0;
    }
    return mass * numerator / (6 * twiceArea);
}

BoxCollider::BoxCollider(Vector2 halfSize, RigidBody* body) : PolygonCollider(body)
{
    setHalfSize(halfSize);
}

void BoxCollider::setHalfSize(Vector2 value)
{
    halfSize = value;
    setVertices({ Vector2(-value.x, -value.y), Vector2(value.x, -value.y), Vector2(value.x, value.y), Vector2(-value.x, value.y) });
}

void CollisionData::reset(int maxContacts)
{
    contactsLeft = maxContacts;
    contactCount = 0;
    contacts = contactArray;
}

void CollisionData::addContacts(int num)
{
    contactsLeft -= num;
    contactCount += num;
    contacts += num;
}

Contact* CollisionData::addContact(RigidBody* a, RigidBody* b, const Vector2& point, const Vector2& normal, real penetration)
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

int CollisionTests::CircleCircle(const CircleCollider& a, const CircleCollider& b, CollisionData* data)
{
    const Vector2 delta = a.getPosition() - b.getPosition();
    const real distance = delta.magnitude();
    const real reach = a.radius + b.radius;
    if (distance >= reach)
    {
        return 0;
    }

    // Exactly on top of each other: any direction works, so push straight up
    const Vector2 normal = distance > 0 ? delta * (1 / distance) : Vector2::UP;
    const real penetration = reach - distance;
    const Vector2 point = b.getPosition() + normal * (b.radius - penetration / 2);
    return data->addContact(a.pBody, b.pBody, point, normal, penetration) != nullptr ? 1 : 0;
}

int CollisionTests::CircleHalfspace(const CircleCollider& circle, const PlaneCollider& plane, CollisionData* data)
{
    const real distance = plane.normal.dot(circle.getPosition()) - plane.planeOffset - circle.radius;
    if (distance >= 0)
    {
        return 0;
    }
    const Vector2 point = circle.getPosition() - plane.normal * (circle.radius + distance / 2);
    return data->addContact(circle.pBody, nullptr, point, plane.normal, -distance) != nullptr ? 1 : 0;
}

int CollisionTests::CirclePolygon(const CircleCollider& circle, const PolygonCollider& polygon, CollisionData* data)
{
    const int count = polygon.getVertexCount();
    if (count < 3)
    {
        return 0;
    }

    // Worked out in the polygon's space, then turned back to the world
    const Vector2 centre = polygon.transform.transformInverse(circle.getPosition());
    const std::vector<Vector2>& vertices = polygon.getVertices();
    const std::vector<Vector2>& normals = polygon.getNormals();

    real separation = -REAL_MAX;
    int face = 0;
    for (int i = 0; i < count; i++)
    {
        const real s = normals[(std::size_t)i].dot(centre - vertices[(std::size_t)i]);
        if (s > circle.radius)
        {
            return 0;
        }
        if (s > separation)
        {
            separation = s;
            face = i;
        }
    }

    const Vector2& v1 = vertices[(std::size_t)face];
    const Vector2& v2 = vertices[(std::size_t)((face + 1) % count)];
    Vector2 normal = normals[(std::size_t)face];
    Vector2 surfacePoint = centre - normal * separation;
    real penetration = circle.radius - separation;

    // Outside the polygon, past the end of the face: the nearest point is a corner
    if (separation > 0)
    {
        const Vector2* corner = nullptr;
        if ((centre - v1).dot(v2 - v1) <= 0)
        {
            corner = &v1;
        }
        else if ((centre - v2).dot(v1 - v2) <= 0)
        {
            corner = &v2;
        }
        if (corner != nullptr)
        {
            const Vector2 toCentre = centre - *corner;
            const real distance = toCentre.magnitude();
            if (distance >= circle.radius || distance <= 0)
            {
                return 0;
            }
            normal = toCentre * (1 / distance);
            surfacePoint = *corner;
            penetration = circle.radius - distance;
        }
    }

    const Vector2 worldNormal = polygon.transform.transformDirection(normal);
    const Vector2 point = polygon.transform.transform(surfacePoint) + worldNormal * (-penetration / 2);
    return data->addContact(circle.pBody, polygon.pBody, point, worldNormal, penetration) != nullptr ? 1 : 0;
}

int CollisionTests::PolygonHalfspace(const PolygonCollider& polygon, const PlaneCollider& plane, CollisionData* data)
{
    int added = 0;
    for (int i = 0; i < polygon.getVertexCount(); i++)
    {
        const Vector2 vertex = polygon.getWorldVertex(i);
        const real distance = plane.normal.dot(vertex) - plane.planeOffset;
        if (distance >= 0)
        {
            continue;
        }
        const Vector2 point = vertex - plane.normal * (distance / 2);
        if (data->addContact(polygon.pBody, nullptr, point, plane.normal, -distance) == nullptr)
        {
            break;
        }
        added++;
    }
    return added;
}

int CollisionTests::PolygonPolygon(const PolygonCollider& a, const PolygonCollider& b, CollisionData* data)
{
    if (a.getVertexCount() < 3 || b.getVertexCount() < 3)
    {
        return 0;
    }

    int edgeA = 0;
    const real separationA = findMaxSeparation(a, b, &edgeA);
    if (separationA > 0)
    {
        return 0;
    }
    int edgeB = 0;
    const real separationB = findMaxSeparation(b, a, &edgeB);
    if (separationB > 0)
    {
        return 0;
    }

    // The face to clip against, preferring a's so the choice doesn't flicker when the two are nearly equal
    const bool flip = separationB > separationA + kLinearSlop * (real)0.1;
    const PolygonCollider& reference = flip ? b : a;
    const PolygonCollider& incident = flip ? a : b;
    const int referenceEdge = flip ? edgeB : edgeA;

    const int referenceCount = reference.getVertexCount();
    const Vector2 v1 = reference.getWorldVertex(referenceEdge);
    const Vector2 v2 = reference.getWorldVertex((referenceEdge + 1) % referenceCount);
    const Vector2 referenceNormal = reference.getWorldNormal(referenceEdge);
    const Vector2 tangent = (v2 - v1).unit();

    const int incidentEdge = findIncidentEdge(incident, referenceNormal);
    const Vector2 incidentPoints[2] = { incident.getWorldVertex(incidentEdge),
                                        incident.getWorldVertex((incidentEdge + 1) % incident.getVertexCount()) };

    // Trim the incident edge to the width of the reference face
    Vector2 clipped1[2];
    Vector2 clipped2[2];
    if (clipSegmentToLine(clipped1, incidentPoints, -tangent, -tangent.dot(v1)) < 2)
    {
        return 0;
    }
    if (clipSegmentToLine(clipped2, clipped1, tangent, tangent.dot(v2)) < 2)
    {
        return 0;
    }

    // Normals point from the second shape to the first
    const Vector2 normal = flip ? referenceNormal : -referenceNormal;
    const real faceOffset = referenceNormal.dot(v1);
    int added = 0;
    for (const Vector2& vertex : clipped2)
    {
        const real separation = referenceNormal.dot(vertex) - faceOffset;
        if (separation > 0)
        {
            continue;
        }
        const Vector2 point = vertex - referenceNormal * (separation / 2);
        if (data->addContact(a.pBody, b.pBody, point, normal, -separation) == nullptr)
        {
            break;
        }
        added++;
    }
    return added;
}

int CollisionTests::Generate(const Collider& a, const Collider& b, CollisionData* data)
{
    const ColliderType ta = a.type;
    const ColliderType tb = b.type;
    if (ta == ColliderType::Plane && tb == ColliderType::Plane)
    {
        return 0;
    }
    if (ta == ColliderType::Plane || (ta == ColliderType::Polygon && tb == ColliderType::Circle))
    {
        // The tests take the shapes the other way round, so turn their contacts back to keep normals pointing at a
        Contact* first = data->contacts;
        const int added = Generate(b, a, data);
        for (int i = 0; i < added; i++)
        {
            first[i].normal = -first[i].normal;
            std::swap(first[i].body[0], first[i].body[1]);
        }
        return added;
    }

    if (ta == ColliderType::Circle)
    {
        const CircleCollider& circle = static_cast<const CircleCollider&>(a);
        switch (tb)
        {
        case ColliderType::Circle:
            return CircleCircle(circle, static_cast<const CircleCollider&>(b), data);
        case ColliderType::Polygon:
            return CirclePolygon(circle, static_cast<const PolygonCollider&>(b), data);
        case ColliderType::Plane:
            return CircleHalfspace(circle, static_cast<const PlaneCollider&>(b), data);
        }
        return 0;
    }

    const PolygonCollider& polygon = static_cast<const PolygonCollider&>(a);
    if (tb == ColliderType::Plane)
    {
        return PolygonHalfspace(polygon, static_cast<const PlaneCollider&>(b), data);
    }
    return PolygonPolygon(polygon, static_cast<const PolygonCollider&>(b), data);
}

void Joint::set(RigidBody* a, const Vector2& aPosition, RigidBody* b, const Vector2& bPosition, real allowedError)
{
    body[0] = a;
    body[1] = b;
    position[0] = aPosition;
    position[1] = bPosition;
    error = allowedError;
}

Vector2 Joint::getWorldPosition(int index) const
{
    if (body[index] == nullptr)
    {
        return position[index];
    }
    return body[index]->getPointInWorldSpace(position[index]);
}

unsigned Joint::addContact(Contact* contact, unsigned limit) const
{
    if (limit == 0 || body[0] == nullptr)
    {
        return 0;
    }
    const Vector2 a = getWorldPosition(0);
    const Vector2 b = getWorldPosition(1);
    const Vector2 aToB = b - a;
    const real length = aToB.magnitude();
    if (length <= error || length <= 0)
    {
        return 0;
    }

    // Resolving moves the first body along the normal, toward the second
    contact->body[0] = body[0];
    contact->body[1] = body[1];
    contact->normal = aToB * (1 / length);
    contact->point = (a + b) * (real)0.5;
    contact->penetration = length - error;
    contact->friction = 1;
    contact->restitution = 0;
    return 1;
}

namespace
{
    // What an impulse at a body's joint point does to it: zero for no body or an immovable one
    struct JointSide
    {
        RigidBody* pBody = nullptr;
        Vector2 arm; // From the body's centre to its joint point
        real inverseMass = 0;
        real inverseInertia = 0;
    };

    JointSide jointSide(RigidBody* pBody, const Vector2& worldPoint)
    {
        JointSide side;
        if (pBody != nullptr && pBody->hasFiniteMass())
        {
            side.pBody = pBody;
            side.arm = worldPoint - pBody->getPosition();
            side.inverseMass = pBody->getInverseMass();
            side.inverseInertia = pBody->getInverseInertia();
        }
        return side;
    }

    // The impulse that changes the gap between the two joint points by exactly wanted
    Vector2 jointImpulse(const JointSide& a, const JointSide& b, const Vector2& wanted)
    {
        const real mass = a.inverseMass + b.inverseMass;
        const real k11 = mass + a.inverseInertia * a.arm.y * a.arm.y + b.inverseInertia * b.arm.y * b.arm.y;
        const real k12 = -a.inverseInertia * a.arm.x * a.arm.y - b.inverseInertia * b.arm.x * b.arm.y;
        const real k22 = mass + a.inverseInertia * a.arm.x * a.arm.x + b.inverseInertia * b.arm.x * b.arm.x;
        const real determinant = k11 * k22 - k12 * k12;
        if (determinant <= 0)
        {
            return Vector2();
        }
        return Vector2(k22 * wanted.x - k12 * wanted.y, k11 * wanted.y - k12 * wanted.x) * (1 / determinant);
    }
}

bool Joint::prepare() const
{
    const bool moves0 = body[0] != nullptr && body[0]->hasFiniteMass();
    const bool moves1 = body[1] != nullptr && body[1]->hasFiniteMass();
    if (!moves0 && !moves1)
    {
        return false;
    }
    const bool awake0 = moves0 && body[0]->isAwake();
    const bool awake1 = moves1 && body[1]->isAwake();
    if (!awake0 && !awake1)
    {
        return false;
    }
    if (moves0 && !awake0)
    {
        body[0]->setAwake(true);
    }
    if (moves1 && !awake1)
    {
        body[1]->setAwake(true);
    }
    return true;
}

void Joint::solveVelocity() const
{
    const Vector2 point0 = getWorldPosition(0);
    const Vector2 point1 = getWorldPosition(1);
    const JointSide a = jointSide(body[0], point0);
    const JointSide b = jointSide(body[1], point1);

    Vector2 relativeVelocity;
    if (b.pBody != nullptr)
    {
        relativeVelocity += b.pBody->getVelocityAtPoint(point1);
    }
    if (a.pBody != nullptr)
    {
        relativeVelocity -= a.pBody->getVelocityAtPoint(point0);
    }

    const Vector2 impulse = jointImpulse(a, b, relativeVelocity);
    if (a.pBody != nullptr)
    {
        a.pBody->setLinearVelocity(a.pBody->getLinearVelocity() + impulse * a.inverseMass);
        a.pBody->setAngularVelocity(a.pBody->getAngularVelocity() + a.arm.cross(impulse) * a.inverseInertia);
    }
    if (b.pBody != nullptr)
    {
        b.pBody->setLinearVelocity(b.pBody->getLinearVelocity() - impulse * b.inverseMass);
        b.pBody->setAngularVelocity(b.pBody->getAngularVelocity() - b.arm.cross(impulse) * b.inverseInertia);
    }
}

real Joint::solvePosition() const
{
    const Vector2 point0 = getWorldPosition(0);
    const Vector2 point1 = getWorldPosition(1);
    Vector2 gap = point1 - point0;
    const real length = gap.magnitude();
    if (length <= error || length <= 0)
    {
        return length;
    }
    gap *= (length - error) / length;

    const JointSide a = jointSide(body[0], point0);
    const JointSide b = jointSide(body[1], point1);
    const Vector2 move = jointImpulse(a, b, gap);
    if (a.pBody != nullptr)
    {
        a.pBody->setPosition(a.pBody->getPosition() + move * a.inverseMass);
        a.pBody->setOrientation(a.pBody->getOrientation() + a.arm.cross(move) * a.inverseInertia);
    }
    if (b.pBody != nullptr)
    {
        b.pBody->setPosition(b.pBody->getPosition() - move * b.inverseMass);
        b.pBody->setOrientation(b.pBody->getOrientation() - b.arm.cross(move) * b.inverseInertia);
    }
    return length;
}
