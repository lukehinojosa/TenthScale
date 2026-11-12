#include "collision.h"

#include <algorithm>

// Convex polyhedra: building them, their mass properties, and contacts against everything else (chapter 13.4.3)

using namespace cyclone;

namespace
{
    constexpr real FACE_SLACK = (real)0.01;

    struct WorldHull
    {
        std::vector<Vector3> vertices;
        std::vector<Vector3> faceNormals;
        std::vector<real> faceOffsets;
        std::vector<Vector3> edgeDirections;
        Vector3 centre;
    };

    WorldHull toWorld(const ConvexCollider& convex)
    {
        WorldHull hull;
        const Matrix4& t = convex.transform;
        hull.centre = convex.getAxis(3);
        hull.vertices.reserve(convex.getVertices().size());
        for (const Vector3& v : convex.getVertices())
        {
            hull.vertices.push_back(t.transform(v));
        }
        for (const ConvexCollider::Face& face : convex.getFaces())
        {
            const Vector3 normal = t.transformDirection(face.normal);
            hull.faceNormals.push_back(normal);
            hull.faceOffsets.push_back(normal * hull.vertices[face.indices[0]]);
        }
        for (const Vector3& direction : convex.getEdgeDirections())
        {
            hull.edgeDirections.push_back(t.transformDirection(direction));
        }
        return hull;
    }

    void project(const std::vector<Vector3>& vertices, const Vector3& axis, real& low, real& high)
    {
        low = REAL_MAX;
        high = -REAL_MAX;
        for (const Vector3& v : vertices)
        {
            const real d = v * axis;
            low = std::min(low, d);
            high = std::max(high, d);
        }
    }

    // Is the point within the face's outline, seen along its normal (with a little slack)?
    bool insideFace(const std::vector<Vector3>& vertices, const ConvexCollider::Face& face, const Vector3& normal, const Vector3& point,
                    real slack)
    {
        const std::size_t count = face.indices.size();
        for (std::size_t i = 0; i < count; i++)
        {
            const Vector3& a = vertices[face.indices[i]];
            const Vector3& b = vertices[face.indices[(i + 1) % count]];
            const Vector3 outward = (b - a) % normal;
            const real length = outward.magnitude();
            if (length > 0 && (point - a) * outward > slack * length)
            {
                return false;
            }
        }
        return true;
    }

    Vector3 closestOnSegment(const Vector3& a, const Vector3& b, const Vector3& p)
    {
        const Vector3 ab = b - a;
        const real lengthSquared = ab.squareMagnitude();
        if (lengthSquared <= 0)
        {
            return a;
        }
        real t = ((p - a) * ab) / lengthSquared;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        return a + ab * t;
    }

    // The closest points between two segments, averaged into one contact point
    Vector3 segmentContact(const Vector3& p1, const Vector3& q1, const Vector3& p2, const Vector3& q2)
    {
        const Vector3 d1 = q1 - p1;
        const Vector3 d2 = q2 - p2;
        const Vector3 r = p1 - p2;
        const real a = d1 * d1;
        const real e = d2 * d2;
        const real f = d2 * r;
        real s = 0;
        real t = 0;
        if (a > 0 && e > 0)
        {
            const real c = d1 * r;
            const real b = d1 * d2;
            const real denom = a * e - b * b;
            s = denom > (real)1e-12 ? std::clamp((b * f - c * e) / denom, (real)0, (real)1) : 0;
            t = (b * s + f) / e;
            if (t < 0)
            {
                t = 0;
                s = std::clamp(-c / a, (real)0, (real)1);
            }
            else if (t > 1)
            {
                t = 1;
                s = std::clamp((b - c) / a, (real)0, (real)1);
            }
        }
        const Vector3 c1 = p1 + d1 * s;
        const Vector3 c2 = p2 + d2 * t;
        return (c1 + c2) * (real)0.5;
    }

    bool nearExisting(const Contact* first, int count, const Vector3& point)
    {
        for (int i = 0; i < count; i++)
        {
            if ((first[i].point - point).squareMagnitude() < (real)1e-6)
            {
                return true;
            }
        }
        return false;
    }

    // Corners of `corners` that are through `face` of `owner` and inside its outline. faceOutward looks at the corners.
    int addFaceCorners(const WorldHull& ownerHull, const ConvexCollider::Face& face, const Vector3& faceOutward,
                       const WorldHull& cornerHull, const Vector3& normal, real towardOwner, RigidBody* bodyA, RigidBody* bodyB,
                       Contact* first, int already, CollisionData* data)
    {
        const real faceOffset = faceOutward * ownerHull.vertices[face.indices[0]];
        int added = 0;
        for (const Vector3& v : cornerHull.vertices)
        {
            // Corners a little apart are kept with their gap as a negative depth, so a lifting corner still holds
            const real depth = faceOffset - v * faceOutward;
            if (depth < -FACE_SLACK || !insideFace(ownerHull.vertices, face, faceOutward, v, FACE_SLACK))
            {
                continue;
            }
            // Halfway between the corner and the face it's through
            const Vector3 point = v + normal * (towardOwner * depth * (real)0.5);
            if (nearExisting(first, already + added, point))
            {
                continue;
            }
            if (data->addContact(bodyA, bodyB, point, normal, depth) == nullptr)
            {
                break;
            }
            added++;
        }
        return added;
    }

    // The face of a hull whose outward normal points most along direction
    int faceFacing(const ConvexCollider& shape, const WorldHull& hull, const Vector3& direction)
    {
        int best = 0;
        real bestDot = -REAL_MAX;
        for (int f = 0; f < (int)shape.getFaces().size(); f++)
        {
            const real d = hull.faceNormals[f] * direction;
            if (d > bestDot)
            {
                bestDot = d;
                best = f;
            }
        }
        return best;
    }

    // A face contact from corners of each shape through the other's facing face. The axis only gives a direction, and a
    // shape such as a box has a face each way along it, so each side uses the face that points at the other shape.
    int fillHullFace(const ConvexCollider& one, const WorldHull& oneHull, const ConvexCollider& two, const WorldHull& twoHull,
                     Vector3 normal, real penetration, CollisionData* data)
    {
        Contact* first = data->contacts;
        const int oneFace = faceFacing(one, oneHull, normal * -1);
        int added = addFaceCorners(oneHull, one.getFaces()[oneFace], oneHull.faceNormals[oneFace], twoHull, normal, -1, one.pBody,
                                   two.pBody, first, 0, data);
        const int twoFace = faceFacing(two, twoHull, normal);
        added += addFaceCorners(twoHull, two.getFaces()[twoFace], twoHull.faceNormals[twoFace], oneHull, normal, 1, one.pBody, two.pBody,
                                first, added, data);
        if (added > 0)
        {
            return added;
        }

        // Nothing inside either outline: the corner of two that reaches furthest toward one
        const Vector3* deepest = &twoHull.vertices[0];
        for (const Vector3& v : twoHull.vertices)
        {
            if (v * normal > *deepest * normal)
            {
                deepest = &v;
            }
        }
        return data->addContact(one.pBody, two.pBody, *deepest, normal, penetration) != nullptr ? 1 : 0;
    }
}

void ConvexCollider::setShape(const std::vector<Vector3>& newVertices, const std::vector<std::vector<int>>& newFaces)
{
    vertices = newVertices;
    faces.clear();
    edges.clear();
    edgeDirections.clear();

    Vector3 middle;
    for (const Vector3& v : vertices)
    {
        middle += v;
    }
    if (!vertices.empty())
    {
        middle *= (real)1 / (real)vertices.size();
    }

    for (const std::vector<int>& indices : newFaces)
    {
        if (indices.size() < 3)
        {
            continue;
        }
        Face face;
        face.indices = indices;

        // Newell's method copes with faces that aren't quite flat
        Vector3 normal;
        for (std::size_t i = 0; i < indices.size(); i++)
        {
            const Vector3& a = vertices[indices[i]];
            const Vector3& b = vertices[indices[(i + 1) % indices.size()]];
            normal.x += (a.y - b.y) * (a.z + b.z);
            normal.y += (a.z - b.z) * (a.x + b.x);
            normal.z += (a.x - b.x) * (a.y + b.y);
        }
        normal.normalise();
        if ((vertices[indices[0]] - middle) * normal < 0)
        {
            normal = normal * -1;
            std::reverse(face.indices.begin(), face.indices.end());
        }
        face.normal = normal;
        face.offset = normal * vertices[face.indices[0]];
        faces.push_back(face);

        for (std::size_t i = 0; i < face.indices.size(); i++)
        {
            int a = face.indices[i];
            int b = face.indices[(i + 1) % face.indices.size()];
            if (a > b)
            {
                std::swap(a, b);
            }
            if (std::find(edges.begin(), edges.end(), std::make_pair(a, b)) == edges.end())
            {
                edges.emplace_back(a, b);
            }
        }
    }

    for (const auto& edge : edges)
    {
        const Vector3 direction = (vertices[edge.second] - vertices[edge.first]).unit();
        bool known = false;
        for (const Vector3& existing : edgeDirections)
        {
            if (real_abs(existing * direction) > (real)0.9999)
            {
                known = true;
                break;
            }
        }
        if (!known)
        {
            edgeDirections.push_back(direction);
        }
    }

    boundingRadius = 0;
    for (const Vector3& v : vertices)
    {
        boundingRadius = std::max(boundingRadius, v.magnitude());
    }
}

ConvexCollider ConvexCollider::box(const Vector3& h, RigidBody* body)
{
    ConvexCollider convex(body);
    convex.setShape({ Vector3(-h.x, -h.y, -h.z), Vector3(h.x, -h.y, -h.z), Vector3(h.x, h.y, -h.z), Vector3(-h.x, h.y, -h.z),
                      Vector3(-h.x, -h.y, h.z), Vector3(h.x, -h.y, h.z), Vector3(h.x, h.y, h.z), Vector3(-h.x, h.y, h.z) },
                    { { 0, 3, 2, 1 }, { 4, 5, 6, 7 }, { 0, 1, 5, 4 }, { 2, 3, 7, 6 }, { 0, 4, 7, 3 }, { 1, 2, 6, 5 } });
    return convex;
}

ConvexCollider ConvexCollider::wedge(const Vector3& h, RigidBody* body)
{
    // A right triangle (tall end at +x) pushed out along z, centred on its centroid
    const real cx = h.x * (real)4 / 3;
    const real cy = h.y * (real)2 / 3;
    const real left = -cx;
    const real right = 2 * h.x - cx;
    const real bottom = -cy;
    const real top = 2 * h.y - cy;
    ConvexCollider convex(body);
    convex.setShape({ Vector3(left, bottom, -h.z), Vector3(right, bottom, -h.z), Vector3(right, top, -h.z), Vector3(left, bottom, h.z),
                      Vector3(right, bottom, h.z), Vector3(right, top, h.z) },
                    { { 0, 1, 2 }, { 3, 4, 5 }, { 0, 1, 4, 3 }, { 1, 2, 5, 4 }, { 0, 2, 5, 3 } });
    return convex;
}

ConvexCollider ConvexCollider::tetrahedron(real size, RigidBody* body)
{
    const real s = size * (real)0.5;
    ConvexCollider convex(body);
    convex.setShape({ Vector3(s, s, s), Vector3(-s, -s, s), Vector3(-s, s, -s), Vector3(s, -s, -s) },
                    { { 0, 1, 2 }, { 0, 3, 1 }, { 0, 2, 3 }, { 1, 3, 2 } });
    return convex;
}

ConvexCollider ConvexCollider::octahedron(real radius, RigidBody* body)
{
    const real r = radius;
    ConvexCollider convex(body);
    convex.setShape({ Vector3(r, 0, 0), Vector3(-r, 0, 0), Vector3(0, r, 0), Vector3(0, -r, 0), Vector3(0, 0, r), Vector3(0, 0, -r) },
                    { { 0, 2, 4 }, { 2, 1, 4 }, { 1, 3, 4 }, { 3, 0, 4 }, { 2, 0, 5 }, { 1, 2, 5 }, { 3, 1, 5 }, { 0, 3, 5 } });
    return convex;
}

namespace
{
    // Accumulates volume, first moment, and covariance over the tetrahedra from the origin to each face triangle
    void massSums(const std::vector<Vector3>& vertices, const std::vector<ConvexCollider::Face>& faces, real& volume, Vector3& moment,
                  real covariance[3][3])
    {
        volume = 0;
        moment = Vector3();
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                covariance[i][j] = 0;
            }
        }
        for (const ConvexCollider::Face& face : faces)
        {
            const Vector3& a = vertices[face.indices[0]];
            for (std::size_t k = 1; k + 1 < face.indices.size(); k++)
            {
                const Vector3& b = vertices[face.indices[k]];
                const Vector3& c = vertices[face.indices[k + 1]];
                const real det = a * (b % c);
                volume += det / 6;
                moment += (a + b + c) * (det / 24);
                const Vector3 sum = a + b + c;
                const real va[3] = { a.x, a.y, a.z };
                const real vb[3] = { b.x, b.y, b.z };
                const real vc[3] = { c.x, c.y, c.z };
                const real vs[3] = { sum.x, sum.y, sum.z };
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        covariance[i][j] += det * (va[i] * va[j] + vb[i] * vb[j] + vc[i] * vc[j] + vs[i] * vs[j]) / 120;
                    }
                }
            }
        }
    }
}

real ConvexCollider::getVolume() const
{
    real volume;
    Vector3 moment;
    real covariance[3][3];
    massSums(vertices, faces, volume, moment, covariance);
    return volume;
}

Vector3 ConvexCollider::getCentroid() const
{
    real volume;
    Vector3 moment;
    real covariance[3][3];
    massSums(vertices, faces, volume, moment, covariance);
    return volume > 0 ? moment * (1 / volume) : Vector3();
}

Matrix3 ConvexCollider::computeInertiaTensor(real mass) const
{
    real volume;
    Vector3 moment;
    real covariance[3][3];
    massSums(vertices, faces, volume, moment, covariance);
    if (volume <= 0)
    {
        return Matrix3();
    }
    const real density = mass / volume;
    const Vector3 centroid = moment * (1 / volume);
    const real c[3] = { centroid.x, centroid.y, centroid.z };

    // Covariance of the solid about its centroid, then the inertia tensor from it
    real cov[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cov[i][j] = covariance[i][j] * density - mass * c[i] * c[j];
        }
    }
    const real trace = cov[0][0] + cov[1][1] + cov[2][2];
    return Matrix3(trace - cov[0][0], -cov[0][1], -cov[0][2], -cov[1][0], trace - cov[1][1], -cov[1][2], -cov[2][0], -cov[2][1],
                   trace - cov[2][2]);
}

int CollisionTests::ConvexHalfspace(const ConvexCollider& convex, const PlaneCollider& plane, CollisionData* data)
{
    if (data->contactsLeft <= 0)
    {
        return 0;
    }
    const Vector3 centre = convex.getAxis(3);
    if (plane.normal * centre - plane.offset > convex.getBoundingRadius())
    {
        return 0;
    }
    int added = 0;
    for (const Vector3& local : convex.getVertices())
    {
        const Vector3 v = convex.transform.transform(local);
        const real distance = v * plane.normal;
        if (distance > plane.offset)
        {
            continue;
        }
        const real depth = plane.offset - distance;
        if (data->addContact(convex.pBody, plane.pBody, v + plane.normal * (depth * (real)0.5), plane.normal, depth) == nullptr)
        {
            break;
        }
        added++;
    }
    return added;
}

int CollisionTests::ConvexSphere(const ConvexCollider& convex, const SphereCollider& sphere, CollisionData* data)
{
    if (data->contactsLeft <= 0)
    {
        return 0;
    }
    const Vector3 centreWorld = sphere.getAxis(3);
    const real radius = sphere.radius;
    if ((centreWorld - convex.getAxis(3)).magnitude() > convex.getBoundingRadius() + radius)
    {
        return 0;
    }

    // Work in the hull's own space
    const Vector3 c = convex.transform.transformInverse(centreWorld);
    const std::vector<Vector3>& vertices = convex.getVertices();
    const std::vector<ConvexCollider::Face>& faces = convex.getFaces();

    real bestDistance = REAL_MAX;
    Vector3 bestPoint;
    bool outside = false;
    int leastDeepFace = 0;
    real leastDeep = -REAL_MAX;
    for (int f = 0; f < (int)faces.size(); f++)
    {
        const ConvexCollider::Face& face = faces[f];
        const real distance = face.normal * c - face.offset;
        if (distance > leastDeep)
        {
            leastDeep = distance;
            leastDeepFace = f;
        }
        if (distance <= 0)
        {
            continue;
        }
        outside = true;

        // Closest point on this face: straight down onto it, or else on its edges
        const Vector3 onPlane = c - face.normal * distance;
        Vector3 candidate = onPlane;
        if (!insideFace(vertices, face, face.normal, onPlane, 0))
        {
            real nearest = REAL_MAX;
            for (std::size_t i = 0; i < face.indices.size(); i++)
            {
                const Vector3 p = closestOnSegment(vertices[face.indices[i]], vertices[face.indices[(i + 1) % face.indices.size()]], c);
                const real d = (p - c).squareMagnitude();
                if (d < nearest)
                {
                    nearest = d;
                    candidate = p;
                }
            }
        }
        const real d = (candidate - c).magnitude();
        if (d < bestDistance)
        {
            bestDistance = d;
            bestPoint = candidate;
        }
    }

    // The normal points from the sphere toward the hull
    if (outside)
    {
        if (bestDistance > radius || bestDistance <= 0)
        {
            return 0;
        }
        const Vector3 pointWorld = convex.transform.transform(bestPoint);
        const Vector3 normal = (pointWorld - centreWorld).unit();
        return data->addContact(convex.pBody, sphere.pBody, pointWorld, normal, radius - bestDistance) != nullptr ? 1 : 0;
    }

    // Centre inside: out through the nearest face
    const Vector3 normal = convex.transform.transformDirection(faces[leastDeepFace].normal) * -1;
    return data->addContact(convex.pBody, sphere.pBody, centreWorld, normal, radius - leastDeep) != nullptr ? 1 : 0;
}

int CollisionTests::ConvexConvex(const ConvexCollider& one, const ConvexCollider& two, CollisionData* data, SatCache* cache)
{
    if (data->contactsLeft <= 0)
    {
        return 0;
    }
    if ((one.getAxis(3) - two.getAxis(3)).magnitude() > one.getBoundingRadius() + two.getBoundingRadius())
    {
        return 0;
    }
    const WorldHull a = toWorld(one);
    const WorldHull b = toWorld(two);
    const int faceAxes = (int)(a.faceNormals.size() + b.faceNormals.size());
    const int edgeAxes = (int)(a.edgeDirections.size() * b.edgeDirections.size());

    auto axisFor = [&](int index)
    {
        if (index < (int)a.faceNormals.size())
        {
            return a.faceNormals[index];
        }
        if (index < faceAxes)
        {
            return b.faceNormals[index - a.faceNormals.size()];
        }
        const int e = index - faceAxes;
        return a.edgeDirections[e / b.edgeDirections.size()] % b.edgeDirections[e % b.edgeDirections.size()];
    };
    // The overlap along an axis; negative when it separates them
    auto overlapOn = [&](Vector3 axis, bool& usable)
    {
        usable = axis.squareMagnitude() > (real)0.0001;
        if (!usable)
        {
            return REAL_MAX;
        }
        axis.normalise();
        real aLow;
        real aHigh;
        real bLow;
        real bHigh;
        project(a.vertices, axis, aLow, aHigh);
        project(b.vertices, axis, bLow, bHigh);
        return std::min(aHigh, bHigh) - std::max(aLow, bLow);
    };

    if (cache != nullptr)
    {
        cache->lastTestHit = false;
        if (cache->separatingAxis >= 0 && cache->separatingAxis < faceAxes + edgeAxes)
        {
            bool usable;
            if (overlapOn(axisFor(cache->separatingAxis), usable) < 0 && usable)
            {
                cache->lastTestHit = true;
                return 0;
            }
        }
    }

    real smallest = REAL_MAX;
    int best = -1;
    for (int index = 0; index < faceAxes + edgeAxes; index++)
    {
        bool usable;
        const real overlap = overlapOn(axisFor(index), usable);
        if (!usable)
        {
            continue;
        }
        if (overlap < 0)
        {
            if (cache != nullptr)
            {
                cache->separatingAxis = index;
            }
            return 0;
        }
        real preference = index < faceAxes ? (real)1 : (real)0.95;

        // Crossed edges of faces turned slightly against each other give an axis almost along a face normal. That's
        // really the face contact, so it can still separate the shapes but never wins.
        if (index >= faceAxes)
        {
            const Vector3 unit = axisFor(index).unit();
            for (const Vector3& faceNormal : a.faceNormals)
            {
                preference = real_abs(unit * faceNormal) > (real)0.999 ? 0 : preference;
            }
            for (const Vector3& faceNormal : b.faceNormals)
            {
                preference = real_abs(unit * faceNormal) > (real)0.999 ? 0 : preference;
            }
        }
        if (overlap < smallest * preference)
        {
            smallest = overlap;
            best = index;
        }
    }
    if (cache != nullptr)
    {
        cache->separatingAxis = -1;
    }
    if (best < 0)
    {
        return 0;
    }

    // The normal points from two toward one
    Vector3 normal = axisFor(best).unit();
    if (normal * (a.centre - b.centre) < 0)
    {
        normal = normal * -1;
    }

    if (best < (int)a.faceNormals.size())
    {
        return fillHullFace(one, a, two, b, normal, smallest, data);
    }
    if (best < faceAxes)
    {
        return fillHullFace(two, b, one, a, normal * -1, smallest, data);
    }

    // Edge against edge: on each shape, the edge along that direction nearest the other shape
    const int e = best - faceAxes;
    const Vector3 dirA = a.edgeDirections[e / b.edgeDirections.size()];
    const Vector3 dirB = b.edgeDirections[e % b.edgeDirections.size()];
    auto pickEdge = [&](const ConvexCollider& shape, const WorldHull& hull, const Vector3& direction, real sign)
    {
        std::pair<int, int> chosen = shape.getEdges()[0];
        real bestScore = REAL_MAX;
        for (const auto& edge : shape.getEdges())
        {
            const Vector3 along = (hull.vertices[edge.second] - hull.vertices[edge.first]).unit();
            if (real_abs(along * direction) < (real)0.999)
            {
                continue;
            }
            const real score = sign * ((hull.vertices[edge.first] + hull.vertices[edge.second]) * normal);
            if (score < bestScore)
            {
                bestScore = score;
                chosen = edge;
            }
        }
        return chosen;
    };
    // One's contact side faces -normal, two's faces +normal
    const auto edgeA = pickEdge(one, a, dirA, 1);
    const auto edgeB = pickEdge(two, b, dirB, -1);
    const Vector3 point =
        segmentContact(a.vertices[edgeA.first], a.vertices[edgeA.second], b.vertices[edgeB.first], b.vertices[edgeB.second]);
    return data->addContact(one.pBody, two.pBody, point, normal, smallest) != nullptr ? 1 : 0;
}

int CollisionTests::ConvexBox(const ConvexCollider& convex, const BoxCollider& box, CollisionData* data, SatCache* cache)
{
    // A box is just a convex shape with six faces
    ConvexCollider asConvex = ConvexCollider::box(box.halfSize, box.pBody);
    asConvex.transform = box.transform;
    return ConvexConvex(convex, asConvex, data, cache);
}
