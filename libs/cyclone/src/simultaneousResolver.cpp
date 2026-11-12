#include "simultaneousResolver.h"

#include <algorithm>

using namespace cyclone;

namespace
{
    bool movable(const RigidBody* body)
    {
        return body != nullptr && body->hasFiniteMass();
    }

    void tangentsFor(const Vector3& n, Vector3& t0, Vector3& t1)
    {
        if (real_abs(n.x) > real_abs(n.y))
        {
            t0 = Vector3(n.z, 0, -n.x).unit();
        }
        else
        {
            t0 = Vector3(0, -n.z, n.y).unit();
        }
        t1 = n % t0;
    }
}

real SimultaneousContactResolver::effectiveMass(const Row& row, const Vector3& direction) const
{
    // One over the velocity change along `direction` that a unit impulse causes: linear plus rotational
    real k = 0;
    for (int b = 0; b < 2; b++)
    {
        const RigidBody* body = row.body[b];
        if (!movable(body))
        {
            continue;
        }
        const Vector3 rotated = body->getInverseInertiaTensorWorld().transform(row.relative[b] % direction);
        k += body->getInverseMass() + (rotated % row.relative[b]) * direction;
    }
    return k > 0 ? 1 / k : 0;
}

Vector3 SimultaneousContactResolver::relativeVelocity(const Row& row) const
{
    Vector3 velocity;
    for (int b = 0; b < 2; b++)
    {
        const RigidBody* body = row.body[b];
        if (body == nullptr)
        {
            continue;
        }
        const Vector3 v = body->getLinearVelocity() + body->getAngularVelocity() % row.relative[b];
        velocity += b == 0 ? v : v * -1;
    }
    return velocity;
}

void SimultaneousContactResolver::applyImpulse(Row& row, const Vector3& impulse) const
{
    for (int b = 0; b < 2; b++)
    {
        RigidBody* body = row.body[b];
        if (!movable(body))
        {
            continue;
        }
        const Vector3 p = b == 0 ? impulse : impulse * -1;
        body->setLinearVelocity(body->getLinearVelocity() + p * body->getInverseMass());
        body->setAngularVelocity(body->getAngularVelocity() + body->getInverseInertiaTensorWorld().transform(row.relative[b] % p));
    }
}

void SimultaneousContactResolver::resolveContacts(Contact* contacts, unsigned numContacts, real duration)
{
    rows.clear();
    if (numContacts == 0 || duration <= 0)
    {
        previous.clear();
        return;
    }

    for (unsigned i = 0; i < numContacts; i++)
    {
        const Contact& contact = contacts[i];
        Row row;
        row.body[0] = contact.body[0];
        row.body[1] = contact.body[1];
        row.normal = contact.normal;
        if (row.body[0] == nullptr)
        {
            std::swap(row.body[0], row.body[1]);
            row.normal = row.normal * -1;
        }
        if (!movable(row.body[0]) && !movable(row.body[1]))
        {
            continue;
        }
        row.friction = contact.friction;
        for (int b = 0; b < 2; b++)
        {
            if (row.body[b] != nullptr)
            {
                row.relative[b] = contact.point - row.body[b]->getPosition();
            }
        }
        tangentsFor(row.normal, row.tangent[0], row.tangent[1]);
        row.normalMass = effectiveMass(row, row.normal);
        row.tangentMass[0] = effectiveMass(row, row.tangent[0]);
        row.tangentMass[1] = effectiveMass(row, row.tangent[1]);
        row.localPoint = row.body[0]->getPointInLocalSpace(contact.point);

        // Push out penetration over a few steps, and bounce only off real impacts
        const real closing = relativeVelocity(row) * row.normal;
        row.bias = positionCorrection / duration * std::max(contact.penetration - penetrationSlop, (real)0);
        if (contact.penetration < 0)
        {
            // Not touching yet: the gap may close this step, but no faster
            row.bias = contact.penetration / duration;
        }
        else if (closing < -velocityLimit)
        {
            row.bias = std::max(row.bias, -contact.restitution * closing);
        }
        rows.push_back(row);
    }

    // Warm start: a contact between the same bodies at nearly the same spot keeps last frame's impulses
    if (warmStarting)
    {
        for (Row& row : rows)
        {
            for (const Row& old : previous)
            {
                if (old.body[0] == row.body[0] && old.body[1] == row.body[1] &&
                    (old.localPoint - row.localPoint).squareMagnitude() < (real)0.0025)
                {
                    row.normalImpulse = old.normalImpulse;
                    row.tangentImpulse[0] = old.tangentImpulse[0];
                    row.tangentImpulse[1] = old.tangentImpulse[1];
                    applyImpulse(row, row.normal * row.normalImpulse + row.tangent[0] * row.tangentImpulse[0] +
                                          row.tangent[1] * row.tangentImpulse[1]);
                    break;
                }
            }
        }
    }

    // Projected Gauss-Seidel over the Jacobian rows
    for (unsigned iteration = 0; iteration < iterations; iteration++)
    {
        for (Row& row : rows)
        {
            // Friction first, limited by the current normal impulse
            for (int t = 0; t < 2; t++)
            {
                const real speed = relativeVelocity(row) * row.tangent[t];
                const real limit = row.friction * row.normalImpulse;
                const real old = row.tangentImpulse[t];
                row.tangentImpulse[t] = std::clamp(old - speed * row.tangentMass[t], -limit, limit);
                applyImpulse(row, row.tangent[t] * (row.tangentImpulse[t] - old));
            }

            // The normal impulse can only push
            const real closing = relativeVelocity(row) * row.normal;
            const real old = row.normalImpulse;
            row.normalImpulse = std::max(old + (row.bias - closing) * row.normalMass, (real)0);
            applyImpulse(row, row.normal * (row.normalImpulse - old));
        }
    }
    previous = rows;
}
