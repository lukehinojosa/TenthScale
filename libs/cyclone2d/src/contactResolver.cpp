#include "contactResolver.h"

using namespace cyclone2d;

namespace
{
    // Bodies with infinite mass, or none at all, never move
    bool movable(const RigidBody* body)
    {
        return body != nullptr && body->hasFiniteMass();
    }
}

void ContactResolver::resolveContacts(Contact* contacts, unsigned numContacts, real duration)
{
    if (numContacts == 0 || duration <= 0)
    {
        return;
    }
    prepareContacts(contacts, numContacts, duration);
    adjustPositions();
    adjustVelocities(duration);

    // Hand the remaining penetration back, so callers can see what was left
    for (unsigned i = 0; i < numContacts; i++)
    {
        contacts[i].penetration = states[i].penetration;
    }
}

void ContactResolver::prepareContacts(const Contact* contacts, unsigned numContacts, real duration)
{
    states.resize(numContacts);
    for (unsigned i = 0; i < numContacts; i++)
    {
        const Contact& contact = contacts[i];
        ContactState& c = states[i];
        c = ContactState();
        c.body[0] = contact.body[0];
        c.body[1] = contact.body[1];
        c.normal = contact.normal;
        c.penetration = contact.penetration;
        c.friction = contact.friction;
        c.restitution = contact.restitution;

        // The maths needs a body first, so swap if only the second has one
        if (c.body[0] == nullptr)
        {
            c.normal = -c.normal;
            c.body[0] = c.body[1];
            c.body[1] = nullptr;
        }
        if (c.body[0] == nullptr || (!movable(c.body[0]) && !movable(c.body[1])))
        {
            // Nothing here can move, so the resolver skips it
            c.body[0] = nullptr;
            c.penetration = 0;
            continue;
        }

        c.tangent = c.normal.perpendicular();
        c.relativeContactPosition[0] = contact.point - c.body[0]->getPosition();
        if (c.body[1] != nullptr)
        {
            c.relativeContactPosition[1] = contact.point - c.body[1]->getPosition();
        }

        c.contactVelocity = calculateLocalVelocity(c, 0, duration);
        if (c.body[1] != nullptr)
        {
            c.contactVelocity -= calculateLocalVelocity(c, 1, duration);
        }
        calculateDesiredDeltaVelocity(c, duration);
    }
}

Vector2 ContactResolver::calculateLocalVelocity(const ContactState& c, int bodyIndex, real duration) const
{
    const RigidBody* body = c.body[bodyIndex];
    const Vector2 velocity = body->getLinearVelocity() + cross(body->getAngularVelocity(), c.relativeContactPosition[bodyIndex]);
    Vector2 contactVelocity(velocity.dot(c.normal), velocity.dot(c.tangent));

    // Sideways velocity gained from this frame's forces is removed by friction, so count it now
    contactVelocity.y += (body->getLastFrameAcceleration() * duration).dot(c.tangent);
    return contactVelocity;
}

void ContactResolver::calculateDesiredDeltaVelocity(ContactState& c, real duration) const
{
    // Closing speed caused only by this frame's acceleration: a resting contact, not an impact (microcollisions)
    real velocityFromAcc = 0;
    if (c.body[0]->isAwake())
    {
        velocityFromAcc += (c.body[0]->getLastFrameAcceleration() * duration).dot(c.normal);
    }
    if (c.body[1] != nullptr && c.body[1]->isAwake())
    {
        velocityFromAcc -= (c.body[1]->getLastFrameAcceleration() * duration).dot(c.normal);
    }

    real restitution = c.restitution;
    if (real_abs(c.contactVelocity.x) < velocityLimit)
    {
        restitution = 0;
    }
    c.desiredDeltaVelocity = -c.contactVelocity.x - restitution * (c.contactVelocity.x - velocityFromAcc);
}

void ContactResolver::matchAwakeState(ContactState& c)
{
    if (c.body[1] == nullptr)
    {
        return;
    }
    const bool awake0 = c.body[0]->isAwake();
    const bool awake1 = c.body[1]->isAwake();
    if (awake0 != awake1)
    {
        (awake0 ? c.body[1] : c.body[0])->setAwake(true);
    }
}

Vector2 ContactResolver::calculateFrictionlessImpulse(const ContactState& c) const
{
    // Velocity change along the normal for a unit impulse: linear part plus the spin it causes
    real deltaVelocity = 0;
    for (int b = 0; b < 2; b++)
    {
        const RigidBody* body = c.body[b];
        if (!movable(body))
        {
            continue;
        }
        const real rn = c.relativeContactPosition[b].cross(c.normal);
        deltaVelocity += body->getInverseMass() + body->getInverseInertia() * rn * rn;
    }
    if (deltaVelocity <= 0)
    {
        return Vector2();
    }
    return Vector2(c.desiredDeltaVelocity / deltaVelocity, 0);
}

Vector2 ContactResolver::calculateFrictionImpulse(const ContactState& c) const
{
    // How much a unit impulse along the normal and along the tangent changes the velocity along each
    real knn = 0;
    real knt = 0;
    real ktt = 0;
    for (int b = 0; b < 2; b++)
    {
        const RigidBody* body = c.body[b];
        if (!movable(body))
        {
            continue;
        }
        const real rn = c.relativeContactPosition[b].cross(c.normal);
        const real rt = c.relativeContactPosition[b].cross(c.tangent);
        const real inverseInertia = body->getInverseInertia();
        knn += body->getInverseMass() + inverseInertia * rn * rn;
        ktt += body->getInverseMass() + inverseInertia * rt * rt;
        knt += inverseInertia * rn * rt;
    }

    // The impulse that removes all sliding and gives the wanted normal velocity
    const real determinant = knn * ktt - knt * knt;
    if (real_abs(determinant) < (real)1e-12)
    {
        return calculateFrictionlessImpulse(c);
    }
    const real wantNormal = c.desiredDeltaVelocity;
    const real wantTangent = -c.contactVelocity.y;
    Vector2 impulse((ktt * wantNormal - knt * wantTangent) / determinant, (knn * wantTangent - knt * wantNormal) / determinant);

    // More sideways impulse than friction allows: slide instead, with the most friction the contact can give
    if (real_abs(impulse.y) > impulse.x * c.friction)
    {
        const real direction = impulse.y < 0 ? (real)-1 : (real)1;
        const real normalPerUnit = knn + knt * c.friction * direction;
        if (real_abs(normalPerUnit) < (real)1e-12)
        {
            return calculateFrictionlessImpulse(c);
        }
        impulse.x = wantNormal / normalPerUnit;
        impulse.y = direction * c.friction * impulse.x;
    }
    return impulse;
}

void ContactResolver::applyVelocityChange(ContactState& c, Vector2 velocityChange[2], real rotationChange[2]) const
{
    for (int b = 0; b < 2; b++)
    {
        velocityChange[b].clear();
        rotationChange[b] = 0;
    }

    const Vector2 impulseContact = c.friction == 0 ? calculateFrictionlessImpulse(c) : calculateFrictionImpulse(c);
    const Vector2 impulse = c.normal * impulseContact.x + c.tangent * impulseContact.y;

    // The second body gets the equal and opposite impulse
    for (int b = 0; b < 2; b++)
    {
        RigidBody* body = c.body[b];
        if (!movable(body))
        {
            continue;
        }
        const Vector2 bodyImpulse = b == 0 ? impulse : -impulse;
        velocityChange[b] = bodyImpulse * body->getInverseMass();
        rotationChange[b] = c.relativeContactPosition[b].cross(bodyImpulse) * body->getInverseInertia();
        body->setLinearVelocity(body->getLinearVelocity() + velocityChange[b]);
        body->setAngularVelocity(body->getAngularVelocity() + rotationChange[b]);
    }
}

void ContactResolver::applyPositionChange(ContactState& c, Vector2 linearChange[2], real angularChange[2], real penetration) const
{
    // How easily each body moves and turns at the contact point decides its share of the fix
    real angularInertia[2] = { 0, 0 };
    real linearInertia[2] = { 0, 0 };
    real totalInertia = 0;
    for (int b = 0; b < 2; b++)
    {
        linearChange[b].clear();
        angularChange[b] = 0;
        if (!movable(c.body[b]))
        {
            continue;
        }
        const real rn = c.relativeContactPosition[b].cross(c.normal);
        angularInertia[b] = c.body[b]->getInverseInertia() * rn * rn;
        linearInertia[b] = c.body[b]->getInverseMass();
        totalInertia += linearInertia[b] + angularInertia[b];
    }
    if (totalInertia <= 0)
    {
        return;
    }

    for (int b = 0; b < 2; b++)
    {
        RigidBody* body = c.body[b];
        if (!movable(body))
        {
            continue;
        }
        const real sign = b == 0 ? (real)1 : (real)-1;
        real angularMove = sign * penetration * (angularInertia[b] / totalInertia);
        real linearMove = sign * penetration * (linearInertia[b] / totalInertia);

        // Turning a long way at a contact far from the centre looks wrong, so cap it and move more instead
        const real maxMagnitude = angularLimit * real_abs(c.relativeContactPosition[b].cross(c.normal));
        if (angularMove < -maxMagnitude || angularMove > maxMagnitude)
        {
            const real totalMove = angularMove + linearMove;
            angularMove = angularMove < 0 ? -maxMagnitude : maxMagnitude;
            linearMove = totalMove - angularMove;
        }

        if (angularMove != 0 && angularInertia[b] != 0)
        {
            // The turn that moves the contact point angularMove along the normal
            const real rn = c.relativeContactPosition[b].cross(c.normal);
            angularChange[b] = body->getInverseInertia() * rn * (angularMove / angularInertia[b]);
        }
        linearChange[b] = c.normal * linearMove;

        body->setPosition(body->getPosition() + linearChange[b]);
        body->setOrientation(body->getOrientation() + angularChange[b]);
    }
}

void ContactResolver::adjustPositions()
{
    const unsigned count = (unsigned)states.size();
    const unsigned limit = positionIterations > 0 ? positionIterations : count * 32;
    Vector2 linearChange[2];
    real angularChange[2];
    positionIterationsUsed = 0;
    while (positionIterationsUsed < limit)
    {
        // The deepest contact first
        real max = positionEpsilon;
        unsigned index = count;
        for (unsigned i = 0; i < count; i++)
        {
            if (states[i].body[0] != nullptr && states[i].penetration > max)
            {
                max = states[i].penetration;
                index = i;
            }
        }
        if (index == count)
        {
            break;
        }

        ContactState& resolved = states[index];
        matchAwakeState(resolved);
        applyPositionChange(resolved, linearChange, angularChange, max);

        // Moving those bodies changed the depth of every other contact that shares them
        RigidBody* const moved[2] = { resolved.body[0], resolved.body[1] };
        for (ContactState& other : states)
        {
            for (int b = 0; b < 2; b++)
            {
                if (other.body[b] == nullptr)
                {
                    continue;
                }
                for (int d = 0; d < 2; d++)
                {
                    if (other.body[b] == moved[d])
                    {
                        const Vector2 deltaPosition = linearChange[d] + cross(angularChange[d], other.relativeContactPosition[b]);
                        other.penetration += deltaPosition.dot(other.normal) * (b == 1 ? (real)1 : (real)-1);
                    }
                }
            }
        }
        positionIterationsUsed++;
    }
}

void ContactResolver::adjustVelocities(real duration)
{
    const unsigned count = (unsigned)states.size();
    const unsigned limit = velocityIterations > 0 ? velocityIterations : count * 32;
    Vector2 velocityChange[2];
    real rotationChange[2];
    velocityIterationsUsed = 0;
    while (velocityIterationsUsed < limit)
    {
        // The contact that most needs its velocity changed first
        real max = velocityEpsilon;
        unsigned index = count;
        for (unsigned i = 0; i < count; i++)
        {
            if (states[i].body[0] != nullptr && states[i].desiredDeltaVelocity > max)
            {
                max = states[i].desiredDeltaVelocity;
                index = i;
            }
        }
        if (index == count)
        {
            break;
        }

        ContactState& resolved = states[index];
        matchAwakeState(resolved);
        applyVelocityChange(resolved, velocityChange, rotationChange);

        // Recompute the closing velocity of every contact sharing those bodies
        RigidBody* const changed[2] = { resolved.body[0], resolved.body[1] };
        for (ContactState& other : states)
        {
            for (int b = 0; b < 2; b++)
            {
                if (other.body[b] == nullptr)
                {
                    continue;
                }
                for (int d = 0; d < 2; d++)
                {
                    if (other.body[b] == changed[d])
                    {
                        const Vector2 deltaVel = velocityChange[d] + cross(rotationChange[d], other.relativeContactPosition[b]);
                        const real sign = b == 1 ? (real)-1 : (real)1;
                        other.contactVelocity += Vector2(deltaVel.dot(other.normal), deltaVel.dot(other.tangent)) * sign;
                        calculateDesiredDeltaVelocity(other, duration);
                    }
                }
            }
        }
        velocityIterationsUsed++;
    }
}
