#include "rigidContactResolver.h"

using namespace cyclone;

namespace
{
    // Bodies with infinite mass, or none at all, never move
    bool movable(const RigidBody* body)
    {
        return body != nullptr && body->hasFiniteMass();
    }
}

void RigidContactResolver::resolveContacts(Contact* contacts, unsigned numContacts, real duration)
{
    if (numContacts == 0 || duration <= 0)
    {
        return;
    }
    prepareContacts(contacts, numContacts, duration);
    adjustPositions(duration);
    adjustVelocities(duration);

    // Hand the remaining penetration back, so callers can see what was left
    for (unsigned i = 0; i < numContacts; i++)
    {
        contacts[i].penetration = states[i].penetration;
    }
}

void RigidContactResolver::prepareContacts(const Contact* contacts, unsigned numContacts, real duration)
{
    states.resize(numContacts);
    for (unsigned i = 0; i < numContacts; i++)
    {
        const Contact& contact = contacts[i];
        ContactState& c = states[i];
        c.body[0] = contact.body[0];
        c.body[1] = contact.body[1];
        c.normal = contact.normal;
        c.penetration = contact.penetration;
        c.friction = contact.friction;
        c.restitution = contact.restitution;

        // The maths needs a body first, so swap if only the second has one
        if (c.body[0] == nullptr)
        {
            c.normal = c.normal * -1;
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

        calculateContactBasis(c);
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

void RigidContactResolver::calculateContactBasis(ContactState& c) const
{
    // Two directions across the normal; which world axis to start from depends on which is further from the normal
    const Vector3& n = c.normal;
    Vector3 tangent[2];
    if (real_abs(n.x) > real_abs(n.y))
    {
        const real s = 1 / real_sqrt(n.z * n.z + n.x * n.x);
        tangent[0] = Vector3(n.z * s, 0, -n.x * s);
        tangent[1] = Vector3(n.y * tangent[0].x, n.z * tangent[0].x - n.x * tangent[0].z, -n.y * tangent[0].x);
    }
    else
    {
        const real s = 1 / real_sqrt(n.z * n.z + n.y * n.y);
        tangent[0] = Vector3(0, -n.z * s, n.y * s);
        tangent[1] = Vector3(n.y * tangent[0].z - n.z * tangent[0].y, -n.x * tangent[0].z, n.x * tangent[0].y);
    }
    c.contactToWorld.setComponents(n, tangent[0], tangent[1]);
}

Vector3 RigidContactResolver::calculateLocalVelocity(const ContactState& c, int bodyIndex, real duration) const
{
    const RigidBody* body = c.body[bodyIndex];
    Vector3 velocity = body->getAngularVelocity() % c.relativeContactPosition[bodyIndex];
    velocity += body->getLinearVelocity();
    Vector3 contactVelocity = c.contactToWorld.transformTranspose(velocity);

    // Sideways velocity gained from this frame's forces is removed by friction, so count it now
    Vector3 accVelocity = c.contactToWorld.transformTranspose(body->getLastFrameAcceleration() * duration);
    accVelocity.x = 0;
    contactVelocity += accVelocity;
    return contactVelocity;
}

void RigidContactResolver::calculateDesiredDeltaVelocity(ContactState& c, real duration) const
{
    // Not touching yet: the gap may close this step, but no faster, and nothing bounces
    if (c.penetration < 0)
    {
        c.desiredDeltaVelocity = -c.contactVelocity.x + c.penetration / duration;
        return;
    }

    // Closing speed caused only by this frame's acceleration: a resting contact, not an impact (microcollisions)
    real velocityFromAcc = 0;
    if (c.body[0]->isAwake())
    {
        velocityFromAcc += c.body[0]->getLastFrameAcceleration() * duration * c.normal;
    }
    if (c.body[1] != nullptr && c.body[1]->isAwake())
    {
        velocityFromAcc -= c.body[1]->getLastFrameAcceleration() * duration * c.normal;
    }

    real restitution = c.restitution;
    if (real_abs(c.contactVelocity.x) < velocityLimit)
    {
        restitution = 0;
    }
    c.desiredDeltaVelocity = -c.contactVelocity.x - restitution * (c.contactVelocity.x - velocityFromAcc);
}

void RigidContactResolver::matchAwakeState(ContactState& c)
{
    if (c.body[1] == nullptr)
    {
        return;
    }
    const bool awake0 = c.body[0]->isAwake();
    const bool awake1 = c.body[1]->isAwake();
    if (awake0 != awake1)
    {
        if (awake0)
        {
            c.body[1]->setAwake(true);
        }
        else
        {
            c.body[0]->setAwake(true);
        }
    }
}

Vector3 RigidContactResolver::calculateFrictionlessImpulse(const ContactState& c, const Matrix3* inverseInertia) const
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
        Vector3 deltaVelWorld = c.relativeContactPosition[b] % c.normal;
        deltaVelWorld = inverseInertia[b].transform(deltaVelWorld);
        deltaVelWorld = deltaVelWorld % c.relativeContactPosition[b];
        deltaVelocity += deltaVelWorld * c.normal + body->getInverseMass();
    }
    if (deltaVelocity <= 0)
    {
        return Vector3();
    }
    return Vector3(c.desiredDeltaVelocity / deltaVelocity, 0, 0);
}

Vector3 RigidContactResolver::calculateFrictionImpulse(const ContactState& c, const Matrix3* inverseInertia) const
{
    // The full matrix of velocity change per unit impulse, in contact coordinates
    real inverseMass = 0;
    Matrix3 deltaVelWorld;
    for (int b = 0; b < 2; b++)
    {
        const RigidBody* body = c.body[b];
        if (!movable(body))
        {
            continue;
        }
        Matrix3 impulseToTorque;
        impulseToTorque.setSkewSymmetric(c.relativeContactPosition[b]);
        Matrix3 term = impulseToTorque;
        term *= inverseInertia[b];
        term *= impulseToTorque;
        term *= -1;
        deltaVelWorld += term;
        inverseMass += body->getInverseMass();
    }

    Matrix3 deltaVelocity = c.contactToWorld.transpose();
    deltaVelocity *= deltaVelWorld;
    deltaVelocity *= c.contactToWorld;
    deltaVelocity.data[0] += inverseMass;
    deltaVelocity.data[4] += inverseMass;
    deltaVelocity.data[8] += inverseMass;

    // The impulse that removes all sliding and gives the wanted normal velocity
    const Matrix3 impulseMatrix = deltaVelocity.inverse();
    const Vector3 velKill(c.desiredDeltaVelocity, -c.contactVelocity.y, -c.contactVelocity.z);
    Vector3 impulseContact = impulseMatrix.transform(velKill);

    // More sideways impulse than friction allows: slide instead, with the most friction the contact can give
    const real planarImpulse = real_sqrt(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);
    if (planarImpulse > impulseContact.x * c.friction)
    {
        impulseContact.y /= planarImpulse;
        impulseContact.z /= planarImpulse;
        const real normalPerUnit = deltaVelocity.data[0] + deltaVelocity.data[1] * c.friction * impulseContact.y +
                                   deltaVelocity.data[2] * c.friction * impulseContact.z;
        if (real_abs(normalPerUnit) < (real)1e-12)
        {
            return calculateFrictionlessImpulse(c, inverseInertia);
        }
        impulseContact.x = c.desiredDeltaVelocity / normalPerUnit;
        impulseContact.y *= c.friction * impulseContact.x;
        impulseContact.z *= c.friction * impulseContact.x;
    }
    return impulseContact;
}

void RigidContactResolver::applyVelocityChange(ContactState& c, Vector3 velocityChange[2], Vector3 rotationChange[2]) const
{
    Matrix3 inverseInertia[2];
    for (int b = 0; b < 2; b++)
    {
        velocityChange[b].clear();
        rotationChange[b].clear();
        if (movable(c.body[b]))
        {
            inverseInertia[b] = c.body[b]->getInverseInertiaTensorWorld();
        }
    }

    const Vector3 impulseContact =
        c.friction == 0 ? calculateFrictionlessImpulse(c, inverseInertia) : calculateFrictionImpulse(c, inverseInertia);
    const Vector3 impulse = c.contactToWorld.transform(impulseContact);

    if (movable(c.body[0]))
    {
        RigidBody* body = c.body[0];
        rotationChange[0] = inverseInertia[0].transform(c.relativeContactPosition[0] % impulse);
        velocityChange[0] = impulse * body->getInverseMass();
        body->setLinearVelocity(body->getLinearVelocity() + velocityChange[0]);
        body->setAngularVelocity(body->getAngularVelocity() + rotationChange[0]);
    }
    if (movable(c.body[1]))
    {
        // The second body gets the equal and opposite impulse
        RigidBody* body = c.body[1];
        rotationChange[1] = inverseInertia[1].transform(impulse % c.relativeContactPosition[1]);
        velocityChange[1] = impulse * -body->getInverseMass();
        body->setLinearVelocity(body->getLinearVelocity() + velocityChange[1]);
        body->setAngularVelocity(body->getAngularVelocity() + rotationChange[1]);
    }
}

void RigidContactResolver::applyPositionChange(ContactState& c, Vector3 linearChange[2], Vector3 angularChange[2], real penetration) const
{
    // How easily each body moves and turns at the contact point decides its share of the fix
    real angularInertia[2] = { 0, 0 };
    real linearInertia[2] = { 0, 0 };
    Matrix3 inverseInertia[2];
    real totalInertia = 0;
    for (int b = 0; b < 2; b++)
    {
        linearChange[b].clear();
        angularChange[b].clear();
        if (!movable(c.body[b]))
        {
            continue;
        }
        inverseInertia[b] = c.body[b]->getInverseInertiaTensorWorld();
        Vector3 angularInertiaWorld = c.relativeContactPosition[b] % c.normal;
        angularInertiaWorld = inverseInertia[b].transform(angularInertiaWorld);
        angularInertiaWorld = angularInertiaWorld % c.relativeContactPosition[b];
        angularInertia[b] = angularInertiaWorld * c.normal;
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
        Vector3 projection = c.relativeContactPosition[b];
        projection.addScaledVector(c.normal, -(c.relativeContactPosition[b] * c.normal));
        const real maxMagnitude = angularLimit * projection.magnitude();
        if (angularMove < -maxMagnitude || angularMove > maxMagnitude)
        {
            const real totalMove = angularMove + linearMove;
            angularMove = angularMove < 0 ? -maxMagnitude : maxMagnitude;
            linearMove = totalMove - angularMove;
        }

        if (angularMove != 0 && angularInertia[b] != 0)
        {
            const Vector3 targetAngularDirection = c.relativeContactPosition[b] % c.normal;
            angularChange[b] = inverseInertia[b].transform(targetAngularDirection) * (angularMove / angularInertia[b]);
        }
        linearChange[b] = c.normal * linearMove;

        Vector3 position = body->getPosition();
        position.addScaledVector(c.normal, linearMove);
        body->setPosition(position);
        Quaternion orientation = body->getOrientation();
        orientation.addScaledVector(angularChange[b], 1);
        body->setOrientation(orientation);
    }
}

void RigidContactResolver::adjustPositions(real)
{
    const unsigned count = (unsigned)states.size();
    const unsigned limit = positionIterations > 0 ? positionIterations : count * 32;
    Vector3 linearChange[2];
    Vector3 angularChange[2];
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
                    if (other.body[b] == resolved.body[d])
                    {
                        const Vector3 deltaPosition = linearChange[d] + angularChange[d] % other.relativeContactPosition[b];
                        other.penetration += (deltaPosition * other.normal) * (b == 1 ? (real)1 : (real)-1);
                    }
                }
            }
        }
        positionIterationsUsed++;
    }
}

void RigidContactResolver::adjustVelocities(real duration)
{
    const unsigned count = (unsigned)states.size();
    const unsigned limit = velocityIterations > 0 ? velocityIterations : count * 32;
    Vector3 velocityChange[2];
    Vector3 rotationChange[2];
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
                    if (other.body[b] == resolved.body[d])
                    {
                        const Vector3 deltaVel = velocityChange[d] + rotationChange[d] % other.relativeContactPosition[b];
                        other.contactVelocity += other.contactToWorld.transformTranspose(deltaVel) * (b == 1 ? (real)-1 : (real)1);
                        calculateDesiredDeltaVelocity(other, duration);
                    }
                }
            }
        }
        velocityIterationsUsed++;
    }
}
