#include "joints.h"

using namespace cyclone;

void Joint::set(RigidBody* a, const Vector3& aPosition, RigidBody* b, const Vector3& bPosition, real allowedError)
{
    body[0] = a;
    body[1] = b;
    position[0] = aPosition;
    position[1] = bPosition;
    error = allowedError;
}

Vector3 Joint::getWorldPosition(int index) const
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
    const Vector3 a = getWorldPosition(0);
    const Vector3 b = getWorldPosition(1);
    const Vector3 aToB = b - a;
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
    // What an impulse at a body's joint point does to it: nothing for no body or an immovable one
    struct JointSide
    {
        RigidBody* pBody = nullptr;
        Vector3 arm; // From the body's centre to its joint point
        real inverseMass = 0;
        Matrix3 inverseInertia;
    };

    JointSide jointSide(RigidBody* pBody, const Vector3& worldPoint)
    {
        JointSide side;
        if (pBody != nullptr && pBody->hasFiniteMass())
        {
            side.pBody = pBody;
            side.arm = worldPoint - pBody->getPosition();
            side.inverseMass = pBody->getInverseMass();
            side.inverseInertia = pBody->getInverseInertiaTensorWorld();
        }
        return side;
    }

    // The impulse that changes the gap between the two joint points by exactly wanted
    Vector3 jointImpulse(const JointSide& a, const JointSide& b, const Vector3& wanted)
    {
        Matrix3 k;
        for (const JointSide* pSide : { &a, &b })
        {
            if (pSide->pBody == nullptr)
            {
                continue;
            }
            Matrix3 skew;
            skew.setSkewSymmetric(pSide->arm);
            Matrix3 term = skew;
            term *= pSide->inverseInertia;
            term *= skew;
            term *= -1;
            k += term;
            k.data[0] += pSide->inverseMass;
            k.data[4] += pSide->inverseMass;
            k.data[8] += pSide->inverseMass;
        }
        const real* d = k.data;
        const real determinant = d[0] * (d[4] * d[8] - d[5] * d[7]) - d[1] * (d[3] * d[8] - d[5] * d[6]) + d[2] * (d[3] * d[7] - d[4] * d[6]);
        if (real_abs(determinant) < (real)1e-12)
        {
            return Vector3();
        }
        return k.inverse().transform(wanted);
    }

    Vector3 velocityAtPoint(const RigidBody* pBody, const Vector3& arm)
    {
        return pBody->getLinearVelocity() + pBody->getAngularVelocity() % arm;
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
    const JointSide a = jointSide(body[0], getWorldPosition(0));
    const JointSide b = jointSide(body[1], getWorldPosition(1));

    Vector3 relativeVelocity;
    if (b.pBody != nullptr)
    {
        relativeVelocity += velocityAtPoint(b.pBody, b.arm);
    }
    if (a.pBody != nullptr)
    {
        relativeVelocity -= velocityAtPoint(a.pBody, a.arm);
    }

    const Vector3 impulse = jointImpulse(a, b, relativeVelocity);
    if (a.pBody != nullptr)
    {
        a.pBody->setLinearVelocity(a.pBody->getLinearVelocity() + impulse * a.inverseMass);
        a.pBody->setAngularVelocity(a.pBody->getAngularVelocity() + a.inverseInertia.transform(a.arm % impulse));
    }
    if (b.pBody != nullptr)
    {
        b.pBody->setLinearVelocity(b.pBody->getLinearVelocity() - impulse * b.inverseMass);
        b.pBody->setAngularVelocity(b.pBody->getAngularVelocity() - b.inverseInertia.transform(b.arm % impulse));
    }
}

real Joint::solvePosition() const
{
    const Vector3 point0 = getWorldPosition(0);
    const Vector3 point1 = getWorldPosition(1);
    Vector3 gap = point1 - point0;
    const real length = gap.magnitude();
    if (length <= error || length <= 0)
    {
        return length;
    }
    gap *= (length - error) / length;

    const JointSide a = jointSide(body[0], point0);
    const JointSide b = jointSide(body[1], point1);
    const Vector3 move = jointImpulse(a, b, gap);
    if (a.pBody != nullptr)
    {
        a.pBody->setPosition(a.pBody->getPosition() + move * a.inverseMass);
        Quaternion orientation = a.pBody->getOrientation();
        orientation.addScaledVector(a.inverseInertia.transform(a.arm % move), 1);
        a.pBody->setOrientation(orientation);
    }
    if (b.pBody != nullptr)
    {
        b.pBody->setPosition(b.pBody->getPosition() - move * b.inverseMass);
        Quaternion orientation = b.pBody->getOrientation();
        orientation.addScaledVector(b.inverseInertia.transform(b.arm % move) * -1, 1);
        b.pBody->setOrientation(orientation);
    }
    return length;
}
