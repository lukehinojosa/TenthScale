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
