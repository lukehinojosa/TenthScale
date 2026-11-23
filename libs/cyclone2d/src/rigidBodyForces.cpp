#include "rigidBodyForces.h"

#include <algorithm>

using namespace cyclone2d;

void RigidBodyForceRegistry::add(RigidBody* body, RigidBodyForceGenerator* generator)
{
    registrations.push_back(Registration{ body, generator });
}

void RigidBodyForceRegistry::remove(RigidBody* body, RigidBodyForceGenerator* generator)
{
    auto found = std::find_if(registrations.begin(), registrations.end(),
                              [&](const Registration& r) { return r.body == body && r.generator == generator; });
    if (found != registrations.end())
    {
        registrations.erase(found);
    }
}

void RigidBodyForceRegistry::updateForces(real duration)
{
    for (const Registration& r : registrations)
    {
        r.generator->updateForce(r.body, duration);
    }
}

void RigidBodyGravity::updateForce(RigidBody* body, real)
{
    if (!body->hasFiniteMass())
    {
        return;
    }
    body->addForce(gravity * body->getMass());
}

void RigidBodySpring::updateForce(RigidBody* body, real)
{
    const Vector2 lws = body->getPointInWorldSpace(connectionPoint);
    const Vector2 ows = other != nullptr ? other->getPointInWorldSpace(otherConnectionPoint) : otherConnectionPoint;
    const Vector2 delta = lws - ows;
    const real length = delta.magnitude();
    if (length <= 0)
    {
        return;
    }
    const Vector2 direction = delta * (1 / length);

    Vector2 relativeVelocity = body->getVelocityAtPoint(lws);
    if (other != nullptr)
    {
        relativeVelocity -= other->getVelocityAtPoint(ows);
    }
    const real magnitude = -springConstant * (length - restLength) - damping * relativeVelocity.dot(direction);
    body->addForceAtPoint(direction * magnitude, lws);
}

void RigidBodyBuoyancy::updateForce(RigidBody* body, real)
{
    const real depth = body->getPointInWorldSpace(centreOfBuoyancy).y;
    if (depth >= waterHeight + maxDepth)
    {
        return;
    }
    real submerged = (waterHeight + maxDepth - depth) / (2 * maxDepth);
    submerged = std::min(submerged, (real)1);
    body->addForceAtBodyPoint(Vector2(0, liquidDensity * volume * gravity * submerged), centreOfBuoyancy);
}
