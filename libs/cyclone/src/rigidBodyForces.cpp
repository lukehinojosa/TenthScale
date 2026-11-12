#include "rigidBodyForces.h"

#include <algorithm>

using namespace cyclone;

namespace
{
    // The same rotation RigidBody builds into its transform
    Matrix3 rotationOf(const Quaternion& q)
    {
        return Matrix3(1 - 2 * q.j * q.j - 2 * q.k * q.k, 2 * q.i * q.j - 2 * q.r * q.k, 2 * q.i * q.k + 2 * q.r * q.j,
                       2 * q.i * q.j + 2 * q.r * q.k, 1 - 2 * q.i * q.i - 2 * q.k * q.k, 2 * q.j * q.k - 2 * q.r * q.i,
                       2 * q.i * q.k - 2 * q.r * q.j, 2 * q.j * q.k + 2 * q.r * q.i, 1 - 2 * q.i * q.i - 2 * q.j * q.j);
    }
}

void RigidBodyForceRegistry::add(RigidBody* body, RigidBodyForceGenerator* generator)
{
    registrations.push_back(Registration{ body, generator });
}

void RigidBodyForceRegistry::remove(RigidBody* body, RigidBodyForceGenerator* generator)
{
    registrations.erase(std::remove_if(registrations.begin(), registrations.end(),
                                       [&](const Registration& r) { return r.body == body && r.generator == generator; }),
                        registrations.end());
}

void RigidBodyForceRegistry::clear()
{
    registrations.clear();
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

RigidBodySpring::RigidBodySpring(const Vector3& localConnectionPoint, RigidBody* other, const Vector3& otherConnectionPoint,
                                 real springConstant, real restLength, real damping)
    : connectionPoint(localConnectionPoint), other(other), otherConnectionPoint(otherConnectionPoint), springConstant(springConstant),
      restLength(restLength), damping(damping)
{
}

void RigidBodySpring::updateForce(RigidBody* body, real)
{
    const Vector3 here = body->getPointInWorldSpace(connectionPoint);
    const Vector3 there = other->getPointInWorldSpace(otherConnectionPoint);
    Vector3 delta = here - there;
    const real length = delta.magnitude();
    if (length <= 0)
    {
        return;
    }
    const Vector3 direction = delta * (1 / length);

    // Stretching pulls the points together and compressing pushes them apart
    real magnitude = springConstant * (length - restLength);
    if (damping != 0)
    {
        const Vector3 hereVelocity = body->getLinearVelocity() + body->getAngularVelocity() % (here - body->getPosition());
        const Vector3 thereVelocity = other->getLinearVelocity() + other->getAngularVelocity() % (there - other->getPosition());
        magnitude += damping * ((hereVelocity - thereVelocity) * direction);
    }
    body->addForceAtPoint(direction * -magnitude, here);
}

RigidBodyBuoyancy::RigidBodyBuoyancy(const Vector3& centreOfBuoyancy, real maxDepth, real volume, real waterHeight, real liquidDensity,
                                     real gravity)
    : centreOfBuoyancy(centreOfBuoyancy), maxDepth(maxDepth), volume(volume), waterHeight(waterHeight), liquidDensity(liquidDensity),
      gravity(gravity)
{
}

void RigidBodyBuoyancy::updateForce(RigidBody* body, real)
{
    const real depth = body->getPointInWorldSpace(centreOfBuoyancy).y;
    if (depth >= waterHeight + maxDepth || maxDepth <= 0)
    {
        return;
    }

    // Fully under, or the fraction under as the point sinks through the band around the surface
    real submerged = 1;
    if (depth > waterHeight - maxDepth)
    {
        submerged = (waterHeight + maxDepth - depth) / (2 * maxDepth);
    }
    body->addForceAtBodyPoint(Vector3(0, liquidDensity * volume * gravity * submerged, 0), centreOfBuoyancy);
}

Aero::Aero(const Matrix3& tensor, const Vector3& position, const Vector3* windSpeed)
    : tensor(tensor), position(position), windSpeed(windSpeed)
{
}

void Aero::updateForce(RigidBody* body, real)
{
    updateForceFromTensor(body, tensor);
}

void Aero::updateForceFromTensor(RigidBody* body, const Matrix3& tensorToUse) const
{
    // The air's velocity relative to the body, as seen in the body's own space
    Vector3 velocity = body->getLinearVelocity();
    if (windSpeed != nullptr)
    {
        velocity += *windSpeed;
    }
    const Vector3 bodyVelocity = body->getDirectionInLocalSpace(velocity);
    const Vector3 bodyForce = tensorToUse.transform(bodyVelocity);
    body->addForceAtBodyPoint(body->getDirectionInWorldSpace(bodyForce), position);
}

AeroControl::AeroControl(const Matrix3& base, const Matrix3& min, const Matrix3& max, const Vector3& position, const Vector3* windSpeed)
    : Aero(base, position, windSpeed), minTensor(min), maxTensor(max)
{
}

void AeroControl::setControl(real value)
{
    controlSetting = std::clamp(value, (real)-1, (real)1);
}

Matrix3 AeroControl::getTensor() const
{
    if (controlSetting <= -1)
    {
        return minTensor;
    }
    if (controlSetting >= 1)
    {
        return maxTensor;
    }
    if (controlSetting < 0)
    {
        return Matrix3::linearInterpolate(minTensor, tensor, controlSetting + 1);
    }
    if (controlSetting > 0)
    {
        return Matrix3::linearInterpolate(tensor, maxTensor, controlSetting);
    }
    return tensor;
}

void AeroControl::updateForce(RigidBody* body, real)
{
    updateForceFromTensor(body, getTensor());
}

AngledAero::AngledAero(const Matrix3& tensor, const Vector3& position, const Vector3* windSpeed) : Aero(tensor, position, windSpeed)
{
}

void AngledAero::setOrientation(const Quaternion& value)
{
    orientation = value;
    orientation.normalise();
}

void AngledAero::updateForce(RigidBody* body, real)
{
    // The surface's tensor turned from its own frame into the body's
    const Matrix3 rotation = rotationOf(orientation);
    const Matrix3 bodyTensor = rotation * tensor * rotation.transpose();
    updateForceFromTensor(body, bodyTensor);
}

void Explosion::detonate(const Vector3& position)
{
    detonation = position;
    timePassed = 0;
}

void Explosion::advance(real duration)
{
    if (timePassed != REAL_MAX)
    {
        timePassed += duration;
    }
}

bool Explosion::isActive() const
{
    if (timePassed == REAL_MAX)
    {
        return false;
    }
    const real end = std::max(implosionDuration + concussionDuration, implosionDuration + convectionDuration);
    return timePassed < end;
}

void Explosion::updateForce(RigidBody* body, real)
{
    if (!isActive() || !body->hasFiniteMass())
    {
        return;
    }
    const Vector3 position = body->getPosition();
    Vector3 away = position - detonation;
    const real distance = away.magnitude();
    const Vector3 direction = distance > 0 ? away * (1 / distance) : Vector3(0, 1, 0);

    if (timePassed < implosionDuration)
    {
        if (distance > implosionMinRadius && distance < implosionMaxRadius)
        {
            body->addForce(direction * -implosionForce);
        }
        return;
    }

    const real t = timePassed - implosionDuration;
    if (t < concussionDuration && shockwaveThickness > 0)
    {
        // Strongest on the wave front, fading to nothing at its edges and weakening as the wave spreads
        const real front = shockwaveSpeed * t;
        const real offset = real_abs(distance - front);
        const real halfThickness = shockwaveThickness * (real)0.5;
        if (offset < halfThickness)
        {
            const real fade = 1 - t / concussionDuration;
            body->addForce(direction * (peakConcussionForce * (1 - offset / halfThickness) * fade));
        }
    }

    if (t < convectionDuration && chimneyRadius > 0)
    {
        const real dx = position.x - detonation.x;
        const real dz = position.z - detonation.z;
        const real horizontal = real_sqrt(dx * dx + dz * dz);
        const real height = position.y - detonation.y;
        if (horizontal < chimneyRadius && height >= 0 && height < chimneyHeight)
        {
            const real fade = 1 - t / convectionDuration;
            body->addForce(Vector3(0, peakConvectionForce * (1 - horizontal / chimneyRadius) * fade, 0));
        }
    }
}
