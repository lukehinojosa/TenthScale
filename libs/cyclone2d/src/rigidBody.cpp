#include "rigidBody.h"

using namespace cyclone2d;

real RigidBody::sleepEpsilon = (real)0.3;

void RigidBody::setLinearVelocity(Vector2 v)
{
    m_linearVelocity = v;
    if (!m_isAwake && v.squareMagnitude() > 0)
    {
        setAwake(true);
    }
}

void RigidBody::setAngularVelocity(real w)
{
    m_angularVelocity = w;
    if (!m_isAwake && w != 0)
    {
        setAwake(true);
    }
}

void RigidBody::setOrientation(real angle)
{
    m_orientation = angle;
    m_rotation.set(angle);
}

void RigidBody::setInertia(real inertia)
{
    m_inverseInertia = inertia <= 0 ? 0 : ((real)1.0) / inertia;
}

void RigidBody::setBoxMass(real mass, const Vector2& halfSize)
{
    setMass(mass);
    const real width = 2 * halfSize.x;
    const real height = 2 * halfSize.y;
    setInertia(mass * (width * width + height * height) / 12);
}

void RigidBody::setCircleMass(real mass, real radius)
{
    setMass(mass);
    setInertia(mass * radius * radius / 2);
}

Vector2 RigidBody::getVelocityAtPoint(const Vector2& worldPoint) const
{
    return m_linearVelocity + cross(m_angularVelocity, worldPoint - m_position);
}

void RigidBody::integrate(real dt)
{
    integrate(dt, true);
}

void RigidBody::integrate(real dt, bool allowSleep)
{
    // A sleeping body doesn't move until something wakes it. Forces still clear, so they don't pile up.
    if (!m_isAwake)
    {
        clearAccumulators();
        return;
    }

    m_lastFrameAcceleration = m_acceleration;
    m_lastFrameAcceleration.addScaledVector(m_forceAccum, m_inverseMass);
    m_linearVelocity.addScaledVector(m_lastFrameAcceleration, dt);
    m_angularVelocity += m_torqueAccum * m_inverseInertia * dt;

    m_linearVelocity *= real_pow(m_linearDamping, dt);
    m_angularVelocity *= real_pow(m_angularDamping, dt);

    m_position.addScaledVector(m_linearVelocity, dt);
    if (m_angularVelocity != 0)
    {
        setOrientation(m_orientation + m_angularVelocity * dt);
    }

    clearAccumulators();

    if (m_canSleep)
    {
        // A recency-weighted average, so one slow frame doesn't put a moving body to sleep
        const real currentMotion = m_linearVelocity.squareMagnitude() + m_angularVelocity * m_angularVelocity;
        const real bias = real_pow((real)0.5, dt);
        m_motion = bias * m_motion + (1 - bias) * currentMotion;
        if (allowSleep && m_motion < sleepEpsilon)
        {
            setAwake(false);
        }
        else if (m_motion > 10 * sleepEpsilon)
        {
            m_motion = 10 * sleepEpsilon;
        }
    }
}

void RigidBody::setMass(real mass)
{
    m_inverseMass = mass <= 0 ? 0 : ((real)1.0) / mass;
}

void RigidBody::addForce(Vector2 force)
{
    m_forceAccum += force;
    wakeIfAsleep();
}

void RigidBody::addForceAtPoint(Vector2 force, Vector2 point)
{
    m_forceAccum += force;
    m_torqueAccum += (point - m_position).cross(force);
    wakeIfAsleep();
}

void RigidBody::addForceAtBodyPoint(Vector2 force, Vector2 localPoint)
{
    addForceAtPoint(force, getPointInWorldSpace(localPoint));
}

void RigidBody::addTorque(real torque)
{
    m_torqueAccum += torque;
    wakeIfAsleep();
}

void RigidBody::wakeIfAsleep()
{
    // Through setAwake, so the motion average restarts too; otherwise the body would doze straight off again
    if (!m_isAwake)
    {
        setAwake(true);
    }
}

void RigidBody::setAwake(bool awake)
{
    if (awake)
    {
        m_isAwake = true;
        m_motion = sleepEpsilon * 2;
    }
    else
    {
        m_isAwake = false;
        m_linearVelocity.clear();
        m_angularVelocity = 0;
    }
}

void RigidBody::setCanSleep(bool canSleep)
{
    m_canSleep = canSleep;
    if (!canSleep && !m_isAwake)
    {
        setAwake(true);
    }
}

void RigidBody::clearAccumulators()
{
    m_forceAccum.clear();
    m_torqueAccum = 0;
}
