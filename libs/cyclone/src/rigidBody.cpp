#include "rigidBody.h"

using namespace cyclone;

// A transform matrix from a position and orientation (chapter 10)
static void calculateTransformMatrix(Matrix4& transformMatrix, Vector3 position, Quaternion orientation)
{
    transformMatrix.data[0] = 1 - 2 * orientation.j * orientation.j - 2 * orientation.k * orientation.k;
    transformMatrix.data[1] = 2 * orientation.i * orientation.j - 2 * orientation.r * orientation.k;
    transformMatrix.data[2] = 2 * orientation.i * orientation.k + 2 * orientation.r * orientation.j;
    transformMatrix.data[3] = position.x;

    transformMatrix.data[4] = 2 * orientation.i * orientation.j + 2 * orientation.r * orientation.k;
    transformMatrix.data[5] = 1 - 2 * orientation.i * orientation.i - 2 * orientation.k * orientation.k;
    transformMatrix.data[6] = 2 * orientation.j * orientation.k - 2 * orientation.r * orientation.i;
    transformMatrix.data[7] = position.y;

    transformMatrix.data[8] = 2 * orientation.i * orientation.k - 2 * orientation.r * orientation.j;
    transformMatrix.data[9] = 2 * orientation.j * orientation.k + 2 * orientation.r * orientation.i;
    transformMatrix.data[10] = 1 - 2 * orientation.i * orientation.i - 2 * orientation.j * orientation.j;
    transformMatrix.data[11] = position.z;
}

Matrix4 RigidBody::getTransformationMatrix() const
{
    Matrix4 transform;
    calculateTransformMatrix(transform, m_position, m_orientation);
    return transform;
}

real RigidBody::sleepEpsilon = (real)0.3;

void RigidBody::setInertiaTensor(const Matrix3& inertiaTensor)
{
    m_inverseInertiaTensor.setInverse(inertiaTensor);
}

void RigidBody::setLinearVelocity(Vector3 v)
{
    m_linearVelocity = v;
    if (!m_isAwake && v.squareMagnitude() > 0)
    {
        setAwake(true);
    }
}

void RigidBody::setAngularVelocity(Vector3 w)
{
    m_angularVelocity = w;
    if (!m_isAwake && w.squareMagnitude() > 0)
    {
        setAwake(true);
    }
}

Vector3 RigidBody::getPointInWorldSpace(const Vector3& localPoint) const
{
    return getTransformationMatrix().transform(localPoint);
}

Vector3 RigidBody::getPointInLocalSpace(const Vector3& worldPoint) const
{
    return getTransformationMatrix().transformInverse(worldPoint);
}

Vector3 RigidBody::getDirectionInWorldSpace(const Vector3& localDirection) const
{
    return getTransformationMatrix().transformDirection(localDirection);
}

Vector3 RigidBody::getDirectionInLocalSpace(const Vector3& worldDirection) const
{
    return getTransformationMatrix().transformInverseDirection(worldDirection);
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

    if (m_linearDamping != 1)
    {
        m_linearVelocity *= real_pow(m_linearDamping, dt);
    }

    m_position.addScaledVector(m_linearVelocity, dt);

    const Vector3 zero(0, 0, 0);
    if (!(m_angularVelocity == zero) || !(m_torqueAccum == zero))
    {
        // With no torque the angular acceleration is zero, so the tensor isn't needed
        if (!(m_torqueAccum == zero))
        {
            Vector3 angularAcceleration = getInverseInertiaTensorWorld().transform(m_torqueAccum);
            m_angularVelocity.addScaledVector(angularAcceleration, dt);
        }
        if (m_angularDamping != 1)
        {
            m_angularVelocity *= real_pow(m_angularDamping, dt);
        }
        m_orientation.addScaledVector(m_angularVelocity, dt);
        m_orientation.normalise(); // Stops the quaternion drifting away from unit length
    }

    clearAccumulators();

    if (m_canSleep)
    {
        // A recency-weighted average, so one slow frame doesn't put a moving body to sleep
        const real currentMotion = m_linearVelocity.squareMagnitude() + m_angularVelocity.squareMagnitude();
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
    if (mass <= 0)
    {
        m_inverseMass = 0;
    }
    else
    {
        m_inverseMass = ((real)1.0) / mass;
    }
}

void RigidBody::addForce(Vector3 force)
{
    m_forceAccum += force;
    wakeIfAsleep();
}

void RigidBody::addForceAtPoint(Vector3 force, Vector3 point)
{
    Vector3 leverArm = point - m_position;
    m_forceAccum += force;
    m_torqueAccum += leverArm % force;
    wakeIfAsleep();
}

void RigidBody::addForceAtBodyPoint(Vector3 force, Vector3 localPoint)
{
    addForceAtPoint(force, getPointInWorldSpace(localPoint));
}

void RigidBody::addTorque(Vector3 torque)
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
        m_angularVelocity.clear();
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
    m_torqueAccum.clear();
}

Matrix3 RigidBody::getInverseInertiaTensorWorld() const
{
    const Matrix4 transform = getTransformationMatrix();
    Matrix3 rotation(transform.data[0], transform.data[1], transform.data[2], transform.data[4], transform.data[5], transform.data[6],
                     transform.data[8], transform.data[9], transform.data[10]);
    return rotation * m_inverseInertiaTensor * rotation.transpose();
}
