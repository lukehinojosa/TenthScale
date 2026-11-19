#include "flight/Rocket.h"

#include <algorithm>
#include <cmath>

#include "flight/Atmosphere.h"
#include "flight/SurfaceFrame.h"

namespace tenth
{
    namespace
    {
        constexpr double GROUND_FRICTION = 0.8;
        constexpr double GROUND_BOUNCE = 0.1;
        constexpr double TOUCH_DISTANCE = 0.02; // points this close above the ground count as touching it

        cyclone::Vector3 toCyclone(const Vec3d& v)
        {
            return { v.x, v.y, v.z };
        }

        Vec3d fromCyclone(const cyclone::Vector3& v)
        {
            return { v.x, v.y, v.z };
        }

        // The rotation that carries the rocket's own x, y, and z axes onto the three given directions
        cyclone::Quaternion fromAxes(const Vec3d& x, const Vec3d& y, const Vec3d& z)
        {
            const double trace = x.x + y.y + z.z;
            cyclone::Quaternion q;
            if (trace > 0.0)
            {
                const double s = std::sqrt(trace + 1.0) * 2.0;
                q = { s * 0.25, (y.z - z.y) / s, (z.x - x.z) / s, (x.y - y.x) / s };
            }
            else if (x.x > y.y && x.x > z.z)
            {
                const double s = std::sqrt(1.0 + x.x - y.y - z.z) * 2.0;
                q = { (y.z - z.y) / s, s * 0.25, (y.x + x.y) / s, (z.x + x.z) / s };
            }
            else if (y.y > z.z)
            {
                const double s = std::sqrt(1.0 + y.y - x.x - z.z) * 2.0;
                q = { (z.x - x.z) / s, (y.x + x.y) / s, s * 0.25, (z.y + y.z) / s };
            }
            else
            {
                const double s = std::sqrt(1.0 + z.z - x.x - y.y) * 2.0;
                q = { (x.y - y.x) / s, (z.x + x.z) / s, (z.y + y.z) / s, s * 0.25 };
            }
            q.normalise();
            return q;
        }
    }

    Rocket::Rocket()
    {
        // Space has no drag and the rocket must never doze off mid-flight, so damping and sleeping are off
        mBody.setLinearDamping(1.0);
        mBody.setAngularDamping(1.0);
        mBody.setCanSleep(false);
        mBody.setAcceleration({ 0.0, 0.0, 0.0 });
    }

    void Rocket::place(BodyId reference, const Vec3d& position, const Vec3d& velocity)
    {
        const SurfaceFrame upright = SurfaceFrame::below(position, 1.0);
        mReference = reference;
        mBody.setPosition(toCyclone(position));
        mBody.setLinearVelocity(toCyclone(velocity));
        mBody.setOrientation(fromAxes(upright.east, upright.up, upright.south));
        mBody.setAngularVelocity({ 0.0, 0.0, 0.0 });
        mOnGround = false;
    }

    void Rocket::setVehicle(const Vehicle& vehicle)
    {
        mVehicle = vehicle;
        applyMass();
    }

    void Rocket::applyMass()
    {
        if (mVehicle.mass <= 0.0)
        {
            return;
        }
        mBody.setMass(mVehicle.mass);

        // Cyclone takes the products of inertia, which are the negatives of the tensor's off-diagonal terms
        const Inertia& i = mVehicle.inertia;
        cyclone::Matrix3 tensor;
        tensor.setInertiaTensorCoeffs(i.xx, i.yy, i.zz, -i.xy, -i.xz, -i.yz);
        mBody.setInertiaTensor(tensor);
    }

    void Rocket::setThrottle(double throttle)
    {
        mVehicle.throttle = std::clamp(throttle, 0.0, 1.0);
    }

    std::vector<PartIndex> Rocket::fireNextStage()
    {
        return tenth::fireNextStage(mVehicle);
    }

    Vec3d Rocket::position() const
    {
        return fromCyclone(mBody.getPosition());
    }

    Vec3d Rocket::velocity() const
    {
        return fromCyclone(mBody.getLinearVelocity());
    }

    double Rocket::altitude(const SolarSystem& system) const
    {
        double lowest = position().length();
        for (const Vec3d& point : mVehicle.outline)
        {
            const Vec3d local = point - mVehicle.centerOfMass;
            lowest = std::min(lowest, fromCyclone(mBody.getPointInWorldSpace(toCyclone(local))).length());
        }
        return lowest - system.body(mReference).radius;
    }

    Vec3d Rocket::toWorld(const Vec3d& direction) const
    {
        return fromCyclone(mBody.getDirectionInWorldSpace(toCyclone(direction)));
    }

    Vec3d Rocket::spin() const
    {
        return fromCyclone(mBody.getAngularVelocity());
    }

    void Rocket::setSpin(const Vec3d& spin)
    {
        mBody.setAngularVelocity(toCyclone(spin));
    }

    void Rocket::turn(const Vec3d& axis, double angle)
    {
        const Vec3d a = axis.normalized() * std::sin(angle * 0.5);
        cyclone::Quaternion q(std::cos(angle * 0.5), a.x, a.y, a.z);
        q *= mBody.getOrientation();
        mBody.setOrientation(q);
    }

    Vec3d Rocket::absolutePosition(const SolarSystem& system, double time) const
    {
        return system.positionAt(mReference, time) + position();
    }

    Vec3d Rocket::absoluteVelocity(const SolarSystem& system, double time) const
    {
        return system.velocityAt(mReference, time) + velocity();
    }

    void Rocket::updateReference(const SolarSystem& system, double time)
    {
        const Vec3d absolute = absolutePosition(system, time);
        const BodyId dominant = system.dominantBody(absolute, time);
        if (dominant == mReference)
        {
            return;
        }
        // Re-expressed against the new body, so the rocket's true position and velocity don't jump
        const Vec3d absoluteVel = absoluteVelocity(system, time);
        mReference = dominant;
        mBody.setPosition(toCyclone(absolute - system.positionAt(mReference, time)));
        mBody.setLinearVelocity(toCyclone(absoluteVel - system.velocityAt(mReference, time)));
    }

    void Rocket::step(double dt, const SolarSystem& system, double time)
    {
        updateReference(system, time);

        // Gravity pulls every part equally, so it's an acceleration of the whole rocket rather than a force on it
        const Vec3d offset = position();
        const double distanceSquared = offset.lengthSquared();
        Vec3d gravity;
        if (distanceSquared > 0.0)
        {
            gravity = offset.normalized() * (-system.body(mReference).mu / distanceSquared);
        }
        mBody.setAcceleration(toCyclone(gravity));
        mPressure = pressureRatio(system.body(mReference), offset.length() - system.body(mReference).radius);
        pushWithEngines();
        mBody.integrate(dt);

        // Burned fuel moves the center of mass, so the body moves with it to keep the parts where they were
        const Vec3d oldCenter = mVehicle.centerOfMass;
        burnFuel(mVehicle, mPressure, dt);
        updateMass(mVehicle);
        mBody.setPosition(mBody.getPosition() + toCyclone(toWorld(mVehicle.centerOfMass - oldCenter)));
        applyMass();

        touchGround(dt, system);
    }

    // Each burning engine pushes up the rocket's axis from where it sits, so lopsided engines would turn it
    void Rocket::pushWithEngines()
    {
        mThrust = 0.0;
        for (PartIndex engine = 0; engine < mVehicle.design.partCount(); engine++)
        {
            if (!isBurning(mVehicle, engine))
            {
                continue;
            }
            const PlacedPart& placed = mVehicle.design.part(engine);
            const double thrust = engineOutput(*placed.pDefinition, mPressure).thrust * mVehicle.throttle;
            mBody.addForceAtBodyPoint(toCyclone(toWorld({ 0.0, thrust, 0.0 })), toCyclone(placed.position - mVehicle.centerOfMass));
            mThrust += thrust;
        }
    }

    // Every outline point at or under the surface becomes a contact, so the rocket rests on whatever touches and
    // tips about it when its weight hangs past the edge
    void Rocket::touchGround(double dt, const SolarSystem& system)
    {
        const double radius = system.body(mReference).radius;
        mContacts.clear();
        for (const Vec3d& part : mVehicle.outline)
        {
            const Vec3d local = part - mVehicle.centerOfMass;
            const Vec3d point = fromCyclone(mBody.getPointInWorldSpace(toCyclone(local)));
            const double distance = point.length();
            const double depth = radius - distance;
            if (distance <= 0.0 || depth < -TOUCH_DISTANCE)
            {
                continue;
            }
            cyclone::Contact contact;
            contact.point = toCyclone(point);
            contact.normal = toCyclone(point / distance);
            contact.penetration = depth;
            contact.body[0] = &mBody;
            contact.body[1] = nullptr;
            contact.friction = GROUND_FRICTION;
            contact.restitution = GROUND_BOUNCE;
            mContacts.push_back(contact);
        }
        mOnGround = !mContacts.empty();
        if (mOnGround)
        {
            mResolver.resolveContacts(mContacts.data(), (unsigned)mContacts.size(), dt);
        }
    }
}
