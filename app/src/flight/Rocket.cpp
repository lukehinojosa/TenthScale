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
        constexpr double PIECE_FRICTION = 0.5;
        constexpr double PIECE_BOUNCE = 0.2;
        constexpr double NOSE_CONE_CHARGE = 0.4; // how big a nose cone's pop looks next to a decoupler's

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
        mClimbRate = 0.0;
        mDropped.clear();
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
        const int stage = mVehicle.nextStage;
        const std::vector<PartIndex> fired = tenth::fireNextStage(mVehicle);
        bool opensParachute = false;
        for (PartIndex i : fired)
        {
            opensParachute = opensParachute || mVehicle.design.part(i).pDefinition->hasParachute;
        }
        if (opensParachute)
        {
            popNoseCones();
        }

        // Each split renumbers the parts, so this stage's decouplers are looked up afresh after every one. One riding
        // on a piece another has already dropped goes with it unfired.
        while (true)
        {
            PartIndex decoupler = NO_PART;
            for (PartIndex i : mVehicle.design.partsInStage(stage))
            {
                if (mVehicle.design.part(i).pDefinition->separates && i != 0)
                {
                    decoupler = i;
                    break;
                }
            }
            if (decoupler == NO_PART)
            {
                break;
            }
            splitOff(decoupler, mVehicle.design.part(decoupler).position, DECOUPLER_SPEED, 1.0);
        }
        return fired;
    }

    std::vector<Separation> Rocket::takeSeparations()
    {
        std::vector<Separation> separations = std::move(mSeparations);
        mSeparations.clear();
        return separations;
    }

    std::vector<Rocket> Rocket::takeDropped()
    {
        std::vector<Rocket> dropped = std::move(mDropped);
        mDropped.clear();
        return dropped;
    }

    void Rocket::popNoseCones()
    {
        while (true)
        {
            PartIndex cone = NO_PART;
            for (PartIndex i = 1; i < mVehicle.design.partCount(); i++)
            {
                if (mVehicle.design.part(i).pDefinition->popsOffAtPeak)
                {
                    cone = i;
                    break;
                }
            }
            if (cone == NO_PART)
            {
                return;
            }
            // The parachute packed under the cone shoves it off from where the two meet
            const PlacedPart& placed = mVehicle.design.part(cone);
            splitOff(cone, placed.position - Vec3d{ 0.0, placed.shape.height * 0.5, 0.0 }, NOSE_CONE_SPEED, NOSE_CONE_CHARGE);
        }
    }

    // Both halves carry on with the motion they had as one, spin included. Then the push kicks them apart from where
    // it sits, equally and oppositely, so their total momentum doesn't change.
    void Rocket::splitOff(PartIndex index, const Vec3d& pushFrom, double speed, double size)
    {
        const Vec3d centre = position();
        const Vec3d oldCenterOfMass = mVehicle.centerOfMass;
        const Vec3d spinNow = spin();
        const Vec3d site = centre + toWorld(pushFrom - oldCenterOfMass);
        const Vec3d jointVelocity = velocity() + spinNow.cross(site - centre);
        Vec3d rootOffset;
        Vehicle branch = detachBranch(mVehicle, index, rootOffset);

        // Whatever's let go of has nothing left to steer or throttle it, so its engines go quiet
        branch.lit.assign(branch.lit.size(), false);

        Rocket piece = *this;
        piece.mDropped.clear();
        piece.mSeparations.clear();
        piece.setVehicle(branch);
        applyMass();
        const Vec3d pieceArm = toWorld(rootOffset + branch.centerOfMass - oldCenterOfMass);
        const Vec3d keptArm = toWorld(mVehicle.centerOfMass - oldCenterOfMass);
        piece.mBody.setPosition(toCyclone(centre + pieceArm));
        mBody.setPosition(toCyclone(centre + keptArm));
        piece.mBody.setLinearVelocity(toCyclone(velocity() + spinNow.cross(pieceArm)));
        mBody.setLinearVelocity(toCyclone(velocity() + spinNow.cross(keptArm)));

        // The impulse that would part two free bodies at this speed if it pushed through both their centers
        const Vec3d apart = (keptArm - pieceArm).normalized();
        const double total = branch.mass + mVehicle.mass;
        if (total > 0.0)
        {
            const Vec3d impulse = apart * (speed * branch.mass * mVehicle.mass / total);
            kick(impulse, site);
            piece.kick(-impulse, site);
        }
        mSeparations.push_back({ site, jointVelocity, apart, size, keptArm });
        mDropped.push_back(std::move(piece));
    }

    void Rocket::kick(const Vec3d& impulse, const Vec3d& site)
    {
        if (mVehicle.mass <= 0.0)
        {
            return;
        }
        mBody.setLinearVelocity(toCyclone(velocity() + impulse / mVehicle.mass));
        const Vec3d twist = (site - position()).cross(impulse);
        const cyclone::Vector3 turn = mBody.getInverseInertiaTensorWorld().transform(toCyclone(twist));
        mBody.setAngularVelocity(mBody.getAngularVelocity() + turn);
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

        // Once flying, the moment it stops climbing is the top of the climb
        const double climbRate = velocity().dot(position().normalized());
        if (mVehicle.nextStage > 0 && !mOnGround && mClimbRate > 0.0 && climbRate <= 0.0)
        {
            popNoseCones();
        }
        mClimbRate = climbRate;
    }

    double Rocket::reach() const
    {
        double furthest = 0.0;
        for (const Vec3d& point : mVehicle.outline)
        {
            furthest = std::max(furthest, (point - mVehicle.centerOfMass).length());
        }
        return furthest;
    }

    void Rocket::addContactsInside(Rocket& other, std::vector<cyclone::Contact>& contacts)
    {
        const RocketDesign& otherDesign = other.mVehicle.design;
        for (const Vec3d& part : mVehicle.outline)
        {
            const cyclone::Vector3 world = mBody.getPointInWorldSpace(toCyclone(part - mVehicle.centerOfMass));
            const Vec3d inOther = fromCyclone(other.mBody.getPointInLocalSpace(world)) + other.mVehicle.centerOfMass;

            // The shallowest way out of whichever of the other's parts it's in
            double best = 0.0;
            Vec3d outward;
            for (PartIndex i = 0; i < otherDesign.partCount(); i++)
            {
                double depth = 0.0;
                Vec3d out;
                if (insidePart(otherDesign, i, inOther, depth, out) && (best <= 0.0 || depth < best))
                {
                    best = depth;
                    outward = out;
                }
            }
            if (best <= 0.0)
            {
                continue;
            }

            // The normal points out of the other rocket, toward this one, as Cyclone expects for body[0]
            cyclone::Contact contact;
            contact.point = world;
            contact.normal = toCyclone(other.toWorld(outward));
            contact.penetration = best;
            contact.body[0] = &mBody;
            contact.body[1] = &other.mBody;
            contact.friction = PIECE_FRICTION;
            contact.restitution = PIECE_BOUNCE;
            contacts.push_back(contact);
        }
    }

    void collide(Rocket& a, Rocket& b, double dt)
    {
        if (a.mReference != b.mReference || a.mVehicle.mass <= 0.0 || b.mVehicle.mass <= 0.0)
        {
            return;
        }
        if ((a.position() - b.position()).length() > a.reach() + b.reach())
        {
            return;
        }
        std::vector<cyclone::Contact> contacts;
        a.addContactsInside(b, contacts);
        b.addContactsInside(a, contacts);
        if (!contacts.empty())
        {
            a.mResolver.resolveContacts(contacts.data(), (unsigned)contacts.size(), dt);
        }
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
