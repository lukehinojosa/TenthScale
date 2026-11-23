#include "flight/Rocket.h"

#include <algorithm>
#include <cmath>

#include "flight/Aerodynamics.h"
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
        constexpr double SAS_RESPONSE = 0.4; // seconds the SAS gives itself to reach the turn rate it wants
        constexpr double SAS_BRAKING = 0.5; // share of its full authority the SAS plans on stopping with
        constexpr double SAS_APPROACH = 1.0; // seconds to close what's left of a small error
        constexpr double SETTLED_RATE = 0.005; // radians per second below which a turn counts as stopped
        constexpr double PATH_SPEED = 2.0; // metres per second from which prograde follows the motion fully
        constexpr double PATH_RATE = 0.5; // radians per second prograde can swing at, when moving that fast or more
        constexpr double CLEAR_SIDEWAYS = 0.1; // share of the motion that's sideways once the orbit's plane is clear
        constexpr double PLANE_RATE = 0.3; // radians per second the orbit's plane can swing at, once it's clear

        // Turns from toward to by at most the given angle. Pointing opposite ways, it turns about hint.
        Vec3d turnToward(const Vec3d& from, const Vec3d& to, double most, const Vec3d& hint)
        {
            const double angle = std::acos(std::clamp(from.dot(to), -1.0, 1.0));
            if (angle <= most)
            {
                return to;
            }
            Vec3d axis = from.cross(to);
            if (axis.lengthSquared() < 1e-18)
            {
                axis = hint;
            }
            axis = axis.normalized();
            const double c = std::cos(most);
            const double s = std::sin(most);
            return (from * c + axis.cross(from) * s + axis * (axis.dot(from) * (1.0 - c))).normalized();
        }

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

        // Standing still the path is up, and the orbit's plane is the one an eastward climb would make, the way D
        // tips the rocket on the pad
        mPath = upright.up;
        mPlane = upright.up.cross(upright.east).normalized();
        updateCourse(1.0e9);
    }

    // Each eases toward what the motion says as the motion makes it clear, and otherwise keeps its last direction,
    // so neither ever jumps
    void Rocket::updateCourse(double dt)
    {
        const Vec3d up = position().normalized();
        const Vec3d along = velocity();
        const double speed = along.length();
        if (speed > 0.0)
        {
            const double sure = std::clamp(speed / PATH_SPEED, 0.0, 1.0);
            mPath = turnToward(mPath, along / speed, sure * PATH_RATE * dt, up.cross(mPath).lengthSquared() > 1e-12 ? up.cross(mPath) : mPlane);
        }

        // Keep the plane square to the path, then ease it toward the plane the motion lies in
        Vec3d plane = mPlane - mPath * mPlane.dot(mPath);
        if (plane.lengthSquared() < 1e-12)
        {
            plane = mPath.cross(up).lengthSquared() > 1e-12 ? mPath.cross(up) : mPath.cross({ 1.0, 0.0, 0.0 });
        }
        plane = plane.normalized();
        const Vec3d square = position().cross(along);
        const double sideways = speed > 0.0 ? square.length() / (position().length() * speed) : 0.0;
        if (sideways > 0.0)
        {
            const double sure = std::clamp(sideways / CLEAR_SIDEWAYS, 0.0, 1.0);
            plane = turnToward(plane, square.normalized(), sure * PLANE_RATE * dt, mPath);
        }
        mPlane = plane;
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

    void Rocket::setSteering(const Vec3d& steering)
    {
        mPilot = { std::clamp(steering.x, -1.0, 1.0), std::clamp(steering.y, -1.0, 1.0), std::clamp(steering.z, -1.0, 1.0) };
    }

    void Rocket::setSas(SasMode mode)
    {
        if (mode == mSas)
        {
            return;
        }
        mSas = mode;
        mHold = toWorld({ 0.0, 1.0, 0.0 });
        mSettling = true;
    }

    bool Rocket::holdDirection(SasMode mode, Vec3d& direction) const
    {
        switch (mode)
        {
        case SasMode::Off:
            return false;
        case SasMode::Stability:
            direction = mHold;
            break;
        case SasMode::Prograde:
            direction = mPath;
            break;
        case SasMode::Retrograde:
            direction = -mPath;
            break;
        case SasMode::Normal:
            direction = mPlane;
            break;
        case SasMode::AntiNormal:
            direction = -mPlane;
            break;
        case SasMode::RadialOut:
            direction = mPath.cross(mPlane).normalized();
            break;
        case SasMode::RadialIn:
            direction = mPlane.cross(mPath).normalized();
            break;
        }
        return true;
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
        piece.mPilot = {};
        piece.mSas = SasMode::Off;
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

    Vec3d Rocket::toLocal(const Vec3d& direction) const
    {
        return fromCyclone(mBody.getDirectionInLocalSpace(toCyclone(direction)));
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
        updateCourse(dt);

        // Gravity pulls every part equally, so it's an acceleration of the whole rocket rather than a force on it
        const Vec3d offset = position();
        const double distanceSquared = offset.lengthSquared();
        Vec3d gravity;
        if (distanceSquared > 0.0)
        {
            gravity = offset.normalized() * (-system.body(mReference).mu / distanceSquared);
        }
        mBody.setAcceleration(toCyclone(gravity));
        const double height = offset.length() - system.body(mReference).radius;
        mPressure = pressureRatio(system.body(mReference), height);
        steerWithSas();
        swivelEngines(dt);
        pushWithEngines();
        turnWithWheels();
        pushWithAir(airDensity(system.body(mReference), height), dt);
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
    void Rocket::pushWithAir(double density, double dt)
    {
        mDrag = 0.0;
        const RocketDesign& design = mVehicle.design;
        if (density <= 0.0 || mVehicle.mass <= 0.0)
        {
            return;
        }
        const Vec3d centre = position();
        const Vec3d spinNow = spin();

        // A push is capped at what would halve the point's own speed in one step, counting both how it moves the
        // rocket and how it turns it. A canopy far up on a light pod would otherwise flip it round between steps.
        const cyclone::Matrix3 inverseInertia = mBody.getInverseInertiaTensorWorld();
        auto apply = [&](const Vec3d& point, auto&& dragAt)
        {
            const Vec3d arm = toWorld(point - mVehicle.centerOfMass);
            const Vec3d worldVelocity = velocity() + spinNow.cross(arm);
            const Vec3d localVelocity = fromCyclone(mBody.getDirectionInLocalSpace(toCyclone(worldVelocity)));
            Vec3d force = toWorld(dragAt(localVelocity));
            if (force.lengthSquared() == 0.0)
            {
                return;
            }
            const Vec3d direction = force.normalized();
            const Vec3d turn = fromCyclone(inverseInertia.transform(toCyclone(arm.cross(direction))));
            const double give = 1.0 / mVehicle.mass + direction.dot(turn.cross(arm)); // speed change per unit impulse
            const double limit = 0.5 * worldVelocity.length() / (give * dt);
            if (force.length() > limit)
            {
                force = direction * limit;
            }
            mBody.addForceAtPoint(toCyclone(force), toCyclone(centre + arm));
            mDrag += force.length();
        };

        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            apply(design.part(i).position,
                  [&](const Vec3d& local)
                  {
                      return partDrag(design, i, local, density);
                  });
            if (!mVehicle.deployed[i])
            {
                continue;
            }
            const double change = (mOnGround ? -dt : dt) / PARACHUTE_OPEN_TIME;
            mVehicle.canopy[i] = std::clamp(mVehicle.canopy[i] + change, 0.0, 1.0);
            const double opening = mVehicle.canopy[i];
            // The risers are flexible, so the canopy pulls on the part's top rather than turning it like a lever
            const PlacedPart& placed = design.part(i);
            apply(placed.position + Vec3d{ 0.0, placed.shape.height * 0.5, 0.0 },
                  [&](const Vec3d& local)
                  {
                      return parachuteDrag(opening, local, density);
                  });
        }
    }

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
            const Vec3d push = thrustDirection(mVehicle, engine) * thrust;
            mBody.addForceAtBodyPoint(toCyclone(toWorld(push)), toCyclone(gimbalPivot(mVehicle, engine) - mVehicle.centerOfMass));
            mThrust += thrust;
        }
    }

    // For each axis it picks the turn rate that would still let it stop in time, and steers toward that rate.
    // Any axis the pilot is steering on is left to them.
    void Rocket::steerWithSas()
    {
        mVehicle.steering = mPilot;
        if (mSas == SasMode::Off || mVehicle.mass <= 0.0)
        {
            return;
        }
        Vec3d target;
        const Vec3d nose = toWorld({ 0.0, 1.0, 0.0 });
        const Vec3d turning = toLocal(spin());
        if (mSas == SasMode::Stability)
        {
            const bool steeringNose = mPilot.x != 0.0 || mPilot.z != 0.0;
            const double noseRate = std::sqrt(turning.x * turning.x + turning.z * turning.z);
            mSettling = steeringNose || (mSettling && noseRate > SETTLED_RATE);
            if (mSettling)
            {
                mHold = nose;
            }
        }
        holdDirection(mSas, target);

        // The turn that would carry the nose onto the target, as an axis in the rocket's own axes scaled by the angle
        const Vec3d local = toLocal(target);
        const double across = std::sqrt(local.x * local.x + local.z * local.z);
        const double angle = std::atan2(across, local.y);
        Vec3d error;
        if (across > 1e-9)
        {
            error = Vec3d{ local.z, 0.0, -local.x } * (angle / across);
        }
        else if (local.y < 0.0)
        {
            error = { angle, 0.0, 0.0 };
        }

        const Vec3d authority = turnAuthority(mVehicle, mPressure);
        const Inertia& inertia = mVehicle.inertia;
        auto command = [&](double wrong, double rate, double torque, double resistance)
        {
            if (torque <= 0.0 || resistance <= 0.0)
            {
                return 0.0;
            }
            const double most = torque / resistance;
            const double wanted = std::min(std::abs(wrong) / SAS_APPROACH, std::sqrt(2.0 * SAS_BRAKING * most * std::abs(wrong)));
            const double target = wrong >= 0.0 ? wanted : -wanted;
            return std::clamp((target - rate) / (most * SAS_RESPONSE), -1.0, 1.0);
        };
        const Vec3d sas = { command(error.x, turning.x, authority.x, inertia.xx), command(0.0, turning.y, authority.y, inertia.yy),
                            command(error.z, turning.z, authority.z, inertia.zz) };
        mVehicle.steering = { mPilot.x != 0.0 ? mPilot.x : sas.x, mPilot.y != 0.0 ? mPilot.y : sas.y, mPilot.z != 0.0 ? mPilot.z : sas.z };
    }

    // An engine below the center of mass leans opposite to one above it to turn the rocket the same way
    void Rocket::swivelEngines(double dt)
    {
        const Vec3d& steering = mVehicle.steering;
        for (PartIndex engine = 0; engine < mVehicle.design.partCount(); engine++)
        {
            const PartDefinition& definition = *mVehicle.design.part(engine).pDefinition;
            if (!definition.isEngine())
            {
                continue;
            }
            Vec3d target;
            if (isBurning(mVehicle, engine) && mVehicle.throttle > 0.0)
            {
                const double side = gimbalPivot(mVehicle, engine).y < mVehicle.centerOfMass.y ? -1.0 : 1.0;
                target = Vec3d{ -steering.z, 0.0, steering.x } * (side * definition.gimbalRange);
                if (target.length() > definition.gimbalRange)
                {
                    target = target.normalized() * definition.gimbalRange;
                }
            }
            Vec3d& lean = mVehicle.gimbal[engine];
            const Vec3d change = target - lean;
            const double step = GIMBAL_RATE * dt;
            lean = change.length() <= step ? target : lean + change.normalized() * step;
        }
    }

    void Rocket::turnWithWheels()
    {
        const Vec3d torque = mVehicle.steering * wheelTorque(mVehicle);
        if (torque.lengthSquared() > 0.0)
        {
            mBody.addTorque(toCyclone(toWorld(torque)));
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
