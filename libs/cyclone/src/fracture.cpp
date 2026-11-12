#include "fracture.h"

#include <algorithm>

using namespace cyclone;

namespace
{
    // The pieces along one axis: [-h, cut] and [cut, h], or the whole span if a piece would be too thin
    int splitAxis(real half, real cut, real minHalfSize, real low[2], real high[2])
    {
        const real minimum = minHalfSize * 2;
        if (cut - -half < minimum || half - cut < minimum)
        {
            low[0] = -half;
            high[0] = half;
            return 1;
        }
        low[0] = -half;
        high[0] = cut;
        low[1] = cut;
        high[1] = half;
        return 2;
    }
}

std::vector<BoxFragment> cyclone::fractureBox(const RigidBody& body, const Vector3& halfSize, const Vector3& localImpactPoint,
                                              real minHalfSize, real burstSpeed)
{
    std::vector<BoxFragment> fragments;
    const Vector3 cut(std::clamp(localImpactPoint.x, -halfSize.x, halfSize.x), std::clamp(localImpactPoint.y, -halfSize.y, halfSize.y),
                      std::clamp(localImpactPoint.z, -halfSize.z, halfSize.z));

    real xLow[2];
    real xHigh[2];
    real yLow[2];
    real yHigh[2];
    real zLow[2];
    real zHigh[2];
    const int nx = splitAxis(halfSize.x, cut.x, minHalfSize, xLow, xHigh);
    const int ny = splitAxis(halfSize.y, cut.y, minHalfSize, yLow, yHigh);
    const int nz = splitAxis(halfSize.z, cut.z, minHalfSize, zLow, zHigh);

    const real totalVolume = 8 * halfSize.x * halfSize.y * halfSize.z;
    const real totalMass = body.hasFiniteMass() ? body.getMass() : 0;
    const Vector3 impactWorld = body.getPointInWorldSpace(cut);

    for (int ix = 0; ix < nx; ix++)
    {
        for (int iy = 0; iy < ny; iy++)
        {
            for (int iz = 0; iz < nz; iz++)
            {
                const Vector3 half((xHigh[ix] - xLow[ix]) * (real)0.5, (yHigh[iy] - yLow[iy]) * (real)0.5,
                                   (zHigh[iz] - zLow[iz]) * (real)0.5);
                const Vector3 localCentre((xHigh[ix] + xLow[ix]) * (real)0.5, (yHigh[iy] + yLow[iy]) * (real)0.5,
                                          (zHigh[iz] + zLow[iz]) * (real)0.5);

                BoxFragment fragment;
                fragment.halfSize = half;
                fragment.position = body.getPointInWorldSpace(localCentre);
                fragment.orientation = body.getOrientation();
                fragment.mass = totalVolume > 0 ? totalMass * (8 * half.x * half.y * half.z) / totalVolume : 0;
                fragment.inertiaTensor.setBlockInertiaTensor(half, fragment.mass);

                // The original's velocity at the piece's centre, so a spinning box flies apart the right way
                fragment.angularVelocity = body.getAngularVelocity();
                fragment.linearVelocity = body.getLinearVelocity() + body.getAngularVelocity() % (fragment.position - body.getPosition());
                if (burstSpeed > 0)
                {
                    Vector3 away = fragment.position - impactWorld;
                    if (away.squareMagnitude() > 0)
                    {
                        fragment.linearVelocity += away.unit() * burstSpeed;
                    }
                }
                fragments.push_back(fragment);
            }
        }
    }
    return fragments;
}
