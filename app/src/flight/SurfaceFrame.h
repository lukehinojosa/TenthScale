#pragma once

#include "shared/Vec3d.h"

namespace tenth
{
    // Axes standing on a body's surface: x east, y straight up, z south. Turns body-centred positions into
    // ones relative to the spot, so the scene near it can be drawn with small numbers and +Y up.
    struct SurfaceFrame
    {
        Vec3d origin;
        Vec3d east;
        Vec3d up;
        Vec3d south;

        // The spot on the surface beneath position, for a body centred at the origin
        static SurfaceFrame below(const Vec3d& position, double radius)
        {
            SurfaceFrame frame;
            frame.up = position.normalized();
            frame.origin = frame.up * radius;

            // North is the body's +Y, flattened onto the ground; at a pole any sideways direction will do
            Vec3d north = Vec3d{ 0.0, 1.0, 0.0 } - frame.up * frame.up.y;
            if (north.lengthSquared() < 1e-12)
            {
                north = Vec3d{ 1.0, 0.0, 0.0 } - frame.up * frame.up.x;
            }
            north = north.normalized();
            frame.east = north.cross(frame.up);
            frame.south = frame.east.cross(frame.up);
            return frame;
        }

        // A direction or offset in body axes, turned into this frame's axes
        Vec3d rotate(const Vec3d& v) const { return { v.dot(east), v.dot(up), v.dot(south) }; }

        Vec3d toLocal(const Vec3d& position) const { return rotate(position - origin); }
    };
}
