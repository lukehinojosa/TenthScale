#pragma once

#include <algorithm>
#include <cmath>

#include "flight/SurfaceFrame.h"
#include "shared/SolarSystem.h"

namespace tenth
{
    // A raised concrete hardstand on a body's surface: a flat top with sloped sides running down to the ground all
    // round. Positions are in its frame's axes, x east and z south, measured from the ground at its middle.
    struct LaunchPad
    {
        BodyId body = NO_BODY;
        SurfaceFrame frame;
        double halfWidth = 0.0; // east to west
        double halfLength = 0.0; // north to south
        double height = 0.0;
        double skirt = 0.0; // how far past the top's edges the sloped sides reach the ground

        // How far the pad's surface stands above the ground over a spot; 0 past the foot of its sides
        double heightAt(double x, double z) const
        {
            const double out = outside(x, z);
            if (out <= 0.0)
            {
                return height;
            }
            if (skirt <= 0.0)
            {
                return 0.0;
            }
            return height * std::clamp(1.0 - out / skirt, 0.0, 1.0);
        }

        // Which way the surface faces over a spot, in the frame's axes
        Vec3d normalAt(double x, double z) const
        {
            const double outX = std::max(std::abs(x) - halfWidth, 0.0);
            const double outZ = std::max(std::abs(z) - halfLength, 0.0);
            const double out = std::max(outX, outZ);
            if (out <= 0.0 || skirt <= 0.0 || out >= skirt)
            {
                return { 0.0, 1.0, 0.0 };
            }
            const double slope = height / skirt;
            if (outX >= outZ)
            {
                return Vec3d{ x > 0.0 ? slope : -slope, 1.0, 0.0 }.normalized();
            }
            return Vec3d{ 0.0, 1.0, z > 0.0 ? slope : -slope }.normalized();
        }

    private:
        // How far a spot lies past the edge of the flat top, 0 on it
        double outside(double x, double z) const
        {
            return std::max(std::max(std::abs(x) - halfWidth, 0.0), std::max(std::abs(z) - halfLength, 0.0));
        }
    };
}
