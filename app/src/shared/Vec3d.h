#pragma once

#include <cmath>

namespace tenth
{
    // A double-precision vector for positions and velocities across the Solar System, where floats run out of precision
    struct Vec3d
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        constexpr Vec3d operator+(const Vec3d& other) const { return { x + other.x, y + other.y, z + other.z }; }

        constexpr Vec3d operator-(const Vec3d& other) const { return { x - other.x, y - other.y, z - other.z }; }

        constexpr Vec3d operator-() const { return { -x, -y, -z }; }

        constexpr Vec3d operator*(double scale) const { return { x * scale, y * scale, z * scale }; }

        constexpr Vec3d operator/(double scale) const { return { x / scale, y / scale, z / scale }; }

        constexpr Vec3d& operator+=(const Vec3d& other)
        {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        constexpr Vec3d& operator-=(const Vec3d& other)
        {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }

        constexpr bool operator==(const Vec3d& other) const = default;

        constexpr double dot(const Vec3d& other) const { return x * other.x + y * other.y + z * other.z; }

        constexpr Vec3d cross(const Vec3d& other) const
        {
            return { y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x };
        }

        constexpr double lengthSquared() const { return dot(*this); }

        double length() const { return std::sqrt(lengthSquared()); }

        // A zero vector stays zero
        Vec3d normalized() const
        {
            const double len = length();
            if (len == 0.0)
            {
                return {};
            }
            return *this / len;
        }
    };

    constexpr Vec3d operator*(double scale, const Vec3d& v)
    {
        return v * scale;
    }
}
