#pragma once

#include <math.h>

#include <cmath>

#include <cyclone/precision.h>

// Cyclone's ideas brought down to two dimensions: x to the right, y up, and angles counter-clockwise in radians.
// It shares Cyclone's real, so CYCLONE_DOUBLE_PRECISION switches both.
namespace cyclone2d
{
    using cyclone::real;

    class Vector2
    {
    public:
        real x = 0;
        real y = 0;

        Vector2() = default;
        Vector2(real x, real y) : x(x), y(y) {}

        const static Vector2 GRAVITY;
        const static Vector2 UP;
        const static Vector2 RIGHT;

        Vector2 operator+(const Vector2& v) const { return Vector2(x + v.x, y + v.y); }
        Vector2 operator-(const Vector2& v) const { return Vector2(x - v.x, y - v.y); }
        Vector2 operator-() const { return Vector2(-x, -y); }
        Vector2 operator*(real value) const { return Vector2(x * value, y * value); }

        void operator+=(const Vector2& v)
        {
            x += v.x;
            y += v.y;
        }

        void operator-=(const Vector2& v)
        {
            x -= v.x;
            y -= v.y;
        }

        void operator*=(real value)
        {
            x *= value;
            y *= value;
        }

        bool operator==(const Vector2& v) const { return x == v.x && y == v.y; }
        bool operator!=(const Vector2& v) const { return !(*this == v); }

        real dot(const Vector2& v) const { return x * v.x + y * v.y; }

        // The z part of the 3D cross product: positive when v is counter-clockwise from this
        real cross(const Vector2& v) const { return x * v.y - y * v.x; }

        // This vector turned a quarter turn counter-clockwise
        Vector2 perpendicular() const { return Vector2(-y, x); }

        Vector2 componentProduct(const Vector2& v) const { return Vector2(x * v.x, y * v.y); }

        void addScaledVector(const Vector2& v, real scale)
        {
            x += v.x * scale;
            y += v.y * scale;
        }

        real magnitude() const { return real_sqrt(x * x + y * y); }
        real squareMagnitude() const { return x * x + y * y; }

        // Leaves a zero vector alone
        void normalise()
        {
            const real length = magnitude();
            if (length > 0)
            {
                *this *= ((real)1) / length;
            }
        }

        Vector2 unit() const
        {
            Vector2 result = *this;
            result.normalise();
            return result;
        }

        void clear()
        {
            x = 0;
            y = 0;
        }
    };

    inline Vector2 operator*(real value, const Vector2& v)
    {
        return v * value;
    }

    // The velocity of a point at r on something spinning at w radians per second
    inline Vector2 cross(real w, const Vector2& r)
    {
        return Vector2(-w * r.y, w * r.x);
    }

    // A turn stored as its cosine and sine, so rotating a vector needs no trigonometry
    class Rotation
    {
    public:
        real c = 1;
        real s = 0;

        Rotation() = default;
        explicit Rotation(real angle) { set(angle); }

        void set(real angle)
        {
            c = real_cos(angle);
            s = real_sin(angle);
        }

        real getAngle() const { return std::atan2(s, c); }

        Vector2 rotate(const Vector2& v) const { return Vector2(c * v.x - s * v.y, s * v.x + c * v.y); }
        Vector2 rotateInverse(const Vector2& v) const { return Vector2(c * v.x + s * v.y, -s * v.x + c * v.y); }

        // The rotated x and y axes
        Vector2 getXAxis() const { return Vector2(c, s); }
        Vector2 getYAxis() const { return Vector2(-s, c); }

        // This turn followed by other
        Rotation operator*(const Rotation& other) const
        {
            Rotation result;
            result.c = c * other.c - s * other.s;
            result.s = s * other.c + c * other.s;
            return result;
        }
    };

    // A position and a rotation: the 2D stand-in for Cyclone's Matrix4
    class Transform
    {
    public:
        Vector2 position;
        Rotation rotation;

        Transform() = default;
        Transform(const Vector2& position, real angle) : position(position), rotation(angle) {}
        Transform(const Vector2& position, const Rotation& rotation) : position(position), rotation(rotation) {}

        Vector2 transform(const Vector2& localPoint) const { return rotation.rotate(localPoint) + position; }
        Vector2 transformInverse(const Vector2& worldPoint) const { return rotation.rotateInverse(worldPoint - position); }
        Vector2 transformDirection(const Vector2& localDirection) const { return rotation.rotate(localDirection); }
        Vector2 transformInverseDirection(const Vector2& worldDirection) const { return rotation.rotateInverse(worldDirection); }

        // A transform given in this one's space, carried out to the space this one lives in
        Transform operator*(const Transform& local) const { return Transform(transform(local.position), rotation * local.rotation); }
    };
}
