#pragma once

#include <cmath>

namespace ai
{
    // Minimal 2D vector so the AI code has no math-library dependency
    struct Vec2
    {
        float x = 0.0f;
        float y = 0.0f;

        Vec2 operator+(const Vec2& o) const { return { x + o.x, y + o.y }; }
        Vec2 operator-(const Vec2& o) const { return { x - o.x, y - o.y }; }
        Vec2 operator*(float s) const { return { x * s, y * s }; }
        Vec2 operator/(float s) const { return { x / s, y / s }; }
        Vec2& operator+=(const Vec2& o)
        {
            x += o.x;
            y += o.y;
            return *this;
        }
        Vec2& operator-=(const Vec2& o)
        {
            x -= o.x;
            y -= o.y;
            return *this;
        }

        float length() const { return std::sqrt(x * x + y * y); }
        float lengthSquared() const { return x * x + y * y; }

        Vec2 normalized() const
        {
            float len = length();
            if (len <= 1e-6f)
            {
                return {};
            }
            return { x / len, y / len };
        }

        // Shortens the vector to maxLength if it is longer
        Vec2 clamped(float maxLength) const
        {
            float len = length();
            if (len <= maxLength || len <= 1e-6f)
            {
                return *this;
            }
            return *this * (maxLength / len);
        }
    };
}
