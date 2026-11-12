#pragma once

#include <raylib.h>

#include <cyclone/cyclone.h>

// Casts keep these working when Cyclone is built with double precision
namespace engine
{
    inline Vector3 toRaylib(const cyclone::Vector3& v)
    {
        return Vector3{ static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z) };
    }

    inline cyclone::Vector3 toCyclone(const Vector3& v)
    {
        return cyclone::Vector3(static_cast<cyclone::real>(v.x), static_cast<cyclone::real>(v.y), static_cast<cyclone::real>(v.z));
    }

    inline Quaternion toRaylib(const cyclone::Quaternion& q)
    {
        return Quaternion{ static_cast<float>(q.i), static_cast<float>(q.j), static_cast<float>(q.k), static_cast<float>(q.r) };
    }

    inline cyclone::Quaternion toCyclone(const Quaternion& q)
    {
        return cyclone::Quaternion(q.w, q.x, q.y, q.z);
    }

    // Cyclone stores the top three rows row by row; raylib names elements column by column
    inline Matrix toRaylib(const cyclone::Matrix4& m)
    {
        Matrix out{};
        out.m0 = static_cast<float>(m.data[0]);
        out.m4 = static_cast<float>(m.data[1]);
        out.m8 = static_cast<float>(m.data[2]);
        out.m12 = static_cast<float>(m.data[3]);
        out.m1 = static_cast<float>(m.data[4]);
        out.m5 = static_cast<float>(m.data[5]);
        out.m9 = static_cast<float>(m.data[6]);
        out.m13 = static_cast<float>(m.data[7]);
        out.m2 = static_cast<float>(m.data[8]);
        out.m6 = static_cast<float>(m.data[9]);
        out.m10 = static_cast<float>(m.data[10]);
        out.m14 = static_cast<float>(m.data[11]);
        out.m15 = 1.0f;
        return out;
    }

    inline cyclone::Matrix4 toCyclone(const Matrix& m)
    {
        cyclone::Matrix4 out;
        out.data[0] = m.m0;
        out.data[1] = m.m4;
        out.data[2] = m.m8;
        out.data[3] = m.m12;
        out.data[4] = m.m1;
        out.data[5] = m.m5;
        out.data[6] = m.m9;
        out.data[7] = m.m13;
        out.data[8] = m.m2;
        out.data[9] = m.m6;
        out.data[10] = m.m10;
        out.data[11] = m.m14;
        return out;
    }
}
