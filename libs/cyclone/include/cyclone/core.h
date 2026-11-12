/*
 * Interface file for core components and functions.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

// Cyclone, from Ian Millington's Game Physics Engine Development: particles, rigid bodies, collisions and
// contacts. World and ParticleWorld, in their own headers, run a whole simulation each frame.

#include <math.h>

// Core maths types and helpers: vectors, quaternions and matrices.
#ifndef CYCLONE_CORE_H
#define CYCLONE_CORE_H

#include "precision.h"
// Everything in Cyclone lives in this namespace.
namespace cyclone
{
    // A vector in three dimensions, with the basic maths inline.
    class Vector3
    {
    public:
        /** Holds the value along the x axis. */
        real x;

        /** Holds the value along the y axis. */
        real y;

        /** Holds the value along the z axis. */
        real z;

        // The original Cyclone pads this to four values for alignment. Nothing relies on that, and dropping it makes
        // every Vector3, and everything holding one, a quarter smaller.

    public:
        /** The default constructor creates a zero vector. */
        Vector3() : x(0), y(0), z(0) {}

        // Creates a vector with the given components.
        Vector3(const real x, const real y, const real z) : x(x), y(y), z(z) {}

        const static Vector3 GRAVITY;
        const static Vector3 UP;
        const static Vector3 RIGHT;
        const static Vector3 FORWARD; //Added by Eric

        real operator[](unsigned i) const
        {
            if (i == 0)
            {
                return x;
            }
            if (i == 1)
            {
                return y;
            }
            return z;
        }

        real& operator[](unsigned i)
        {
            if (i == 0)
            {
                return x;
            }
            if (i == 1)
            {
                return y;
            }
            return z;
        }

        /** Adds the given vector to this. */
        void operator+=(const Vector3& v)
        {
            x += v.x;
            y += v.y;
            z += v.z;
        }

        // The sum of this vector and the given one.
        Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }

        /** Subtracts the given vector from this. */
        void operator-=(const Vector3& v)
        {
            x -= v.x;
            y -= v.y;
            z -= v.z;
        }

        // This vector minus the given one.
        Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }

        /** Multiplies this vector by the given scalar. */
        void operator*=(const real value)
        {
            x *= value;
            y *= value;
            z *= value;
        }

        /** Returns a copy of this vector scaled the given value. */
        Vector3 operator*(const real value) const { return Vector3(x * value, y * value, z * value); }

        // The component-wise product with the given vector.
        Vector3 componentProduct(const Vector3& vector) const { return Vector3(x * vector.x, y * vector.y, z * vector.z); }

        // Multiplies this vector by the given one, component by component.
        void componentProductUpdate(const Vector3& vector)
        {
            x *= vector.x;
            y *= vector.y;
            z *= vector.z;
        }

        // The vector product with the given vector.
        Vector3 vectorProduct(const Vector3& vector) const
        {
            return Vector3(y * vector.z - z * vector.y, z * vector.x - x * vector.z, x * vector.y - y * vector.x);
        }

        // Sets this vector to its vector product with the given one.
        void operator%=(const Vector3& vector) { *this = vectorProduct(vector); }

        // The vector product with the given vector.
        Vector3 operator%(const Vector3& vector) const
        {
            return Vector3(y * vector.z - z * vector.y, z * vector.x - x * vector.z, x * vector.y - y * vector.x);
        }

        // The scalar product with the given vector.
        real scalarProduct(const Vector3& vector) const { return x * vector.x + y * vector.y + z * vector.z; }

        // The scalar product of two vectors.
        static real scalarProduct(const Vector3& vector1, const Vector3& vector2)
        {
            return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
        }

        // The scalar product with the given vector.
        real operator*(const Vector3& vector) const { return x * vector.x + y * vector.y + z * vector.z; }

        // Adds the given vector, scaled by the given amount.
        void addScaledVector(const Vector3& vector, real scale)
        {
            x += vector.x * scale;
            y += vector.y * scale;
            z += vector.z * scale;
        }

        /** Gets the magnitude of this vector. */
        real magnitude() const { return real_sqrt(x * x + y * y + z * z); }

        /** Gets the squared magnitude of this vector. */
        real squareMagnitude() const { return x * x + y * y + z * z; }

        /** Limits the size of the vector to the given maximum. */
        void trim(real size)
        {
            if (squareMagnitude() > size * size)
            {
                normalise();
                x *= size;
                y *= size;
                z *= size;
            }
        }

        /** Turns a non-zero vector into a vector of unit length. */
        void normalise()
        {
            real l = magnitude();
            if (l > 0)
            {
                (*this) *= ((real)1) / l;
            }
        }

        /** Returns the normalised version of a vector. */
        Vector3 unit() const
        {
            Vector3 result = *this;
            result.normalise();
            return result;
        }

        /** Checks if the two vectors have identical components. */
        bool operator==(const Vector3& other) const { return x == other.x && y == other.y && z == other.z; }

        /** Checks if the two vectors have non-identical components. */
        bool operator!=(const Vector3& other) const { return !(*this == other); }

        // True when every component is less than the other's. It isn't a full ordering, so false doesn't mean the
        // other way round is true.
        bool operator<(const Vector3& other) const { return x < other.x && y < other.y && z < other.z; }

        // True when every component is greater than the other's. It isn't a full ordering.
        bool operator>(const Vector3& other) const { return x > other.x && y > other.y && z > other.z; }

        // True when every component is at most the other's. It isn't a full ordering.
        bool operator<=(const Vector3& other) const { return x <= other.x && y <= other.y && z <= other.z; }

        // True when every component is at least the other's. It isn't a full ordering.
        bool operator>=(const Vector3& other) const { return x >= other.x && y >= other.y && z >= other.z; }

        /** Zero all the components of the vector. */
        void clear() { x = y = z = 0; }

        /** Flips all the components of the vector. */
        void invert()
        {
            x = -x;
            y = -y;
            z = -z;
        }
    };

    // An orientation, only valid as a rotation at unit length. Angular velocity and acceleration are plain
    // vectors.
    class Quaternion
    {
    public:
        union
        {
            struct
            {
                // The real component.
                real r;

                // The first complex component.
                real i;

                // The second complex component.
                real j;

                // The third complex component.
                real k;
            };

            // The components as an array.
            real data[4];
        };

        // ... other Quaternion code as before ...

        // Creates a quaternion with no rotation.
        Quaternion() : r(1), i(0), j(0), k(0) {}

        // Creates a quaternion from its components, without normalising it; call normalise() for a valid rotation.
        Quaternion(const real r, const real i, const real j, const real k) : r(r), i(i), j(j), k(k) {}

        // Scales the quaternion to unit length, making it a valid orientation.
        void normalise()
        {
            real d = r * r + i * i + j * j + k * k;

            // Check for zero length quaternion, and use the no-rotation
            // quaternion in that case.
            if (d < real_epsilon)
            {
                r = 1;
                return;
            }

            d = ((real)1.0) / real_sqrt(d);
            r *= d;
            i *= d;
            j *= d;
            k *= d;
        }

        // Multiplies this quaternion by the given one.
        void operator*=(const Quaternion& multiplier)
        {
            Quaternion q = *this;
            r = q.r * multiplier.r - q.i * multiplier.i - q.j * multiplier.j - q.k * multiplier.k;
            i = q.r * multiplier.i + q.i * multiplier.r + q.j * multiplier.k - q.k * multiplier.j;
            j = q.r * multiplier.j + q.j * multiplier.r + q.k * multiplier.i - q.i * multiplier.k;
            k = q.r * multiplier.k + q.k * multiplier.r + q.i * multiplier.j - q.j * multiplier.i;
        }

        // Adds the given vector, scaled by the given amount; used to turn an orientation by a rotation over time.
        void addScaledVector(const Vector3& vector, real scale)
        {
            Quaternion q(0, vector.x * scale, vector.y * scale, vector.z * scale);
            q *= *this;
            r += q.r * ((real)0.5);
            i += q.i * ((real)0.5);
            j += q.j * ((real)0.5);
            k += q.k * ((real)0.5);
        }

        void rotateByVector(const Vector3& vector)
        {
            Quaternion q(0, vector.x, vector.y, vector.z);
            (*this) *= q;
        }
    };

    // A transform: a rotation and a position, with the bottom row of the full matrix left implied.
    class Matrix4
    {
    public:
        // The matrix values as an array.
        real data[12];

        // ... Other Matrix4 code as before ...

        // Creates an identity matrix.
        Matrix4()
        {
            data[1] = data[2] = data[3] = data[4] = data[6] = data[7] = data[8] = data[9] = data[11] = 0;
            data[0] = data[5] = data[10] = 1;
        }

        // Sets the matrix to a diagonal matrix with the given values.
        void setDiagonal(real a, real b, real c)
        {
            data[0] = a;
            data[5] = b;
            data[10] = c;
        }

        // This matrix multiplied by the given one.
        Matrix4 operator*(const Matrix4& o) const
        {
            Matrix4 result;
            result.data[0] = (o.data[0] * data[0]) + (o.data[4] * data[1]) + (o.data[8] * data[2]);
            result.data[4] = (o.data[0] * data[4]) + (o.data[4] * data[5]) + (o.data[8] * data[6]);
            result.data[8] = (o.data[0] * data[8]) + (o.data[4] * data[9]) + (o.data[8] * data[10]);

            result.data[1] = (o.data[1] * data[0]) + (o.data[5] * data[1]) + (o.data[9] * data[2]);
            result.data[5] = (o.data[1] * data[4]) + (o.data[5] * data[5]) + (o.data[9] * data[6]);
            result.data[9] = (o.data[1] * data[8]) + (o.data[5] * data[9]) + (o.data[9] * data[10]);

            result.data[2] = (o.data[2] * data[0]) + (o.data[6] * data[1]) + (o.data[10] * data[2]);
            result.data[6] = (o.data[2] * data[4]) + (o.data[6] * data[5]) + (o.data[10] * data[6]);
            result.data[10] = (o.data[2] * data[8]) + (o.data[6] * data[9]) + (o.data[10] * data[10]);

            result.data[3] = (o.data[3] * data[0]) + (o.data[7] * data[1]) + (o.data[11] * data[2]) + data[3];
            result.data[7] = (o.data[3] * data[4]) + (o.data[7] * data[5]) + (o.data[11] * data[6]) + data[7];
            result.data[11] = (o.data[3] * data[8]) + (o.data[7] * data[9]) + (o.data[11] * data[10]) + data[11];

            return result;
        }

        // Transforms the given vector by this matrix.
        Vector3 operator*(const Vector3& vector) const
        {
            return Vector3(vector.x * data[0] + vector.y * data[1] + vector.z * data[2] + data[3],

                           vector.x * data[4] + vector.y * data[5] + vector.z * data[6] + data[7],

                           vector.x * data[8] + vector.y * data[9] + vector.z * data[10] + data[11]);
        }

        // Transforms the given vector by this matrix.
        Vector3 transform(const Vector3& vector) const { return (*this) * vector; }

        // The matrix's determinant.
        real getDeterminant() const;

        // Sets this matrix to the inverse of the given one.
        void setInverse(const Matrix4& m);

        /** Returns a new matrix containing the inverse of this matrix. */
        Matrix4 inverse() const
        {
            Matrix4 result;
            result.setInverse(*this);
            return result;
        }

        // Inverts the matrix.
        void invert() { setInverse(*this); }

        // Transforms a direction by this matrix, leaving out the translation.
        Vector3 transformDirection(const Vector3& vector) const
        {
            return Vector3(vector.x * data[0] + vector.y * data[1] + vector.z * data[2],

                           vector.x * data[4] + vector.y * data[5] + vector.z * data[6],

                           vector.x * data[8] + vector.y * data[9] + vector.z * data[10]);
        }

        // Transforms a direction by the inverse of this matrix, leaving out the translation. Only correct for
        // transforms without scale or shear.
        Vector3 transformInverseDirection(const Vector3& vector) const
        {
            return Vector3(vector.x * data[0] + vector.y * data[4] + vector.z * data[8],

                           vector.x * data[1] + vector.y * data[5] + vector.z * data[9],

                           vector.x * data[2] + vector.y * data[6] + vector.z * data[10]);
        }

        // Transforms a vector by the inverse of this matrix. Only correct for transforms without scale or shear.
        Vector3 transformInverse(const Vector3& vector) const
        {
            Vector3 tmp = vector;
            tmp.x -= data[3];
            tmp.y -= data[7];
            tmp.z -= data[11];
            return Vector3(tmp.x * data[0] + tmp.y * data[4] + tmp.z * data[8],

                           tmp.x * data[1] + tmp.y * data[5] + tmp.z * data[9],

                           tmp.x * data[2] + tmp.y * data[6] + tmp.z * data[10]);
        }

        // One column of the matrix; column 3 is the position.
        Vector3 getAxisVector(int i) const { return Vector3(data[i], data[i + 4], data[i + 8]); }

        // Sets this matrix from an orientation and a position.
        void setOrientationAndPos(const Quaternion& q, const Vector3& pos)
        {
            data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
            data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
            data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
            data[3] = pos.x;

            data[4] = 2 * q.i * q.j - 2 * q.k * q.r;
            data[5] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
            data[6] = 2 * q.j * q.k + 2 * q.i * q.r;
            data[7] = pos.y;

            data[8] = 2 * q.i * q.k + 2 * q.j * q.r;
            data[9] = 2 * q.j * q.k - 2 * q.i * q.r;
            data[10] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
            data[11] = pos.z;
        }

        // Copies the matrix into an OpenGL-style array, which stores columns first.
        void fillGLArray(float array[16]) const
        {
            array[0] = (float)data[0];
            array[1] = (float)data[4];
            array[2] = (float)data[8];
            array[3] = (float)0;

            array[4] = (float)data[1];
            array[5] = (float)data[5];
            array[6] = (float)data[9];
            array[7] = (float)0;

            array[8] = (float)data[2];
            array[9] = (float)data[6];
            array[10] = (float)data[10];
            array[11] = (float)0;

            array[12] = (float)data[3];
            array[13] = (float)data[7];
            array[14] = (float)data[11];
            array[15] = (float)1;
        }
    };

    // A 3 by 3 matrix, usually an inertia tensor.
    class Matrix3
    {
    public:
        // The matrix values as an array.
        real data[9];

        // ... Other Matrix3 code as before ...

        // Creates a zero matrix.
        Matrix3() { data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = data[7] = data[8] = 0; }

        // Creates a matrix with the three vectors as its columns.
        Matrix3(const Vector3& compOne, const Vector3& compTwo, const Vector3& compThree) { setComponents(compOne, compTwo, compThree); }

        // Creates a matrix with the given values.
        Matrix3(real c0, real c1, real c2, real c3, real c4, real c5, real c6, real c7, real c8)
        {
            data[0] = c0;
            data[1] = c1;
            data[2] = c2;
            data[3] = c3;
            data[4] = c4;
            data[5] = c5;
            data[6] = c6;
            data[7] = c7;
            data[8] = c8;
        }

        // Sets the matrix to a diagonal matrix with the given values.
        void setDiagonal(real a, real b, real c) { setInertiaTensorCoeffs(a, b, c); }

        // Sets the matrix from inertia tensor values.
        void setInertiaTensorCoeffs(real ix, real iy, real iz, real ixy = 0, real ixz = 0, real iyz = 0)
        {
            data[0] = ix;
            data[1] = data[3] = -ixy;
            data[2] = data[6] = -ixz;
            data[4] = iy;
            data[5] = data[7] = -iyz;
            data[8] = iz;
        }

        // Sets the inertia tensor of a solid box with the given half sizes and mass, aligned with the body.
        void setBlockInertiaTensor(const Vector3& halfSizes, real mass)
        {
            Vector3 squares = halfSizes.componentProduct(halfSizes);
            setInertiaTensorCoeffs(0.3f * mass * (squares.y + squares.z), 0.3f * mass * (squares.x + squares.z),
                                   0.3f * mass * (squares.x + squares.y));
        }

        // Sets the matrix so multiplying by it gives the vector product with the given vector.
        void setSkewSymmetric(const Vector3 vector)
        {
            data[0] = data[4] = data[8] = 0;
            data[1] = -vector.z;
            data[2] = vector.y;
            data[3] = vector.z;
            data[5] = -vector.x;
            data[6] = -vector.y;
            data[7] = vector.x;
        }

        // Sets the matrix's three columns from the given vectors.
        void setComponents(const Vector3& compOne, const Vector3& compTwo, const Vector3& compThree)
        {
            data[0] = compOne.x;
            data[1] = compTwo.x;
            data[2] = compThree.x;
            data[3] = compOne.y;
            data[4] = compTwo.y;
            data[5] = compThree.y;
            data[6] = compOne.z;
            data[7] = compTwo.z;
            data[8] = compThree.z;
        }

        // Transforms the given vector by this matrix.
        Vector3 operator*(const Vector3& vector) const
        {
            return Vector3(vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
                           vector.x * data[3] + vector.y * data[4] + vector.z * data[5],
                           vector.x * data[6] + vector.y * data[7] + vector.z * data[8]);
        }

        // Transforms the given vector by this matrix.
        Vector3 transform(const Vector3& vector) const { return (*this) * vector; }

        // Transforms the given vector by this matrix's transpose.
        Vector3 transformTranspose(const Vector3& vector) const
        {
            return Vector3(vector.x * data[0] + vector.y * data[3] + vector.z * data[6],
                           vector.x * data[1] + vector.y * data[4] + vector.z * data[7],
                           vector.x * data[2] + vector.y * data[5] + vector.z * data[8]);
        }

        // One row of the matrix.
        Vector3 getRowVector(int i) const { return Vector3(data[i * 3], data[i * 3 + 1], data[i * 3 + 2]); }

        // One column of the matrix.
        Vector3 getAxisVector(int i) const { return Vector3(data[i], data[i + 3], data[i + 6]); }

        // Sets this matrix to the inverse of the given one.
        void setInverse(const Matrix3& m)
        {
            real t4 = m.data[0] * m.data[4];
            real t6 = m.data[0] * m.data[5];
            real t8 = m.data[1] * m.data[3];
            real t10 = m.data[2] * m.data[3];
            real t12 = m.data[1] * m.data[6];
            real t14 = m.data[2] * m.data[6];

            // Calculate the determinant
            real t16 = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] + t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

            // Make sure the determinant is non-zero.
            if (t16 == (real)0.0f)
            {
                return;
            }
            real t17 = 1 / t16;

            data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * t17;
            data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * t17;
            data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * t17;
            data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * t17;
            data[4] = (m.data[0] * m.data[8] - t14) * t17;
            data[5] = -(t6 - t10) * t17;
            data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * t17;
            data[7] = -(m.data[0] * m.data[7] - t12) * t17;
            data[8] = (t4 - t8) * t17;
        }

        /** Returns a new matrix containing the inverse of this matrix. */
        Matrix3 inverse() const
        {
            Matrix3 result;
            result.setInverse(*this);
            return result;
        }

        // Inverts the matrix.
        void invert() { setInverse(*this); }

        // Sets this matrix to the transpose of the given one.
        void setTranspose(const Matrix3& m)
        {
            data[0] = m.data[0];
            data[1] = m.data[3];
            data[2] = m.data[6];
            data[3] = m.data[1];
            data[4] = m.data[4];
            data[5] = m.data[7];
            data[6] = m.data[2];
            data[7] = m.data[5];
            data[8] = m.data[8];
        }

        /** Returns a new matrix containing the transpose of this matrix. */
        Matrix3 transpose() const
        {
            Matrix3 result;
            result.setTranspose(*this);
            return result;
        }

        // This matrix multiplied by the given one.
        Matrix3 operator*(const Matrix3& o) const
        {
            return Matrix3(data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
                           data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
                           data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

                           data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
                           data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
                           data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

                           data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
                           data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
                           data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]);
        }

        // Multiplies this matrix by the given one.
        void operator*=(const Matrix3& o)
        {
            real t1;
            real t2;
            real t3;

            t1 = data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6];
            t2 = data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7];
            t3 = data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8];
            data[0] = t1;
            data[1] = t2;
            data[2] = t3;

            t1 = data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6];
            t2 = data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7];
            t3 = data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8];
            data[3] = t1;
            data[4] = t2;
            data[5] = t3;

            t1 = data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6];
            t2 = data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7];
            t3 = data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8];
            data[6] = t1;
            data[7] = t2;
            data[8] = t3;
        }

        // Multiplies every value by the given number.
        void operator*=(const real scalar)
        {
            data[0] *= scalar;
            data[1] *= scalar;
            data[2] *= scalar;
            data[3] *= scalar;
            data[4] *= scalar;
            data[5] *= scalar;
            data[6] *= scalar;
            data[7] *= scalar;
            data[8] *= scalar;
        }

        // Adds the given matrix to this one, value by value.
        void operator+=(const Matrix3& o)
        {
            data[0] += o.data[0];
            data[1] += o.data[1];
            data[2] += o.data[2];
            data[3] += o.data[3];
            data[4] += o.data[4];
            data[5] += o.data[5];
            data[6] += o.data[6];
            data[7] += o.data[7];
            data[8] += o.data[8];
        }

        // Sets this matrix to the rotation for the given quaternion.
        void setOrientation(const Quaternion& q)
        {
            data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
            data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
            data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
            data[3] = 2 * q.i * q.j - 2 * q.k * q.r;
            data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
            data[5] = 2 * q.j * q.k + 2 * q.i * q.r;
            data[6] = 2 * q.i * q.k + 2 * q.j * q.r;
            data[7] = 2 * q.j * q.k - 2 * q.i * q.r;
            data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
        }

        // Blends between two matrices.
        static Matrix3 linearInterpolate(const Matrix3& a, const Matrix3& b, real prop);
    };

}

#endif // CYCLONE_CORE_H
