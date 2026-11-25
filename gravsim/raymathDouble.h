#pragma once
/**********************************************************************************************
*
*   raymath v2.1 - Math functions to work with Vector2, Vector3d, Vector4, Matrix and Quaternions
*
*   CONVENTIONS:
*     - Matrix structure is defined as row-major (memory layout) but parameters naming AND all
*       math operations performed by the library consider the structure as it was column-major
*       It is like transposed versions of the matrices are used for all the maths
*       It benefits some functions making them cache-friendly and also avoids matrix
*       transpositions sometimes required by OpenGL
*       Example: In memory order, row0 is [m0 m4 m8 m12] but in semantic math row0 is [m0 m1 m2 m3]
*     - All math is double only, no doubles usage
*     - Functions are always self-contained, no function use another raymath function inside,
*       required code is directly re-implemented inside
*     - Functions input parameters are always received by value (2 unavoidable exceptions)
*     - Functions use always a "result" variable for return (except C++ operators)
*     - Functions are always defined inline
*     - Angles are always in radians (DEG2RAD/RAD2DEG macros provided for convenience)
*     - No compound literals used to make sure the library is compatible with C++
*
*   CONFIGURATION:
*       #define RAYMATH_IMPLEMENTATION
*           Generates the implementation of the library into the included file
*           If not defined, the library is in header only mode and can be included in other headers
*           or source files without problems. But only ONE file should hold the implementation
*
*       #define RAYMATH_STATIC_INLINE
*           Define static inline functions code, so #include header suffices for use
*           This may use up lots of memory
*
*       #define RAYMATH_DISABLE_CPP_OPERATORS
*           Disables C++ operator overloads for raymath types.
*
*       #define RAYMATH_USE_SIMD_INTRINSICS   1
*           Try to enable SIMD intrinsics for MatrixMultiply()
*           Note that users enabling it must be aware of the target platform where application will
*           run to support the selected SIMD intrinsic, for now, only SSE is supported
*
*   LICENSE: zlib/libpng
*
*   Copyright (c) 2015-2026 Ramon Santamaria (@raysan5)
*
*   This software is provided "as-is", without any express or implied warranty. In no event
*   will the authors be held liable for any damages arising from the use of this software.
*
*   Permission is granted to anyone to use this software for any purpose, including commercial
*   applications, and to alter it and redistribute it freely, subject to the following restrictions:
*
*     1. The origin of this software must not be misrepresented; you must not claim that you
*     wrote the original software. If you use this software in a product, an acknowledgment
*     in the product documentation would be appreciated but is not required.
*
*     2. Altered source versions must be plainly marked as such, and must not be misrepresented
*     as being the original software.
*
*     3. This notice may not be removed or altered from any source distribution.
*
**********************************************************************************************/

//----------------------------------------------------------------------------------
// Defines and Macros
//----------------------------------------------------------------------------------
typedef struct Vector3d {
    double x;
    double y;
    double z;
    Vector3d() = default;
    constexpr Vector3d(double x, double y, double z) : x(x), y(y), z(z) {}
    constexpr Vector3d(Vector3 vec) : x(vec.x), y(vec.y), z(vec.z) {}
    explicit operator Vector3() const
    {
        return Vector3{ static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    }
} Vector3d;

#include <math.h>       // Required for: sin(), cos(), tan(), atan2(), sqrt(), floor(), fmin(), fmax(), fabs()

#if RAYMATH_USE_SIMD_INTRINSICS
// SIMD is used on the most costly raymath function MatrixMultiply()
// NOTE: Only SSE intrinsics support implemented
// TODO: Consider support for other SIMD intrinsics:
//  - SSEx, AVX, AVX2, FMA, NEON, RVV
/*
#if defined(__SSE4_2__)
    #include <nmmintrin.h>
    #define RAYMATH_SSE42_ENABLED
#elif defined(__SSE4_1__)
    #include <smmintrin.h>
    #define RAYMATH_SSE41_ENABLED
#elif defined(__SSSE3__)
    #include <tmmintrin.h>
    #define RAYMATH_SSSE3_ENABLED
#elif defined(__SSE3__)
    #include <pmmintrin.h>
    #define RAYMATH_SSE3_ENABLED
#elif defined(__SSE2__) || (defined(_M_AMD64) || defined(_M_X64)) // SSE2 x64
    #include <emmintrin.h>
    #define RAYMATH_SSE2_ENABLED
#endif
*/
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && (_M_IX86_FP >= 1))
#include <xmmintrin.h>
#define RAYMATH_SSE_ENABLED
#endif
#endif

//----------------------------------------------------------------------------------
// Module Functions Definition - Utils math
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// Module Functions Definition - Vector3d math
//----------------------------------------------------------------------------------

// Vector with components value 0.0f
inline Vector3d Vector3dZero(void)
{
    Vector3d result = { 0.0f, 0.0f, 0.0f };

    return result;
}
// Convert Vector3d (double) to Raylib Vector3 (float)
inline Vector3 Vector3dToVector3(Vector3d v)
{
    Vector3 result = { (float)v.x, (float)v.y, (float)v.z };
    return result;
}

// Convert Raylib Vector3 (float) to Vector3d (double)
inline Vector3d Vector3ToVector3d(Vector3 v)
{
    Vector3d result = { (double)v.x, (double)v.y, (double)v.z };
    return result;
}
// Vector with components value 1.0f
inline Vector3d Vector3dOne(void)
{
    Vector3d result = { 1.0f, 1.0f, 1.0f };

    return result;
}

// Add two vectors
inline Vector3d Vector3dAdd(Vector3d v1, Vector3d v2)
{
    Vector3d result = { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };

    return result;
}

// Add vector and double value
inline Vector3d Vector3dAddValue(Vector3d v, double add)
{
    Vector3d result = { v.x + add, v.y + add, v.z + add };

    return result;
}

// Subtract two vectors
inline Vector3d Vector3dSubtract(Vector3d v1, Vector3d v2)
{
    Vector3d result = { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };

    return result;
}

// Subtract vector by double value
inline Vector3d Vector3dSubtractValue(Vector3d v, double sub)
{
    Vector3d result = { v.x - sub, v.y - sub, v.z - sub };

    return result;
}

// Multiply vector by scalar
inline Vector3d Vector3dScale(Vector3d v, double scalar)
{
    Vector3d result = { v.x * scalar, v.y * scalar, v.z * scalar };

    return result;
}

// Multiply vector by vector
inline Vector3d Vector3dMultiply(Vector3d v1, Vector3d v2)
{
    Vector3d result = { v1.x * v2.x, v1.y * v2.y, v1.z * v2.z };

    return result;
}

// Calculate two vectors cross product
inline Vector3d Vector3dCrossProduct(Vector3d v1, Vector3d v2)
{
    Vector3d result = { v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x };

    return result;
}

// Calculate one vector perpendicular vector
inline Vector3d Vector3dPerpendicular(Vector3d v)
{
    Vector3d result = { 0.0,0.0,0.0 };

    double min = fabs(v.x);
    Vector3d cardinalAxis = { 1.0f, 0.0f, 0.0f };

    if (fabs(v.y) < min)
    {
        min = fabs(v.y);
        Vector3d tmp = { 0.0f, 1.0f, 0.0f };
        cardinalAxis = tmp;
    }

    if (fabs(v.z) < min)
    {
        Vector3d tmp = { 0.0f, 0.0f, 1.0f };
        cardinalAxis = tmp;
    }

    // Cross product between vectors
    result.x = v.y * cardinalAxis.z - v.z * cardinalAxis.y;
    result.y = v.z * cardinalAxis.x - v.x * cardinalAxis.z;
    result.z = v.x * cardinalAxis.y - v.y * cardinalAxis.x;

    return result;
}

// Calculate vector length
inline double Vector3dLength(const Vector3d v)
{
    double result = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

    return result;
}

// Calculate vector square length
inline double Vector3dLengthSqr(const Vector3d v)
{
    double result = v.x * v.x + v.y * v.y + v.z * v.z;

    return result;
}

// Calculate two vectors dot product
inline double Vector3dDotProduct(Vector3d v1, Vector3d v2)
{
    double result = (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);

    return result;
}

// Calculate distance between two vectors
inline double Vector3dDistance(Vector3d v1, Vector3d v2)
{
    double result = 0.0f;

    double dx = v2.x - v1.x;
    double dy = v2.y - v1.y;
    double dz = v2.z - v1.z;
    result = sqrt(dx * dx + dy * dy + dz * dz);

    return result;
}

// Calculate square distance between two vectors
inline double Vector3dDistanceSqr(Vector3d v1, Vector3d v2)
{
    double result = 0.0f;

    double dx = v2.x - v1.x;
    double dy = v2.y - v1.y;
    double dz = v2.z - v1.z;
    result = dx * dx + dy * dy + dz * dz;

    return result;
}

// Calculate angle between two vectors
inline double Vector3dAngle(Vector3d v1, Vector3d v2)
{
    double result = 0.0f;

    Vector3d cross = { v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x };
    double len = sqrt(cross.x * cross.x + cross.y * cross.y + cross.z * cross.z);
    double dot = (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
    result = atan2(len, dot);

    return result;
}

// Negate provided vector (invert direction)
inline Vector3d Vector3dNegate(Vector3d v)
{
    Vector3d result = { -v.x, -v.y, -v.z };

    return result;
}

// Divide vector by vector
inline Vector3d Vector3dDivide(Vector3d v1, Vector3d v2)
{
    Vector3d result = { v1.x / v2.x, v1.y / v2.y, v1.z / v2.z };

    return result;
}

// Normalize provided vector
inline Vector3d Vector3dNormalize(Vector3d v)
{
    Vector3d result = v;

    double length = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (length != 0.0f)
    {
        double ilength = 1.0f / length;

        result.x *= ilength;
        result.y *= ilength;
        result.z *= ilength;
    }

    return result;
}

//Calculate the projection of the vector v1 on to v2
inline Vector3d Vector3dProject(Vector3d v1, Vector3d v2)
{
    Vector3d result = { 0.0,0.0,0.0 };

    double v1dv2 = (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
    double v2dv2 = (v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);

    double mag = v1dv2 / v2dv2;

    result.x = v2.x * mag;
    result.y = v2.y * mag;
    result.z = v2.z * mag;

    return result;
}

//Calculate the rejection of the vector v1 on to v2
inline Vector3d Vector3dReject(Vector3d v1, Vector3d v2)
{
    Vector3d result = { 0.0,0.0,0.0 };

    double v1dv2 = (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
    double v2dv2 = (v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);

    double mag = v1dv2 / v2dv2;

    result.x = v1.x - (v2.x * mag);
    result.y = v1.y - (v2.y * mag);
    result.z = v1.z - (v2.z * mag);

    return result;
}

// Orthonormalize provided vectors
// Makes vectors normalized and orthogonal to each other
// Gram-Schmidt function implementation
inline void Vector3dOrthoNormalize(Vector3d* v1, Vector3d* v2)
{
    double length = 0.0f;
    double ilength = 0.0f;

    // Vector3dNormalize(*v1);
    Vector3d v = *v1;
    length = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (length == 0.0f) length = 1.0f;
    ilength = 1.0f / length;
    v1->x *= ilength;
    v1->y *= ilength;
    v1->z *= ilength;

    // Vector3dCrossProduct(*v1, *v2)
    Vector3d vn1 = { v1->y * v2->z - v1->z * v2->y, v1->z * v2->x - v1->x * v2->z, v1->x * v2->y - v1->y * v2->x };

    // Vector3dNormalize(vn1);
    v = vn1;
    length = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (length == 0.0f) length = 1.0f;
    ilength = 1.0f / length;
    vn1.x *= ilength;
    vn1.y *= ilength;
    vn1.z *= ilength;

    // Vector3dCrossProduct(vn1, *v1)
    Vector3d vn2 = { vn1.y * v1->z - vn1.z * v1->y, vn1.z * v1->x - vn1.x * v1->z, vn1.x * v1->y - vn1.y * v1->x };

    *v2 = vn2;
}

// Transforms a Vector3d by a given Matrix

// Rotates a vector around an axis
inline Vector3d Vector3dRotateByAxisAngle(Vector3d v, Vector3d axis, double angle)
{
    // Using Euler-Rodrigues Formula
    // Ref.: https://en.wikipedia.org/w/index.php?title=Euler%E2%80%93Rodrigues_formula

    Vector3d result = v;

    // Vector3dNormalize(axis);
    double length = sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (length == 0.0f) length = 1.0f;
    double ilength = 1.0f / length;
    axis.x *= ilength;
    axis.y *= ilength;
    axis.z *= ilength;

    angle /= 2.0f;
    double a = sin(angle);
    double b = axis.x * a;
    double c = axis.y * a;
    double d = axis.z * a;
    a = cos(angle);
    Vector3d w = { b, c, d };

    // Vector3dCrossProduct(w, v)
    Vector3d wv = { w.y * v.z - w.z * v.y, w.z * v.x - w.x * v.z, w.x * v.y - w.y * v.x };

    // Vector3dCrossProduct(w, wv)
    Vector3d wwv = { w.y * wv.z - w.z * wv.y, w.z * wv.x - w.x * wv.z, w.x * wv.y - w.y * wv.x };

    // Vector3dScale(wv, 2*a)
    a *= 2;
    wv.x *= a;
    wv.y *= a;
    wv.z *= a;

    // Vector3dScale(wwv, 2)
    wwv.x *= 2;
    wwv.y *= 2;
    wwv.z *= 2;

    result.x += wv.x;
    result.y += wv.y;
    result.z += wv.z;

    result.x += wwv.x;
    result.y += wwv.y;
    result.z += wwv.z;

    return result;
}

// Move Vector towards target
inline Vector3d Vector3dMoveTowards(Vector3d v, Vector3d target, double maxDistance)
{
    Vector3d result = { 0.0,0.0,0.0 };

    double dx = target.x - v.x;
    double dy = target.y - v.y;
    double dz = target.z - v.z;
    double value = (dx * dx) + (dy * dy) + (dz * dz);

    if ((value == 0) || ((maxDistance >= 0) && (value <= maxDistance * maxDistance))) return target;

    double dist = sqrt(value);

    result.x = v.x + dx / dist * maxDistance;
    result.y = v.y + dy / dist * maxDistance;
    result.z = v.z + dz / dist * maxDistance;

    return result;
}

// Calculate linear interpolation between two vectors
inline Vector3d Vector3dLerp(Vector3d v1, Vector3d v2, double amount)
{
    Vector3d result = { 0.0,0.0,0.0 };

    result.x = v1.x + amount * (v2.x - v1.x);
    result.y = v1.y + amount * (v2.y - v1.y);
    result.z = v1.z + amount * (v2.z - v1.z);

    return result;
}

// Calculate cubic hermite interpolation between two vectors and their tangents
// as described in the GLTF 2.0 specification: https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#interpolation-cubic
inline Vector3d Vector3dCubicHermite(Vector3d v1, Vector3d tangent1, Vector3d v2, Vector3d tangent2, double amount)
{
    Vector3d result = { 0.0,0.0,0.0 };

    double amountPow2 = amount * amount;
    double amountPow3 = amount * amount * amount;

    result.x = (2 * amountPow3 - 3 * amountPow2 + 1) * v1.x + (amountPow3 - 2 * amountPow2 + amount) * tangent1.x + (-2 * amountPow3 + 3 * amountPow2) * v2.x + (amountPow3 - amountPow2) * tangent2.x;
    result.y = (2 * amountPow3 - 3 * amountPow2 + 1) * v1.y + (amountPow3 - 2 * amountPow2 + amount) * tangent1.y + (-2 * amountPow3 + 3 * amountPow2) * v2.y + (amountPow3 - amountPow2) * tangent2.y;
    result.z = (2 * amountPow3 - 3 * amountPow2 + 1) * v1.z + (amountPow3 - 2 * amountPow2 + amount) * tangent1.z + (-2 * amountPow3 + 3 * amountPow2) * v2.z + (amountPow3 - amountPow2) * tangent2.z;

    return result;
}

// Calculate reflected vector to normal
inline Vector3d Vector3dReflect(Vector3d v, Vector3d normal)
{
    Vector3d result = { 0.0,0.0,0.0 };

    // I is the original vector
    // N is the normal of the incident plane
    // R = I - (2*N*(DotProduct[I, N]))

    double dotProduct = (v.x * normal.x + v.y * normal.y + v.z * normal.z);

    result.x = v.x - (2.0f * normal.x) * dotProduct;
    result.y = v.y - (2.0f * normal.y) * dotProduct;
    result.z = v.z - (2.0f * normal.z) * dotProduct;

    return result;
}

// Get min value for each pair of components
inline Vector3d Vector3dMin(Vector3d v1, Vector3d v2)
{
    Vector3d result = { 0.0,0.0,0.0 };

    result.x = fmin(v1.x, v2.x);
    result.y = fmin(v1.y, v2.y);
    result.z = fmin(v1.z, v2.z);

    return result;
}

// Get max value for each pair of components
inline Vector3d Vector3dMax(Vector3d v1, Vector3d v2)
{
    Vector3d result = { 0.0,0.0,0.0 };

    result.x = fmax(v1.x, v2.x);
    result.y = fmax(v1.y, v2.y);
    result.z = fmax(v1.z, v2.z);

    return result;
}
inline Vector3d Vector3dTransform(Vector3d v, Matrix mat)
{
    double x = v.x;
    double y = v.y;
    double z = v.z;

    return {
        mat.m0 * x + mat.m4 * y + mat.m8 * z + mat.m12,
        mat.m1 * x + mat.m5 * y + mat.m9 * z + mat.m13,
        mat.m2 * x + mat.m6 * y + mat.m10 * z + mat.m14
    };
}
// Compute barycenter coordinates (u, v, w) for point p with respect to triangle (a, b, c)
// NOTE: Assumes P is on the plane of the triangle
inline Vector3d Vector3dBarycenter(Vector3d p, Vector3d a, Vector3d b, Vector3d c)
{
    Vector3d result = { 0.0,0.0,0.0 };

    Vector3d v0 = { b.x - a.x, b.y - a.y, b.z - a.z };   // Vector3dSubtract(b, a)
    Vector3d v1 = { c.x - a.x, c.y - a.y, c.z - a.z };   // Vector3dSubtract(c, a)
    Vector3d v2 = { p.x - a.x, p.y - a.y, p.z - a.z };   // Vector3dSubtract(p, a)u
    double d00 = (v0.x * v0.x + v0.y * v0.y + v0.z * v0.z);    // Vector3dDotProduct(v0, v0)
    double d01 = (v0.x * v1.x + v0.y * v1.y + v0.z * v1.z);    // Vector3dDotProduct(v0, v1)
    double d11 = (v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);    // Vector3dDotProduct(v1, v1)
    double d20 = (v2.x * v0.x + v2.y * v0.y + v2.z * v0.z);    // Vector3dDotProduct(v2, v0)
    double d21 = (v2.x * v1.x + v2.y * v1.y + v2.z * v1.z);    // Vector3dDotProduct(v2, v1)

    double denom = d00 * d11 - d01 * d01;

    result.y = (d11 * d20 - d01 * d21) / denom;
    result.z = (d00 * d21 - d01 * d20) / denom;
    result.x = 1.0f - (result.z + result.y);

    return result;
}

// Projects a Vector3d from screen space into object space
// NOTE: Self-contained function, no other raymath functions are called
// Get Vector3d as double array


// Invert the given vector
inline Vector3d Vector3dInvert(Vector3d v)
{
    Vector3d result = { 1.0f / v.x, 1.0f / v.y, 1.0f / v.z };

    return result;
}

// Clamp the components of the vector between
// min and max values specified by the given vectors
inline Vector3d Vector3dClamp(Vector3d v, Vector3d min, Vector3d max)
{
    Vector3d result = { 0.0,0.0,0.0 };

    result.x = fmin(max.x, fmax(min.x, v.x));
    result.y = fmin(max.y, fmax(min.y, v.y));
    result.z = fmin(max.z, fmax(min.z, v.z));

    return result;
}

// Clamp the magnitude of the vector between two values
inline Vector3d Vector3dClampValue(Vector3d v, double min, double max)
{
    Vector3d result = v;

    double length = (v.x * v.x) + (v.y * v.y) + (v.z * v.z);
    if (length > 0.0f)
    {
        length = sqrt(length);

        double scale = 1; // By default, 1 as the neutral element
        if (length < min) scale = min / length;
        else if (length > max) scale = max / length;

        result.x = v.x * scale;
        result.y = v.y * scale;
        result.z = v.z * scale;
    }

    return result;
}

// Check whether two given vectors are almost equal
inline int Vector3dEquals(Vector3d p, Vector3d q)
{
#if !defined(EPSILON)
#define EPSILON 0.000001f
#endif

    int result = ((fabs(p.x - q.x)) <= (EPSILON * fmax(1.0f, fmax(fabs(p.x), fabs(q.x))))) &&
        ((fabs(p.y - q.y)) <= (EPSILON * fmax(1.0f, fmax(fabs(p.y), fabs(q.y))))) &&
        ((fabs(p.z - q.z)) <= (EPSILON * fmax(1.0f, fmax(fabs(p.z), fabs(q.z)))));

    return result;
}

// Compute the direction of a refracted ray
// v: normalized direction of the incoming ray
// n: normalized normal vector of the interface of two optical media
// r: ratio of the refractive index of the medium from where the ray comes
// to the refractive index of the medium on the other side of the surface
inline Vector3d Vector3dRefract(Vector3d v, Vector3d n, double r)
{
    Vector3d result = { 0.0,0.0,0.0 };

    double dot = v.x * n.x + v.y * n.y + v.z * n.z;
    double d = 1.0f - r * r * (1.0f - dot * dot);

    if (d >= 0.0f)
    {
        d = sqrt(d);
        v.x = r * v.x - (r * dot + d) * n.x;
        v.y = r * v.y - (r * dot + d) * n.y;
        v.z = r * v.z - (r * dot + d) * n.z;

        result = v;
    }

    return result;
}
// Optional C++ math operators
//-------------------------------------------------------------------------------

// Vector3d operators

inline Vector3d operator + (const Vector3d& lhs, const Vector3d& rhs)
{
    return Vector3dAdd(lhs, rhs);
}

inline const Vector3d& operator += (Vector3d& lhs, const Vector3d& rhs)
{
    lhs = Vector3dAdd(lhs, rhs);
    return lhs;
}

inline Vector3d operator - (const Vector3d& lhs, const Vector3d& rhs)
{
    return Vector3dSubtract(lhs, rhs);
}

inline const Vector3d& operator -= (Vector3d& lhs, const Vector3d& rhs)
{
    lhs = Vector3dSubtract(lhs, rhs);
    return lhs;
}

inline Vector3d operator * (const double& lhs, const Vector3d& rhs)
{
    return Vector3dScale(rhs, lhs);
}

inline Vector3d operator * (const Vector3d& lhs, const double& rhs)
{
    return Vector3dScale(lhs, rhs);
}

inline const Vector3d& operator *= (Vector3d& lhs, const double& rhs)
{
    lhs = Vector3dScale(lhs, rhs);
    return lhs;
}

inline Vector3d operator * (const Vector3d& lhs, const Vector3d& rhs)
{
    return Vector3dMultiply(lhs, rhs);
}

inline const Vector3d& operator *= (Vector3d& lhs, const Vector3d& rhs)
{
    lhs = Vector3dMultiply(lhs, rhs);
    return lhs;
}

inline Vector3d operator / (const Vector3d& lhs, const double& rhs)
{
    return Vector3dScale(lhs, 1.0f / rhs);
}

inline const Vector3d& operator /= (Vector3d& lhs, const double& rhs)
{
    lhs = Vector3dScale(lhs, 1.0f / rhs);
    return lhs;
}

inline Vector3d operator / (const Vector3d& lhs, const Vector3d& rhs)
{
    return Vector3dDivide(lhs, rhs);
}

inline const Vector3d& operator /= (Vector3d& lhs, const Vector3d& rhs)
{
    lhs = Vector3dDivide(lhs, rhs);
    return lhs;
}

inline bool operator == (const Vector3d& lhs, const Vector3d& rhs)
{
    return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

inline bool operator != (const Vector3d& lhs, const Vector3d& rhs)
{
    return !(lhs.x == rhs.x || lhs.y == rhs.y || lhs.z == rhs.z);
}
inline Vector3 ConvertRegular(Vector3d v)
{
    return Vector3{ float(v.x), float(v.y), float(v.z) };
}
inline Vector3d ConvertDouble(Vector3 v)
{
    return Vector3d{ double(v.x),double(v.y),double(v.z) };
}
//-------------------------------------------------------------------------------