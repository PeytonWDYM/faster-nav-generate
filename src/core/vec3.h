#pragma once

#include <cmath>

namespace navfast {

// Plain 3D vector in Source units (inches), Source axes (+Z up).
struct Vec3 {
    float x = 0, y = 0, z = 0;

    constexpr Vec3 operator+(Vec3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(Vec3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3& operator+=(Vec3 o) { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec3& operator-=(Vec3 o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr bool operator==(const Vec3&) const = default;

    constexpr float dot(Vec3 o) const { return x * o.x + y * o.y + z * o.z; }
    constexpr Vec3 cross(Vec3 o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    constexpr float lengthSqr() const { return dot(*this); }
    float length() const { return std::sqrt(lengthSqr()); }
};

// Axis-aligned bounding box.
struct Aabb {
    Vec3 mins, maxs;
};

} // namespace navfast
