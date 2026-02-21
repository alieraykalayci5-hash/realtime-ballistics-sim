#pragma once
#include <cmath>

namespace sim {

struct Vec3 {
    double x{0.0}, y{0.0}, z{0.0};

    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& r) const { return {x + r.x, y + r.y, z + r.z}; }
    constexpr Vec3 operator-(const Vec3& r) const { return {x - r.x, y - r.y, z - r.z}; }
    constexpr Vec3 operator*(double s)     const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(double s)     const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& r) { x += r.x; y += r.y; z += r.z; return *this; }
    Vec3& operator-=(const Vec3& r) { x -= r.x; y -= r.y; z -= r.z; return *this; }
    Vec3& operator*=(double s)      { x *= s; y *= s; z *= s; return *this; }
    Vec3& operator/=(double s)      { x /= s; y /= s; z /= s; return *this; }

    double length_sq() const { return x*x + y*y + z*z; }
    double length() const { return std::sqrt(length_sq()); }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

inline double dot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline Vec3 normalize(const Vec3& v) {
    const double len = v.length();
    if (len <= 0.0) return {0.0, 0.0, 0.0};
    return v / len;
}

} // namespace sim