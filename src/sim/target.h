#pragma once
#include <cstdint>
#include "geometry.h"
#include "material.h"

namespace sim {

enum class TargetType : uint8_t {
    Sphere,
    AABB
};

struct Target {
    TargetType type = TargetType::Sphere;
    Material material{};

    Sphere sphere{};
    AABB aabb{};

    static Target make_sphere(const Vec3& c_m, double r_m, Material m) {
        Target t;
        t.type = TargetType::Sphere;
        t.material = m;
        t.sphere.center_m = c_m;
        t.sphere.radius_m = r_m;
        return t;
    }

    static Target make_aabb(const Vec3& mn_m, const Vec3& mx_m, Material m) {
        Target t;
        t.type = TargetType::AABB;
        t.material = m;
        t.aabb.min_m = mn_m;
        t.aabb.max_m = mx_m;
        return t;
    }
};

} // namespace sim