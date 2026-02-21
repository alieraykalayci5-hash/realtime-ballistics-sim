#pragma once
#include "vec3.h"

namespace sim {

struct Sphere {
    Vec3 center_m{};
    double radius_m = 0.5;
};

struct AABB {
    Vec3 min_m{};
    Vec3 max_m{};
};

} // namespace sim