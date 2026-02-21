#pragma once
#include "vec3.h"
#include "target.h"

namespace sim {

struct HitInfo {
    bool hit = false;
    double t = 0.0;   // along segment [0..1]
    Vec3 point_m{};
    Vec3 normal{};
    const Target* target = nullptr;
};

HitInfo intersect_segment_target(const Vec3& p0_m, const Vec3& p1_m, const Target& target);

} // namespace sim