#include "collision.h"
#include <algorithm>
#include <cmath>

namespace sim {

static HitInfo intersect_segment_sphere(const Vec3& p0, const Vec3& p1, const Sphere& s) {
    HitInfo out;

    const Vec3 d = p1 - p0;
    const Vec3 m = p0 - s.center_m;

    const double a = dot(d, d);
    if (a <= 0.0) return out;

    const double b = 2.0 * dot(m, d);
    const double c = dot(m, m) - s.radius_m * s.radius_m;

    const double disc = b*b - 4.0*a*c;
    if (disc < 0.0) return out;

    const double sqrt_disc = std::sqrt(disc);
    const double inv2a = 1.0 / (2.0 * a);

    double t0 = (-b - sqrt_disc) * inv2a;
    double t1 = (-b + sqrt_disc) * inv2a;
    if (t0 > t1) std::swap(t0, t1);

    double thit = t0;
    if (thit < 0.0 || thit > 1.0) {
        thit = t1;
        if (thit < 0.0 || thit > 1.0) return out;
    }

    out.hit = true;
    out.t = thit;
    out.point_m = p0 + d * thit;
    out.normal = normalize(out.point_m - s.center_m);
    return out;
}

static HitInfo intersect_segment_aabb(const Vec3& p0, const Vec3& p1, const AABB& b) {
    HitInfo out;

    const Vec3 d = p1 - p0;
    double tmin = 0.0;
    double tmax = 1.0;

    auto update_slab = [&](double p, double dp, double mn, double mx) -> bool {
        if (std::abs(dp) < 1e-12) {
            return (p >= mn && p <= mx);
        }
        const double inv = 1.0 / dp;
        double t0 = (mn - p) * inv;
        double t1 = (mx - p) * inv;
        if (t0 > t1) std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        return (tmin <= tmax);
    };

    if (!update_slab(p0.x, d.x, b.min_m.x, b.max_m.x)) return out;
    if (!update_slab(p0.y, d.y, b.min_m.y, b.max_m.y)) return out;
    if (!update_slab(p0.z, d.z, b.min_m.z, b.max_m.z)) return out;

    const double thit = tmin;
    if (thit < 0.0 || thit > 1.0) return out;

    const Vec3 phit = p0 + d * thit;

    Vec3 n{0,0,0};
    const double eps = 1e-6;
    if (std::abs(phit.x - b.min_m.x) < eps) n = {-1,0,0};
    else if (std::abs(phit.x - b.max_m.x) < eps) n = {+1,0,0};
    else if (std::abs(phit.y - b.min_m.y) < eps) n = {0,-1,0};
    else if (std::abs(phit.y - b.max_m.y) < eps) n = {0,+1,0};
    else if (std::abs(phit.z - b.min_m.z) < eps) n = {0,0,-1};
    else if (std::abs(phit.z - b.max_m.z) < eps) n = {0,0,+1};

    out.hit = true;
    out.t = thit;
    out.point_m = phit;
    out.normal = n;
    return out;
}

HitInfo intersect_segment_target(const Vec3& p0_m, const Vec3& p1_m, const Target& target) {
    HitInfo h;
    switch (target.type) {
        case TargetType::Sphere:
            h = intersect_segment_sphere(p0_m, p1_m, target.sphere);
            break;
        case TargetType::AABB:
            h = intersect_segment_aabb(p0_m, p1_m, target.aabb);
            break;
        default:
            break;
    }
    if (h.hit) h.target = &target;
    return h;
}

} // namespace sim