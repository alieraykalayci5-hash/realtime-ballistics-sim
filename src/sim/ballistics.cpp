#include "ballistics.h"
#include "collision.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace sim {

// Quadratic drag: Fd = 0.5 * rho * Cd * A * v^2  (opposite velocity)
// a = F / m
Vec3 drag_accel(const Projectile& p, const SimConfig& cfg) {
    const double v2 = p.vel_mps.length_sq();
    if (v2 <= 0.0) return {0.0, 0.0, 0.0};

    const double k = 0.5 * cfg.air_density * p.cd * p.area_m2;
    const double v = std::sqrt(v2);
    const Vec3 vhat = p.vel_mps / v;

    const Vec3 Fd = (-k * v2) * vhat;
    return Fd / std::max(p.mass_kg, 1e-9);
}

// Semi-implicit Euler (symplectic Euler)
void step_projectile(Projectile& p, const SimConfig& cfg) {
    if (!p.alive) return;

    Vec3 a = cfg.gravity;
    if (cfg.enable_drag) {
        a += drag_accel(p, cfg);
    }

    p.vel_mps += a * cfg.dt;
    p.pos_m   += p.vel_mps * cfg.dt;

    // ground collision at y=0
    if (cfg.ground_collision && p.pos_m.y <= 0.0) {
        p.pos_m.y = 0.0;

        if (p.restitution <= 0.0) {
            p.vel_mps = {0.0, 0.0, 0.0};
            p.alive = false;
        } else {
            p.vel_mps.y = -p.vel_mps.y * p.restitution;
            p.vel_mps.x *= 0.98;
            p.vel_mps.z *= 0.98;

            if (p.vel_mps.length() < 0.1) {
                p.vel_mps = {0.0, 0.0, 0.0};
                p.alive = false;
            }
        }
    }
}

ImpactEvent step_projectile_with_targets(Projectile& p,
                                        const SimConfig& cfg,
                                        const std::vector<Target>& targets,
                                        double current_time_s) {
    ImpactEvent ev{};
    if (!p.alive) return ev;

    const Vec3 prev = p.pos_m;

    // physics step -> produces next
    step_projectile(p, cfg);

    const Vec3 next = p.pos_m;

    // If it died due to ground, do not report target impact
    if (!p.alive) return ev;

    // Find earliest segment hit
    HitInfo best{};
    best.hit = false;
    best.t = 1e9;

    for (const auto& tgt : targets) {
        HitInfo h = intersect_segment_target(prev, next, tgt);
        if (h.hit && h.t < best.t) best = h;
    }

    if (!best.hit) return ev;

    // Move projectile to hit point
    p.pos_m = best.point_m;

    // Impact energy
    const double v = p.vel_mps.length();
    const double ke = 0.5 * p.mass_kg * v * v;

    const double thr = best.target->material.penetration_threshold_j;

    bool penetrated = false;

    if (ke <= thr) {
        // stop
        p.vel_mps = {0.0, 0.0, 0.0};
        p.alive = false;
        penetrated = false;
    } else {
        // penetrate: subtract threshold energy deterministically
        const double ke_rem = ke - thr;
        const double v_new =
            std::sqrt(std::max(0.0, (2.0 * ke_rem) / std::max(p.mass_kg, 1e-9)));

        const Vec3 dir = normalize(p.vel_mps);
        p.vel_mps = dir * v_new;

        // push slightly forward to avoid re-hitting same surface
        p.pos_m += dir * 1e-6;
        penetrated = true;
    }

    // Fill event
    ev.occurred = true;
    ev.t = current_time_s;
    ev.material = best.target->material.name;
    ev.target_type = best.target->type;
    ev.point_m = best.point_m;
    ev.ke_j = ke;
    ev.threshold_j = thr;
    ev.penetrated = penetrated;

    return ev;
}

} // namespace sim