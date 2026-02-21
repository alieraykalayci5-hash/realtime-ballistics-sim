#include "ballistics.h"
#include <algorithm>

namespace sim {

// Quadratic drag: Fd = 0.5 * rho * Cd * A * v^2  (direction opposite velocity)
// a = F / m
Vec3 drag_accel(const Projectile& p, const SimConfig& cfg) {
    const double v2 = p.vel_mps.length_sq();
    if (v2 <= 0.0) return {0.0, 0.0, 0.0};

    const double k = 0.5 * cfg.air_density * p.cd * p.area_m2; // kg/m
    const double v = std::sqrt(v2);
    const Vec3 vhat = p.vel_mps / v;

    // Drag force opposite v
    const Vec3 Fd = (-k * v2) * vhat;
    return Fd / std::max(p.mass_kg, 1e-9);
}

// Semi-implicit Euler (a.k.a. symplectic Euler):
// v_{t+dt} = v_t + a_t * dt
// x_{t+dt} = x_t + v_{t+dt} * dt
void step_projectile(Projectile& p, const SimConfig& cfg) {
    if (!p.alive) return;

    Vec3 a = cfg.gravity;

    if (cfg.enable_drag) {
        a += drag_accel(p, cfg);
    }

    // integrate
    p.vel_mps += a * cfg.dt;
    p.pos_m   += p.vel_mps * cfg.dt;

    // ground collision at y=0
    if (cfg.ground_collision && p.pos_m.y <= 0.0) {
        p.pos_m.y = 0.0;

        if (p.restitution <= 0.0) {
            // stick: stop vertical and horizontal (simple)
            p.vel_mps = {0.0, 0.0, 0.0};
            p.alive = false; // simulation can decide to stop tracking
        } else {
            // bounce (optional)
            p.vel_mps.y = -p.vel_mps.y * p.restitution;
            // small friction-like damping to avoid infinite bouncing
            p.vel_mps.x *= 0.98;
            p.vel_mps.z *= 0.98;

            // If it becomes too slow, kill it
            if (p.vel_mps.length() < 0.1) {
                p.vel_mps = {0.0, 0.0, 0.0};
                p.alive = false;
            }
        }
    }
}

} // namespace sim