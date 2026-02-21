#pragma once
#include <cstdint>
#include "vec3.h"

namespace sim {

// Core config: deterministic fixed-step simulation parameters
struct SimConfig {
    double dt = 1.0 / 60.0;      // fixed timestep [s]
    Vec3 gravity{0.0, -9.80665, 0.0}; // m/s^2 (Y-up convention; ground at y=0)
    double air_density = 1.225;  // kg/m^3 (sea level)
    bool enable_drag = true;
    bool ground_collision = true;
};

// Simple projectile model
struct Projectile {
    Vec3 pos_m;        // position [m]
    Vec3 vel_mps;      // velocity [m/s]

    double mass_kg = 0.01;   // kg
    double area_m2 = 1e-4;   // m^2 (cross-section)
    double cd = 0.47;        // drag coefficient (sphere ~0.47)
    double restitution = 0.0;// 0=stick, 1=perfect bounce (we'll use stick now)

    bool alive = true;
};

// A single deterministic step
void step_projectile(Projectile& p, const SimConfig& cfg);

// Utility: compute quadratic drag acceleration (for debugging / tests)
Vec3 drag_accel(const Projectile& p, const SimConfig& cfg);

} // namespace sim