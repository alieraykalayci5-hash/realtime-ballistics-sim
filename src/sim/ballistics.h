#pragma once
#include <cstdint>
#include <vector>

#include "vec3.h"
#include "target.h"

namespace sim {

// Core config: deterministic fixed-step simulation parameters
struct SimConfig {
    double dt = 1.0 / 60.0;
    Vec3 gravity{0.0, -9.80665, 0.0};
    double air_density = 1.225;
    bool enable_drag = true;
    bool ground_collision = true;
};

// Simple projectile model
struct Projectile {
    Vec3 pos_m;
    Vec3 vel_mps;

    double mass_kg = 0.01;
    double area_m2 = 1e-4;
    double cd = 0.47;
    double restitution = 0.0;

    bool alive = true;
};

// Deterministic step (no targets)
void step_projectile(Projectile& p, const SimConfig& cfg);

// Utility: quadratic drag acceleration (for debugging / tests)
Vec3 drag_accel(const Projectile& p, const SimConfig& cfg);

// Impact event report (Stage 2 - reporting)
struct ImpactEvent {
    bool occurred = false;
    double t = 0.0;                 // sim time at impact [s]
    const char* material = nullptr; // material name
    TargetType target_type{TargetType::Sphere};
    Vec3 point_m{};
    double ke_j = 0.0;
    double threshold_j = 0.0;
    bool penetrated = false;
};

// Step with static targets + returns impact event if any
ImpactEvent step_projectile_with_targets(Projectile& p,
                                        const SimConfig& cfg,
                                        const std::vector<Target>& targets,
                                        double current_time_s);

} // namespace sim