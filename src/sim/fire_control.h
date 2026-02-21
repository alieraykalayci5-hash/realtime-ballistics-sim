#pragma once
#include <cstdint>
#include "ballistics.h"

namespace sim {

struct Target2D {
    // Y-up convention. Use z=0.
    Vec3 pos_m{0.0, 0.0, 0.0};
    Vec3 vel_mps{0.0, 0.0, 0.0};
};

struct FireControlResult {
    bool success = false;

    double lead_elev_deg = 0.0;  // solved elevation
    double t_hit_s = 0.0;        // time at closest approach
    double miss_m = 0.0;         // min distance between projectile and target

    Vec3 proj_at_hit_m{0.0,0.0,0.0};
    Vec3 tgt_at_hit_m{0.0,0.0,0.0};

    // deterministic search diagnostics
    int eval_count = 0;
};

struct FireControlConfig {
    // Search limits
    double elev_min_deg = -5.0;
    double elev_max_deg = 80.0;

    // Coarse-to-fine deterministic grid search steps (deg)
    double step1_deg = 1.0;
    double step2_deg = 0.1;
    double step3_deg = 0.01;

    // Hit threshold (if miss <= hit_radius => success)
    double hit_radius_m = 0.5;
};

// Solve for best elevation angle to hit/match a moving target (2D x-y).
// Uses deterministic simulation-based search (works with gravity and optional drag).
FireControlResult solve_lead_2d(const SimConfig& cfg,
                               double sim_time_s,
                               double v0_mps,
                               double y0_m,
                               const Target2D& tgt,
                               const FireControlConfig& fc);

} // namespace sim