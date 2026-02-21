#include "fire_control.h"
#include <cmath>
#include <algorithm>

namespace sim {

static double deg2rad(double deg) {
    return deg * (3.14159265358979323846 / 180.0);
}

static double clamp(double x, double a, double b) {
    return std::max(a, std::min(b, x));
}

// Minimum distance between two points moving linearly over [0, dt]:
// P(t) = P0 + VP * t
// G(t) = G0 + VG * t
// D(t) = P(t) - G(t) = D0 + Vrel * t
// minimize ||D(t)|| over t in [0, dt]
static void min_dist_linear(const Vec3& p0, const Vec3& p1,
                            const Vec3& g0, const Vec3& g1,
                            double dt,
                            double& out_dmin,
                            double& out_tau,   // tau in [0, dt]
                            Vec3& out_p_at,
                            Vec3& out_g_at) {
    // Handle dt edge
    if (dt <= 0.0) {
        const Vec3 d = p0 - g0;
        out_dmin = d.length();
        out_tau = 0.0;
        out_p_at = p0;
        out_g_at = g0;
        return;
    }

    const Vec3 vp = (p1 - p0) / dt;
    const Vec3 vg = (g1 - g0) / dt;

    const Vec3 d0 = p0 - g0;
    const Vec3 v  = vp - vg;

    const double vv = v.length_sq();

    double tau = 0.0;
    if (vv > 1e-18) {
        // minimize ||d0 + v*t||^2 => t* = -dot(d0, v)/||v||^2
        tau = -dot(d0, v) / vv;
        tau = clamp(tau, 0.0, dt);
    } else {
        tau = 0.0;
    }

    const Vec3 p_at = p0 + vp * tau;
    const Vec3 g_at = g0 + vg * tau;
    const Vec3 d_at = p_at - g_at;

    out_dmin = d_at.length();
    out_tau = tau;
    out_p_at = p_at;
    out_g_at = g_at;
}

// Evaluate candidate elevation: run deterministic sim and track continuous-time minimum miss.
static void eval_angle(const SimConfig& cfg,
                       double sim_time_s,
                       double v0_mps,
                       double y0_m,
                       const Target2D& tgt0,
                       double elev_deg,
                       double& out_min_miss,
                       double& out_t_at_min,
                       Vec3& out_proj_at_min,
                       Vec3& out_tgt_at_min,
                       int& io_eval_count) {
    io_eval_count++;

    Projectile p;
    p.pos_m = {0.0, y0_m, 0.0};

    const double th = deg2rad(elev_deg);
    p.vel_mps = {v0_mps * std::cos(th), v0_mps * std::sin(th), 0.0};

    // Representative projectile parameters (match sim)
    p.mass_kg = 0.0095;
    p.area_m2 = 6.0e-5;
    p.cd = 0.295;
    p.restitution = 0.0;
    p.alive = true;

    Target2D tgt = tgt0;

    const int steps = (cfg.dt > 0.0) ? static_cast<int>(sim_time_s / cfg.dt) : 0;

    double t = 0.0;

    double best = 1e300;
    double best_t = 0.0;
    Vec3 best_p{};
    Vec3 best_g{};

    // We track continuous min over each step interval [k, k+1]
    for (int k = 0; k < steps; ++k) {
        if (!p.alive) break;

        const Vec3 p0 = p.pos_m;
        const Vec3 g0 = tgt.pos_m;

        // advance one step
        step_projectile(p, cfg);
        tgt.pos_m += tgt.vel_mps * cfg.dt;

        const Vec3 p1 = p.pos_m;
        const Vec3 g1 = tgt.pos_m;

        // continuous-time min in this interval
        double dmin = 0.0, tau = 0.0;
        Vec3 p_at{}, g_at{};
        min_dist_linear(p0, p1, g0, g1, cfg.dt, dmin, tau, p_at, g_at);

        if (dmin < best) {
            best = dmin;
            best_t = t + tau;
            best_p = p_at;
            best_g = g_at;
        }

        t += cfg.dt;
    }

    out_min_miss = best;
    out_t_at_min = best_t;
    out_proj_at_min = best_p;
    out_tgt_at_min = best_g;
}

static FireControlResult search_grid(const SimConfig& cfg,
                                    double sim_time_s,
                                    double v0_mps,
                                    double y0_m,
                                    const Target2D& tgt,
                                    const FireControlConfig& fc,
                                    double center_deg,
                                    double half_window_deg,
                                    double step_deg,
                                    int& io_eval_count) {
    FireControlResult res;
    res.success = false;
    res.eval_count = 0;

    const double a = clamp(center_deg - half_window_deg, fc.elev_min_deg, fc.elev_max_deg);
    const double b = clamp(center_deg + half_window_deg, fc.elev_min_deg, fc.elev_max_deg);

    double best_miss = 1e300;
    double best_elev = a;
    double best_t = 0.0;
    Vec3 best_p{}, best_g{};

    // Deterministic scan
    for (double e = a; e <= b + 1e-12; e += step_deg) {
        double miss = 0.0;
        double tmin = 0.0;
        Vec3 pmin{}, gmin{};
        eval_angle(cfg, sim_time_s, v0_mps, y0_m, tgt, e, miss, tmin, pmin, gmin, io_eval_count);

        if (miss < best_miss) {
            best_miss = miss;
            best_elev = e;
            best_t = tmin;
            best_p = pmin;
            best_g = gmin;
        }
    }

    res.lead_elev_deg = best_elev;
    res.miss_m = best_miss;
    res.t_hit_s = best_t;
    res.proj_at_hit_m = best_p;
    res.tgt_at_hit_m = best_g;
    res.success = (best_miss <= fc.hit_radius_m);

    return res;
}

FireControlResult solve_lead_2d(const SimConfig& cfg,
                               double sim_time_s,
                               double v0_mps,
                               double y0_m,
                               const Target2D& tgt,
                               const FireControlConfig& fc) {
    FireControlResult best{};
    best.success = false;
    best.miss_m = 1e300;
    best.eval_count = 0;

    // Stage 1: coarse scan (full range)
    {
        const double center = 0.5 * (fc.elev_min_deg + fc.elev_max_deg);
        const double half = 0.5 * (fc.elev_max_deg - fc.elev_min_deg);
        int evals = 0;
        FireControlResult r = search_grid(cfg, sim_time_s, v0_mps, y0_m, tgt, fc,
                                         center, half, fc.step1_deg, evals);
        r.eval_count = evals;
        best = r;
        best.eval_count = evals;
    }

    // Stage 2: refine around best
    {
        int evals = best.eval_count;
        FireControlResult r = search_grid(cfg, sim_time_s, v0_mps, y0_m, tgt, fc,
                                         best.lead_elev_deg, 2.0 * fc.step1_deg, fc.step2_deg, evals);
        if (r.miss_m < best.miss_m) best = r;
        best.eval_count = evals;
    }

    // Stage 3: fine refine around best
    {
        int evals = best.eval_count;
        FireControlResult r = search_grid(cfg, sim_time_s, v0_mps, y0_m, tgt, fc,
                                         best.lead_elev_deg, 2.0 * fc.step2_deg, fc.step3_deg, evals);
        if (r.miss_m < best.miss_m) best = r;
        best.eval_count = evals;
    }

    best.success = (best.miss_m <= fc.hit_radius_m);
    return best;
}

} // namespace sim