#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <cstdint>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <fstream>

#include "sim/ballistics.h"
#include "sim/target.h"

struct Args {
    double dt = 1.0 / 60.0;
    double sim_time_s = 8.0;
    int drag_on = 1;

    double v0 = 260.0;
    double elev_deg = 8.0;
    double y0 = 1.5;

    std::string scenario = "demo"; // demo | hit | none
    bool print_hash = false;
};

static void usage() {
    std::cout <<
        "Usage:\n"
        "  ballistics [--scenario demo|hit|none]\n"
        "             [--dt <seconds>] [--t <seconds>] [--drag 0|1]\n"
        "             [--v0 <mps>] [--elev <deg>] [--y0 <m>] [--hash]\n"
        "\n"
        "Scenarios:\n"
        "  demo : medium-range targets (sphere at ~40m, AABB at ~70m), drag default ON\n"
        "  hit  : hit-guaranteed test scenario (sphere at ~20m), drag default OFF\n"
        "  none : no targets (trajectory only)\n"
        "\n"
        "Outputs:\n"
        "  - Trajectory CSV to stdout\n"
        "  - impacts.csv written in working directory (only header if no impacts)\n"
        "  - Optional determinism hash printed to stderr when --hash is used\n"
        "\n"
        "Examples:\n"
        "  ballistics --scenario demo --dt 0.0166667 --t 8 --drag 1 --v0 260 --elev 8 --y0 1.5 --hash > traj.csv\n"
        "  ballistics --scenario hit  --t 3 --hash > traj.csv\n"
        "  ballistics --scenario none --t 8 --hash > traj.csv\n";
}

static bool parse_double(int& i, int argc, char** argv, double& out) {
    if (i + 1 >= argc) return false;
    out = std::atof(argv[i + 1]);
    i++;
    return true;
}

static bool parse_int(int& i, int argc, char** argv, int& out) {
    if (i + 1 >= argc) return false;
    out = std::atoi(argv[i + 1]);
    i++;
    return true;
}

static uint64_t fnv1a64_update(uint64_t h, const void* data, size_t n) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    for (size_t k = 0; k < n; ++k) {
        h ^= p[k];
        h *= 1099511628211ULL;
    }
    return h;
}

static bool parse_args(int argc, char** argv, Args& a) {
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];

        if (s == "--help" || s == "-h") {
            usage();
            return false;
        } else if (s == "--scenario") {
            if (i + 1 >= argc) return false;
            a.scenario = argv[i + 1];
            i++;
        } else if (s == "--dt") {
            if (!parse_double(i, argc, argv, a.dt)) return false;
        } else if (s == "--t") {
            if (!parse_double(i, argc, argv, a.sim_time_s)) return false;
        } else if (s == "--drag") {
            if (!parse_int(i, argc, argv, a.drag_on)) return false;
        } else if (s == "--v0") {
            if (!parse_double(i, argc, argv, a.v0)) return false;
        } else if (s == "--elev") {
            if (!parse_double(i, argc, argv, a.elev_deg)) return false;
        } else if (s == "--y0") {
            if (!parse_double(i, argc, argv, a.y0)) return false;
        } else if (s == "--hash") {
            a.print_hash = true;
        } else {
            std::cerr << "Unknown arg: " << s << "\n";
            return false;
        }
    }
    return true;
}

struct ScenarioConfig {
    // When scenario sets “recommended defaults”, we apply them only if user didn’t override.
    // To keep it simple, we apply scenario defaults always unless user explicitly provides flags.
    // For now: scenario can override dt/t/drag/v0/elev/y0 if desired.
    // (We keep it minimal: only drag defaults differ between demo and hit.)
};

static std::vector<sim::Target> make_targets_for_scenario(const std::string& name) {
    using namespace sim;

    static const Material steel{"steel", 1800.0};
    static const Material wood {"wood",   250.0};

    if (name == "none") {
        return {};
    }

    if (name == "hit") {
        // Hit-guaranteed: sphere aligned with launch height, closer range.
        return {
            Target::make_sphere({20.0, 1.0, 0.0}, 0.6, wood),
            Target::make_aabb  ({40.0, 0.0, -0.5}, {42.0, 2.0, 0.5}, steel)
        };
    }

    // default: "demo"
    return {
        Target::make_sphere({40.0, 1.0, 0.0}, 0.5, wood),
        Target::make_aabb  ({70.0, 0.0, -0.5}, {72.0, 2.0, 0.5}, steel)
    };
}

static void apply_scenario_defaults(const std::string& scenario, Args& a) {
    // Keep defaults “engineering sensible”:
    // - demo: drag ON, v0/elev/y0 typical
    // - hit : drag OFF, straight shot, short sim
    // - none: doesn’t matter, keep user’s inputs
    if (scenario == "hit") {
        a.drag_on = 0;
        a.sim_time_s = 3.0;
        a.v0 = 120.0;
        a.elev_deg = 0.0;
        a.y0 = 1.0;
        a.dt = 1.0 / 60.0;
    } else if (scenario == "demo") {
        // keep existing defaults as-is
    } else if (scenario == "none") {
        // keep as-is
    }
}

int main(int argc, char** argv) {
    using namespace sim;

    Args args;

    // First pass parse to detect scenario early? We’ll do:
    // - If user passed --scenario, we want scenario defaults first,
    //   then parse again to allow user overrides.
    // Implementation: scan scenario manually, apply defaults, then full parse.

    std::string scenario_from_cli = "demo";
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if (s == "--scenario" && i + 1 < argc) {
            scenario_from_cli = argv[i + 1];
            break;
        }
    }
    args.scenario = scenario_from_cli;
    apply_scenario_defaults(args.scenario, args);

    if (!parse_args(argc, argv, args)) {
        usage();
        return 2;
    }

    if (args.scenario != "demo" && args.scenario != "hit" && args.scenario != "none") {
        std::cerr << "Invalid --scenario: " << args.scenario << "\n";
        usage();
        return 2;
    }

    SimConfig cfg;
    cfg.dt = args.dt;
    cfg.gravity = {0.0, -9.80665, 0.0};
    cfg.air_density = 1.225;
    cfg.enable_drag = (args.drag_on != 0);
    cfg.ground_collision = true;

    if (cfg.dt <= 0.0 || args.sim_time_s <= 0.0) {
        std::cerr << "Invalid dt or t\n";
        return 2;
    }

    Projectile p;
    p.pos_m = {0.0, args.y0, 0.0};

    const double deg2rad = 3.14159265358979323846 / 180.0;
    const double th = args.elev_deg * deg2rad;
    p.vel_mps = {args.v0 * std::cos(th), args.v0 * std::sin(th), 0.0};

    p.mass_kg = 0.0095;
    p.area_m2 = 6.0e-5;
    p.cd = 0.295;
    p.restitution = 0.0;

    const std::vector<Target> targets = make_targets_for_scenario(args.scenario);

    const int steps = static_cast<int>(args.sim_time_s / cfg.dt);
    if (steps <= 0) {
        std::cerr << "Invalid dt or t\n";
        return 2;
    }

    uint64_t h = 1469598103934665603ULL;

    auto emit = [&](const std::string& s) {
        std::cout << s;
        h = fnv1a64_update(h, s.data(), s.size());
    };

    // Trajectory CSV (stdout)
    emit("t,x,y,z,vx,vy,vz,speed,alive\n");

    // Impact report (file)
    std::ofstream impacts("impacts.csv", std::ios::binary);
    impacts << "t,material,target,ke_j,threshold_j,result,x,y,z\n";
    impacts << std::fixed << std::setprecision(6);

    double t = 0.0;
    for (int k = 0; k < steps; ++k) {
        const double speed = p.vel_mps.length();

        std::ostringstream line;
        line.setf(std::ios::fixed);
        line << std::setprecision(6);

        line << t << ","
             << p.pos_m.x << "," << p.pos_m.y << "," << p.pos_m.z << ","
             << p.vel_mps.x << "," << p.vel_mps.y << "," << p.vel_mps.z << ","
             << speed << ","
             << (p.alive ? 1 : 0) << "\n";

        emit(line.str());

        if (!p.alive) break;

        if (!targets.empty()) {
            ImpactEvent ev = step_projectile_with_targets(p, cfg, targets, t);
            if (ev.occurred) {
                impacts << ev.t << ","
                        << (ev.material ? ev.material : "unknown") << ","
                        << (ev.target_type == TargetType::Sphere ? "sphere" : "aabb") << ","
                        << ev.ke_j << ","
                        << ev.threshold_j << ","
                        << (ev.penetrated ? "penetrate" : "stop") << ","
                        << ev.point_m.x << ","
                        << ev.point_m.y << ","
                        << ev.point_m.z << "\n";
            }
        } else {
            step_projectile(p, cfg);
        }

        t += cfg.dt;
    }

    if (args.print_hash) {
        std::cerr << "FNV1A64=" << std::hex << h << std::dec << "\n";
    }

    return 0;
}