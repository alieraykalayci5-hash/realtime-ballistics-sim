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
#include "sim/fire_control.h"

struct Args {
    std::string mode = "sim";         // sim | fc
    std::string scenario = "demo";    // demo | hit | none (sim mode)

    double dt = 1.0 / 60.0;
    double sim_time_s = 8.0;
    int drag_on = 1;

    double v0 = 260.0;
    double elev_deg = 8.0;
    double y0 = 1.5;

    // Fire-control target (fc mode) - x/y plane, z=0
    double tx = 60.0;
    double ty = 1.0;
    double tvx = -5.0;
    double tvy = 0.0;

    double hit_radius = 0.5;

    // Fire-control report output file
    std::string fc_out = "fc_report.csv";

    bool print_hash = false;
};

static void usage() {
    std::cout <<
        "Usage:\n"
        "  ballistics --mode sim [--scenario demo|hit|none] [sim params...] [--hash] > traj.csv\n"
        "  ballistics --mode fc  [fc params...] [--fc_out <file>] [--hash]\n"
        "\n"
        "SIM params:\n"
        "  --dt <sec> --t <sec> --drag 0|1 --v0 <m/s> --elev <deg> --y0 <m>\n"
        "  --scenario demo|hit|none\n"
        "\n"
        "FC params (moving target in x-y plane):\n"
        "  --dt <sec> --t <sec> --drag 0|1 --v0 <m/s> --y0 <m>\n"
        "  --tx <m> --ty <m> --tvx <m/s> --tvy <m/s> --hit_radius <m>\n"
        "  --fc_out <file>   (default: fc_report.csv)\n"
        "\n"
        "Outputs:\n"
        "  SIM: trajectory CSV to stdout + impacts.csv file\n"
        "  FC : summary CSV line to stdout + fc_report.csv file\n"
        "\n"
        "Examples:\n"
        "  ballistics --mode sim --scenario hit --hash > traj_hit.csv\n"
        "  ballistics --mode fc --t 6 --drag 0 --v0 220 --y0 1.5 --tx 80 --ty 1 --tvx -6 --tvy 0 --hit_radius 0.5 --hash\n";
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
        } else if (s == "--mode") {
            if (i + 1 >= argc) return false;
            a.mode = argv[i + 1];
            i++;
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
        } else if (s == "--tx") {
            if (!parse_double(i, argc, argv, a.tx)) return false;
        } else if (s == "--ty") {
            if (!parse_double(i, argc, argv, a.ty)) return false;
        } else if (s == "--tvx") {
            if (!parse_double(i, argc, argv, a.tvx)) return false;
        } else if (s == "--tvy") {
            if (!parse_double(i, argc, argv, a.tvy)) return false;
        } else if (s == "--hit_radius") {
            if (!parse_double(i, argc, argv, a.hit_radius)) return false;
        } else if (s == "--fc_out") {
            if (i + 1 >= argc) return false;
            a.fc_out = argv[i + 1];
            i++;
        } else if (s == "--hash") {
            a.print_hash = true;
        } else {
            std::cerr << "Unknown arg: " << s << "\n";
            return false;
        }
    }
    return true;
}

static void apply_scenario_defaults(const std::string& scenario, Args& a) {
    if (scenario == "hit") {
        a.drag_on = 0;
        a.sim_time_s = 3.0;
        a.v0 = 120.0;
        a.elev_deg = 0.0;
        a.y0 = 1.0;
        a.dt = 1.0 / 60.0;
    } else if (scenario == "demo") {
        // keep defaults
    } else if (scenario == "none") {
        // keep defaults
    }
}

static std::vector<sim::Target> make_targets_for_scenario(const std::string& name) {
    using namespace sim;

    static const Material steel{"steel", 1800.0};
    static const Material wood {"wood",   250.0};

    if (name == "none") return {};

    if (name == "hit") {
        return {
            Target::make_sphere({20.0, 1.0, 0.0}, 0.6, wood),
            Target::make_aabb  ({40.0, 0.0, -0.5}, {42.0, 2.0, 0.5}, steel)
        };
    }

    // demo
    return {
        Target::make_sphere({40.0, 1.0, 0.0}, 0.5, wood),
        Target::make_aabb  ({70.0, 0.0, -0.5}, {72.0, 2.0, 0.5}, steel)
    };
}

static int run_sim(const Args& args) {
    using namespace sim;

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

    emit("t,x,y,z,vx,vy,vz,speed,alive\n");

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

static int run_fc(const Args& args) {
    using namespace sim;

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

    Target2D tgt;
    tgt.pos_m = {args.tx, args.ty, 0.0};
    tgt.vel_mps = {args.tvx, args.tvy, 0.0};

    FireControlConfig fc;
    fc.hit_radius_m = args.hit_radius;

    FireControlResult r = solve_lead_2d(cfg, args.sim_time_s, args.v0, args.y0, tgt, fc);

    // Build deterministic stdout summary (CSV)
    std::ostringstream summary;
    summary.setf(std::ios::fixed);
    summary << std::setprecision(6);
    summary << "success,lead_elev_deg,t_hit_s,miss_m,eval_count,proj_x,proj_y,tgt_x,tgt_y\n";
    summary << (r.success ? 1 : 0) << ","
            << r.lead_elev_deg << ","
            << r.t_hit_s << ","
            << r.miss_m << ","
            << r.eval_count << ","
            << r.proj_at_hit_m.x << ","
            << r.proj_at_hit_m.y << ","
            << r.tgt_at_hit_m.x << ","
            << r.tgt_at_hit_m.y << "\n";

    const std::string s_summary = summary.str();
    std::cout << s_summary;

    // Write detailed FC report file (single-run artifact)
    std::ostringstream report;
    report.setf(std::ios::fixed);
    report << std::setprecision(6);

    report << "dt,sim_time_s,drag_on,v0,y0,tx,ty,tvx,tvy,hit_radius,success,lead_elev_deg,t_hit_s,miss_m,eval_count,proj_x,proj_y,tgt_x,tgt_y\n";
    report << cfg.dt << ","
           << args.sim_time_s << ","
           << (args.drag_on != 0 ? 1 : 0) << ","
           << args.v0 << ","
           << args.y0 << ","
           << args.tx << ","
           << args.ty << ","
           << args.tvx << ","
           << args.tvy << ","
           << args.hit_radius << ","
           << (r.success ? 1 : 0) << ","
           << r.lead_elev_deg << ","
           << r.t_hit_s << ","
           << r.miss_m << ","
           << r.eval_count << ","
           << r.proj_at_hit_m.x << ","
           << r.proj_at_hit_m.y << ","
           << r.tgt_at_hit_m.x << ","
           << r.tgt_at_hit_m.y << "\n";

    const std::string s_report = report.str();

    {
        std::ofstream f(args.fc_out, std::ios::binary);
        if (!f) {
            std::cerr << "Failed to write " << args.fc_out << "\n";
            return 2;
        }
        f << s_report;
    }

    // Hash determinism for FC mode:
    // hash over both summary(stdout text) + report(file text)
    if (args.print_hash) {
        uint64_t h = 1469598103934665603ULL;
        h = fnv1a64_update(h, s_summary.data(), s_summary.size());
        h = fnv1a64_update(h, s_report.data(), s_report.size());
        std::cerr << "FNV1A64=" << std::hex << h << std::dec << "\n";
    }

    return 0;
}

int main(int argc, char** argv) {
    Args args;

    // sniff mode/scenario for defaults
    std::string scenario_from_cli = "demo";
    std::string mode_from_cli = "sim";

    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if (s == "--mode" && i + 1 < argc) mode_from_cli = argv[i + 1];
        if (s == "--scenario" && i + 1 < argc) scenario_from_cli = argv[i + 1];
    }

    args.mode = mode_from_cli;
    args.scenario = scenario_from_cli;

    if (args.mode == "sim") {
        apply_scenario_defaults(args.scenario, args);
    }

    if (!parse_args(argc, argv, args)) {
        usage();
        return 2;
    }

    if (args.mode != "sim" && args.mode != "fc") {
        std::cerr << "Invalid --mode: " << args.mode << "\n";
        usage();
        return 2;
    }

    if (args.mode == "sim") {
        if (args.scenario != "demo" && args.scenario != "hit" && args.scenario != "none") {
            std::cerr << "Invalid --scenario: " << args.scenario << "\n";
            usage();
            return 2;
        }
        return run_sim(args);
    }

    return run_fc(args);
}