#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <cstdint>
#include <cstdlib>

#include "sim/ballistics.h"

static void usage() {
    std::cout <<
        "Usage:\n"
        "  ballistics [--dt <seconds>] [--t <seconds>] [--drag 0|1]\n"
        "             [--v0 <mps>] [--elev <deg>] [--y0 <m>] [--hash]\n"
        "\n"
        "Example:\n"
        "  ballistics --dt 0.0166667 --t 8 --drag 1 --v0 260 --elev 8 --y0 1.5 --hash\n";
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

// FNV-1a 64-bit hash over the CSV bytes we emit (determinism proof)
static uint64_t fnv1a64_update(uint64_t h, const void* data, size_t n) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    for (size_t k = 0; k < n; ++k) {
        h ^= p[k];
        h *= 1099511628211ULL;
    }
    return h;
}

int main(int argc, char** argv) {
    using namespace sim;

    // Defaults
    double dt = 1.0 / 60.0;
    double sim_time_s = 8.0;
    int drag_on = 1;

    double v0 = 250.0;     // initial speed [m/s]
    double elev_deg = 8.0; // elevation angle [deg]
    double y0 = 1.5;       // launch height [m]

    bool print_hash = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            usage();
            return 0;
        } else if (a == "--dt") {
            if (!parse_double(i, argc, argv, dt)) { usage(); return 2; }
        } else if (a == "--t") {
            if (!parse_double(i, argc, argv, sim_time_s)) { usage(); return 2; }
        } else if (a == "--drag") {
            if (!parse_int(i, argc, argv, drag_on)) { usage(); return 2; }
        } else if (a == "--v0") {
            if (!parse_double(i, argc, argv, v0)) { usage(); return 2; }
        } else if (a == "--elev") {
            if (!parse_double(i, argc, argv, elev_deg)) { usage(); return 2; }
        } else if (a == "--y0") {
            if (!parse_double(i, argc, argv, y0)) { usage(); return 2; }
        } else if (a == "--hash") {
            print_hash = true;
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            usage();
            return 2;
        }
    }

    SimConfig cfg;
    cfg.dt = dt;
    cfg.gravity = {0.0, -9.80665, 0.0};
    cfg.air_density = 1.225;
    cfg.enable_drag = (drag_on != 0);
    cfg.ground_collision = true;

    Projectile p;
    p.pos_m = {0.0, y0, 0.0};

    // Convert elevation angle to velocity components (x forward, y up)
    const double deg2rad = 3.14159265358979323846 / 180.0;
    const double th = elev_deg * deg2rad;
    p.vel_mps = {v0 * std::cos(th), v0 * std::sin(th), 0.0};

    // Representative projectile parameters (can become CLI later)
    p.mass_kg = 0.0095;
    p.area_m2 = 6.0e-5;
    p.cd = 0.295;
    p.restitution = 0.0;

    const int steps = (dt > 0.0) ? static_cast<int>(sim_time_s / dt) : 0;
    if (steps <= 0) {
        std::cerr << "Invalid dt or t\n";
        return 2;
    }

    uint64_t h = 1469598103934665603ULL;

    auto emit = [&](const std::string& s) {
        std::cout << s;
        h = fnv1a64_update(h, s.data(), s.size());
    };

    // Header
    emit("t,x,y,z,vx,vy,vz,speed,alive\n");

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

        step_projectile(p, cfg);
        t += dt;
    }

    if (print_hash) {
        // Print hash to stderr so it doesn't contaminate CSV
        std::cerr << "FNV1A64=" << std::hex << h << std::dec << "\n";
    }

    return 0;
}