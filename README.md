realtime-ballistics-sim

Deterministic real-time projectile simulation and fire-control demonstration
C++17, CMake, Ninja, MSYS2 UCRT64

This project implements a systems-oriented ballistic simulation core with continuous collision detection, material response modeling, and a deterministic fire-control lead solver. It includes reproducibility guarantees via FNV-1a hashing and a golden smoke test.

Engineering Scope

This is not a game prototype. It is a minimal simulation engine demo focusing on deterministic fixed-timestep integration, quadratic drag and gravity, continuous collision detection between segments and sphere or AABB targets, deterministic material response based on penetration thresholds, impact event reporting, a fire-control lead solver for moving targets in 2D, continuous-time closest approach validation, and reproducibility verification using golden hash tests.

Core Features

Deterministic Ballistic Simulation

The simulation uses a semi-implicit Euler integrator with fixed timestep.
Quadratic air drag can be enabled or disabled.
Gravity follows a Y-up convention.
Ground collision handling is included.
Trajectory data is written as CSV to standard output.

Continuous Collision Detection

The engine performs segment versus sphere intersection tests and segment versus axis-aligned bounding box intersection tests.
The earliest hit within a timestep is selected deterministically.
Material response is energy-based and fully deterministic.
Impact events are logged into impacts.csv.

Material Model

Each target defines a material name and a penetration energy threshold expressed in Joules.
If kinetic energy is less than or equal to the threshold the projectile stops.
If kinetic energy exceeds the threshold the projectile penetrates and remaining energy is computed deterministically.

Fire-Control Lead Solver (2D)

The fire-control module solves a moving target interception problem in the X-Y plane.
It performs a simulation-based search over elevation angles.
Continuous-time closest approach is evaluated for accuracy.
The solution is deterministic.
A summary CSV is written to standard output.
A detailed execution report is written to fc_report.csv.

Build (Windows / MSYS2 UCRT64)

Prerequisites

Install the following MSYS2 UCRT64 packages:

mingw-w64-ucrt-x86_64-toolchain
mingw-w64-ucrt-x86_64-cmake
mingw-w64-ucrt-x86_64-ninja

Configure and Build

cd /c/Users/AliEray/Desktop/Staj-Proje/realtime-ballistics-sim
cmake -S . -B build -G Ninja
cmake --build build -j

The executable will be generated at:

build/ballistics.exe

Run Modes

SIM Mode writes trajectory CSV to standard output. Example usage:

./build/ballistics.exe --mode sim --scenario hit --hash > traj_hit.csv

This produces trajectory CSV on stdout, creates impacts.csv when targets exist, and prints a FNV1A64 hash to stderr when hash mode is enabled.

FC Mode solves the fire-control lead problem. Example usage:

./build/ballistics.exe --mode fc --t 6 --drag 0 --v0 220 --y0 1.5 --tx 80 --ty 1 --tvx -6 --tvy 0 --hit_radius 0.5 --hash

This produces a summary CSV on stdout, creates fc_report.csv, and prints a FNV1A64 hash to stderr.

Determinism Verification

Run:

./scripts/smoke.sh

Expected output includes SIM_HIT_HASH=OK, FC_HASH=OK, and ALL_TESTS_PASS.
If hashes change, simulation behavior or output formatting has changed.

Repository Structure

src/sim/vec3.h contains math primitives.
src/sim/ballistics.* contains the core physics integrator.
src/sim/geometry.* contains intersection helpers.
src/sim/collision.* contains collision logic.
src/sim/material.h defines the material model.
src/sim/target.h defines target types.
src/sim/fire_control.* implements the lead solver.
scripts/smoke.sh runs the golden determinism test.

Design Philosophy

The project prioritizes deterministic simulation behavior, explicit reproducibility testing, clean system boundaries, and systems-level engineering clarity over visualization or user interface elements.

License

MIT

Author

Ali Eray Kalaycı
Computer Engineering – Systems and Real-Time Systems Focus