# Real-Time Ballistics Simulation Engine

![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)
![Deterministic](https://img.shields.io/badge/Simulation-Deterministic-success)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow)

Deterministic real-time projectile simulation and fire-control demonstration engine implemented in C++17.

This project implements:

* Fixed-timestep ballistic simulation
* Semi-implicit Euler integration
* Optional quadratic air drag
* Continuous collision detection (segment-based)
* Energy-based deterministic material response
* Structured impact logging
* Deterministic 2D fire-control lead solver
* FNV-1a 64-bit golden hash regression verification

The engine is intentionally systems-oriented. It does not focus on graphics or UI; instead it emphasizes numerical stability, deterministic execution, modular architecture, and regression-detectable behavior.

---

## Quick Start

Minimal build and run example:

```bash
cmake -S . -B build -G Ninja
cmake --build build -j
./build/ballistics.exe --mode sim --scenario hit --hash
```

---

## Example Output (SIM – Hit Scenario)

Expected hash (smoke reference):

```text
SIM_HIT_HASH=OK
```

Trajectory data is written to stdout as CSV.
If targets are present, `impacts.csv` is generated.

---

## System Overview

This repository implements a deterministic projectile simulation core designed to demonstrate systems-level engineering principles:

* Fixed timestep evolution
* Deterministic floating-point evaluation order
* Explicit collision resolution
* Reproducible energy-based material modeling
* Regression detection via golden hashing

Two execution modes are supported:

* **SIM Mode** – Projectile trajectory simulation
* **FC Mode** – Fire-control interception solving

---

## Core Physics Model

Projectile motion is propagated using a semi-implicit Euler integrator under a fixed timestep.

### State Representation

* Position: `(x, y, z)`
* Velocity: `(vx, vy, vz)`

### Dynamics

* Constant gravitational acceleration (Y-up convention)
* Optional quadratic air drag proportional to `|v|v`
* Deterministic ground collision handling
* No stochastic perturbations

Under identical parameters and seeds, state evolution is bitwise reproducible.

---

## Continuous Collision Detection

Discrete collision checks are insufficient for high-velocity projectiles. Each timestep constructs a motion segment:

* Previous position → Current position

Supported intersection tests:

* Segment vs Sphere
* Segment vs Axis-Aligned Bounding Box (AABB)

For each timestep:

1. All candidate intersections are evaluated.
2. The earliest valid hit is selected deterministically.
3. Material response is applied.
4. Impact event is logged.

### Impact Log Format (`impacts.csv`)

```text
t, material, target, ke_j, threshold_j, result, x, y, z
```

---

## Material Response Model

Each target defines:

* Material name
* Penetration energy threshold (Joules)

Deterministic rule set:

* If `KE <= threshold` → projectile stops
* If `KE > threshold` → projectile penetrates and residual energy is computed deterministically

No randomness is involved in penetration behavior.

---

## Fire-Control Lead Solver (2D)

The fire-control module solves a moving-target interception problem in the X–Y plane.

Features:

* Configurable target initial position and velocity
* Simulation-based search over elevation angles
* Continuous-time closest approach validation
* Deterministic convergence
* Summary CSV output to stdout
* Detailed report written to `fc_report.csv`

### Example Summary Output

```text
success, lead_elev_deg, t_hit_s, miss_m, eval_count, proj_x, proj_y, tgt_x, tgt_y
```

---

## Determinism & Reproducibility

Both SIM and FC modes compute an FNV-1a 64-bit hash over output data.

This enables:

* Regression detection across commits
* Verification of deterministic behavior
* Detection of formatting or logic changes

### Golden Smoke Test

```bash
./scripts/smoke.sh
```

Expected:

```text
SIM_HIT_HASH=OK
FC_HASH=OK
ALL_TESTS_PASS
```

If hashes change, simulation behavior or output formatting changed.

---

## Repository Structure

```text
src/
  sim/
    vec3.h
    ballistics.*
    geometry.*
    collision.*
    material.h
    target.h
    fire_control.*

scripts/
  smoke.sh
```

---

## Build (Windows – MSYS2 UCRT64 Example)

### Install Dependencies

```bash
pacman -S --needed \
  mingw-w64-ucrt-x86_64-toolchain \
  mingw-w64-ucrt-x86_64-cmake \
  mingw-w64-ucrt-x86_64-ninja
```

### Configure and Build

```bash
cd /c/Users/AliEray/Desktop/Staj-Proje/realtime-ballistics-sim
cmake -S . -B build -G Ninja
cmake --build build -j
```

Executable:

```text
build/ballistics.exe
```

---

## Running the Simulation

### SIM Mode

```bash
./build/ballistics.exe \
  --mode sim \
  --scenario hit \
  --hash > traj_hit.csv
```

Outputs:

* Trajectory CSV → stdout
* `impacts.csv` (if collision occurs)
* FNV1A64 hash → stderr (when `--hash` enabled)

Available scenarios: `demo`, `hit`, `none`

---

### FC Mode

```bash
./build/ballistics.exe \
  --mode fc \
  --t 6 \
  --drag 0 \
  --v0 220 \
  --y0 1.5 \
  --tx 80 \
  --ty 1 \
  --tvx -6 \
  --tvy 0 \
  --hit_radius 0.5 \
  --hash
```

Outputs:

* Summary CSV → stdout
* `fc_report.csv`
* FNV1A64 hash → stderr

---

## CLI Parameters (Selected)

| Parameter    | Description                    |
| ------------ | ------------------------------ |
| --mode       | sim / fc                       |
| --scenario   | Predefined simulation scenario |
| --t          | Simulation time (FC mode)      |
| --v0         | Initial muzzle velocity        |
| --drag       | Enable quadratic drag (0/1)    |
| --y0         | Initial projectile height      |
| --tx, --ty   | Target initial position        |
| --tvx, --tvy | Target velocity                |
| --hit_radius | Interception tolerance         |
| --hash       | Enable FNV-1a output hashing   |

---

## Performance Characteristics

Example (hit scenario, no drag):

```text
Deterministic fixed timestep
Bitwise-reproducible under identical parameters
Golden hash verified via smoke test
```

Performance is stable and reproducible under identical execution conditions.

---

## Engineering Highlights

* Fully deterministic fixed-timestep simulation
* Continuous collision detection (segment-based)
* Energy-consistent penetration modeling
* Modular separation (physics / geometry / collision / fire-control)
* Regression-detecting FNV-1a golden hash
* Systems-oriented design (no hidden stochastic behavior)
* Clean CMake + Ninja build pipeline

---

## Technologies

* C++17
* CMake
* Ninja
* MSYS2 UCRT64
* Custom FNV-1a 64-bit hashing

---

## License

MIT

---

## Author

Ali Eray Kalaycı
Computer Engineering
Focus: Real-Time Systems, Simulation, Tracking & Estimation
