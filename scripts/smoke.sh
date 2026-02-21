#!/usr/bin/env bash
set -euo pipefail

# Always run from repo root
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "[1/3] Build"
cmake --build build -j

# Helper: run command, capture FNV from stderr
run_and_get_fnv() {
  # We capture stderr, grep FNV, and return only the hex value.
  # stdout is discarded (or redirected by caller).
  local out
  out="$("$@" 1>/dev/null 2>&1 || true)"
  echo "$out" | sed -n 's/^FNV1A64=\([0-9a-fA-F]\+\)$/\1/p' | tail -n 1
}

echo "[2/3] SIM determinism (scenario=hit)"
SIM_HASH="$(./build/ballistics.exe --mode sim --scenario hit --hash > /dev/null 2> >(cat >&2) ; true)"
# The line above prints hash to stderr; we parse again in a safer way:
SIM_FNV="$( (./build/ballistics.exe --mode sim --scenario hit --hash > /dev/null) 2>&1 | sed -n 's/^FNV1A64=\([0-9a-fA-F]\+\)$/\1/p' | tail -n 1 )"
if [[ -z "${SIM_FNV}" ]]; then
  echo "SIM_HIT_HASH=FAIL (no hash produced)"
  exit 1
fi

# NOTE: Set expected values once, then keep stable.
# Update these expected hashes only if you intentionally change the sim output format/logic.
EXPECTED_SIM_FNV="${EXPECTED_SIM_FNV:-1ab5a34f950442b3}"

if [[ "${SIM_FNV,,}" == "${EXPECTED_SIM_FNV,,}" ]]; then
  echo "SIM_HIT_HASH=OK (${SIM_FNV})"
else
  echo "SIM_HIT_HASH=FAIL (got=${SIM_FNV} expected=${EXPECTED_SIM_FNV})"
  exit 1
fi

echo "[3/3] FC determinism"
FC_FNV="$( (./build/ballistics.exe --mode fc --t 6 --drag 0 --v0 220 --y0 1.5 --tx 80 --ty 1 --tvx -6 --tvy 0 --hit_radius 0.5 --hash > /dev/null) 2>&1 | sed -n 's/^FNV1A64=\([0-9a-fA-F]\+\)$/\1/p' | tail -n 1 )"
if [[ -z "${FC_FNV}" ]]; then
  echo "FC_HASH=FAIL (no hash produced)"
  exit 1
fi

EXPECTED_FC_FNV="${EXPECTED_FC_FNV:-4013fed8c19bce70}"

if [[ "${FC_FNV,,}" == "${EXPECTED_FC_FNV,,}" ]]; then
  echo "FC_HASH=OK (${FC_FNV})"
else
  echo "FC_HASH=FAIL (got=${FC_FNV} expected=${EXPECTED_FC_FNV})"
  exit 1
fi

echo "ALL_TESTS_PASS"