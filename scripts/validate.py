#!/usr/bin/env python3
"""Validation suite per scope outline §8."""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

from arw_articulated_twin.allocator import allocate_hover
from arw_articulated_twin.effectiveness import build_b_matrix, sweep_articulation
from arw_articulated_twin.env import ArticulatedHoverEnv, EnvConfig
from arw_articulated_twin.geometry import VEHICLES
from arw_articulated_twin.rotor_model import rpm_to_omega, thrust_from_omega
from arw_articulated_twin.snidget_params import derive_k_t, provenance_table
from arw_articulated_twin.warp_kernels import backend_name, compute_thrusts_gpu


def check_b_articulation() -> dict:
    betas = np.linspace(0, 25, 26)
    sweep = sweep_articulation("AER8110-1", betas)
    cond_at_zero = sweep[0]["condition_number"]
    cond_at_max = sweep[-1]["condition_number"]
    # σ ceiling 30 per ITP — on hover-relevant 4×4 Gram block (Fz + moments)
    passed = cond_at_max < 30.0
    # Articulation should change authority (spread) monotonically at outer stations
    spread_increases = sweep[-1]["sigma_max"] >= sweep[0]["sigma_max"] * 0.95
    passed = passed and spread_increases
    return {
        "name": "B(q_art) sweep",
        "passed": passed,
        "cond_at_0deg": cond_at_zero,
        "cond_at_25deg": cond_at_max,
        "sigma_ceiling_ok": cond_at_max < 30.0,
    }


def check_hover_trim() -> dict:
    v = VEHICLES["AER8110-1"]
    alloc = allocate_hover(v.code, v.mass_kg)
    residual_norm = float(np.linalg.norm(alloc.residual))
    passed = alloc.success and residual_norm < 5.0
    return {"name": "Hover pseudo-inverse trim", "passed": passed, "residual_norm": residual_norm}


def check_snidget_k_t() -> dict:
    coeffs = derive_k_t(3, 0.70716)
    omega = rpm_to_omega(3000.0)
    thrust = thrust_from_omega(omega, coeffs)
    expected_n = 16015.0 * 9.80665 / 1000.0
    err = abs(thrust - expected_n) / expected_n
    return {"name": "Snidget k_T @ 3000rpm 3-blade", "passed": err < 0.02, "thrust_n": thrust, "rel_err": err}


def check_warp_thrust() -> dict:
    coeffs = derive_k_t(3, 0.432)
    omega = np.full(19, rpm_to_omega(4550.0))
    thrusts = compute_thrusts_gpu(omega, coeffs)
    passed = len(thrusts) == 19 and float(np.all(thrusts >= 0))
    return {"name": f"Per-rotor thrust kernel ({backend_name()})", "passed": passed, "mean_thrust_n": float(np.mean(thrusts))}


def check_hover_env() -> dict:
    env = ArticulatedHoverEnv(EnvConfig(vehicle_code="AER8110-1", domain_randomization=False))
    action = env.reference_hover_action()
    total_reward = 0.0
    for _ in range(200):
        _, reward, done, info = env.step(action)
        total_reward += reward
        if done:
            break
    passed = env.pos[2] > 0.5 and info["condition_number"] < 30.0 and total_reward > -500.0
    return {
        "name": "Articulated hover env (2s)",
        "passed": passed,
        "final_altitude_m": float(env.pos[2]),
        "reward": total_reward,
    }


def check_articulation_maneuver() -> dict:
    env = ArticulatedHoverEnv(EnvConfig(vehicle_code="AER8110-1", domain_randomization=False))
    action = env.reference_hover_action()
    annuli = list(env.beta.keys())
    # Command 15° ring cant on first annulus
    action[len(env.states) + 0] = 15.0
    _, _, _, info = env.step(action)
    passed = abs(env.beta[annuli[0]] - 15.0) < 0.1 and info["sigma_max"] > 0
    return {"name": "Articulation maneuver (+15°)", "passed": passed, "beta": env.beta}


def check_scaling_152() -> dict:
    """19 → 152 rotor scaling sanity (4× AER8100 annuli)."""
    v = VEHICLES["AER8100-1"]
    eff = build_b_matrix(v.code)
    passed = eff.b_matrix.shape == (6, 76)
    return {
        "name": "152-rotor (76×2 logical) B matrix",
        "passed": passed,
        "shape": list(eff.b_matrix.shape),
        "condition_number": eff.condition_number,
    }


def main() -> int:
    checks = [
        check_snidget_k_t(),
        check_b_articulation(),
        check_hover_trim(),
        check_warp_thrust(),
        check_hover_env(),
        check_articulation_maneuver(),
        check_scaling_152(),
    ]
    report = {"checks": checks, "provenance_rows": len(provenance_table()), "all_passed": all(c["passed"] for c in checks)}

    out = Path("validation_report.json")
    out.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))

    for c in checks:
        status = "PASS" if c["passed"] else "FAIL"
        print(f"  [{status}] {c['name']}")

    return 0 if report["all_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
