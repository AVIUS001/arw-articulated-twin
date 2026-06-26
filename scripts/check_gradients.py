#!/usr/bin/env python3
"""Differentiable-physics checks (scope §8 DoD).

Two real checks:

  1. Finite-difference vs analytic gradient of the hover allocation objective
         J(ω) = ½ ‖ B(β)·T(ω) − w_des ‖²,  T_i(ω) = k_T ω_i²
     through the Snidget thrust model and B(q_art). Central-difference vs the
     closed-form gradient   ∂J/∂ω_i = (Bᵀ r)_i · 2 k_T ω_i,  r = B·T − w_des.

  2. B(q_art) is genuinely configuration-dependent: the condition number must
     MOVE across an articulation sweep (guards against the constant-σ bug).

  3. (Optional) NVIDIA Warp autodiff cross-check on the thrust kernel, when
     warp-lang is installed; skipped cleanly otherwise.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from arw_articulated_twin.effectiveness import CONTROLLED_DOF_ROWS, build_b_matrix, sweep_articulation
from arw_articulated_twin.geometry import VEHICLES
from arw_articulated_twin.snidget_params import derive_k_t


def _objective(omega: np.ndarray, b_ctrl: np.ndarray, k_t: float, w_des: np.ndarray) -> float:
    thrust = k_t * omega * omega
    r = b_ctrl @ thrust - w_des
    return 0.5 * float(r @ r)


def _analytic_grad(omega: np.ndarray, b_ctrl: np.ndarray, k_t: float, w_des: np.ndarray) -> np.ndarray:
    thrust = k_t * omega * omega
    r = b_ctrl @ thrust - w_des           # (4,)
    dJ_dthrust = b_ctrl.T @ r              # (N,)
    dthrust_domega = 2.0 * k_t * omega     # (N,)
    return dJ_dthrust * dthrust_domega


def check_finite_difference_gradient(vehicle="AER8110-1", h=1e-3, tol=1e-5) -> dict:
    v = VEHICLES[vehicle]
    eff = build_b_matrix(v.code, {"A": 12.0})          # articulated state, non-trivial B
    b_ctrl = eff.b_matrix[list(CONTROLLED_DOF_ROWS), :]
    coeffs = derive_k_t(v.blade_count, v.rotor_diameter_m)
    k_t = coeffs.k_t

    rng = np.random.default_rng(0)
    omega = (v.hover_rpm * 2 * np.pi / 60.0) * (1.0 + 0.05 * rng.standard_normal(eff.b_matrix.shape[1]))
    w_des = np.array([v.mass_kg * 9.80665, 0.0, 0.0, 0.0])  # [Fz, Mx, My, Mz]

    g_analytic = _analytic_grad(omega, b_ctrl, k_t, w_des)
    g_fd = np.zeros_like(omega)
    for i in range(len(omega)):
        op, om = omega.copy(), omega.copy()
        op[i] += h
        om[i] -= h
        g_fd[i] = (_objective(op, b_ctrl, k_t, w_des) - _objective(om, b_ctrl, k_t, w_des)) / (2 * h)

    denom = np.maximum(np.abs(g_analytic), 1.0)
    rel_err = float(np.max(np.abs(g_fd - g_analytic) / denom))
    return {"name": "FD vs analytic gradient", "passed": rel_err < tol, "max_rel_err": rel_err}


def check_articulation_varies(vehicle="AER8110-1", min_rel_change=0.02) -> dict:
    sweep = sweep_articulation(vehicle, np.linspace(0, 25, 26))
    conds = np.array([r["condition_number"] for r in sweep])
    finite = conds[np.isfinite(conds)]
    rel_change = float((finite.max() - finite.min()) / max(finite.min(), 1e-9))
    passed = rel_change > min_rel_change and bool(np.all(finite < 30.0))
    return {
        "name": "B(q_art) varies with articulation",
        "passed": passed,
        "cond_0deg": float(conds[0]),
        "cond_25deg": float(conds[-1]),
        "rel_change": rel_change,
        "ceiling_ok": bool(np.all(finite < 30.0)),
    }


def check_warp_autodiff() -> dict:
    try:
        import warp as wp
        wp.init()
    except Exception as exc:  # noqa: BLE001
        return {"name": "Warp autodiff cross-check", "passed": True, "skipped": True, "reason": str(exc)[:60]}

    k_t = 3.0e-3
    n = 19

    @wp.kernel
    def thrust_sum(omega: wp.array(dtype=float), k_t: float, loss: wp.array(dtype=float)):
        i = wp.tid()
        wp.atomic_add(loss, 0, k_t * omega[i] * omega[i])

    omega_np = np.full(n, 400.0, dtype=np.float32)
    omega = wp.array(omega_np, dtype=float, requires_grad=True)
    loss = wp.zeros(1, dtype=float, requires_grad=True)
    tape = wp.Tape()
    with tape:
        wp.launch(thrust_sum, dim=n, inputs=[omega, k_t, loss])
    tape.backward(loss=loss)
    g_warp = omega.grad.numpy()
    g_analytic = 2.0 * k_t * omega_np
    rel = float(np.max(np.abs(g_warp - g_analytic) / np.maximum(np.abs(g_analytic), 1.0)))
    return {"name": "Warp autodiff cross-check", "passed": rel < 1e-4, "skipped": False, "max_rel_err": rel}


def main() -> int:
    checks = [check_finite_difference_gradient(), check_articulation_varies(), check_warp_autodiff()]
    ok = all(c["passed"] for c in checks)
    for c in checks:
        tag = "SKIP" if c.get("skipped") else ("PASS" if c["passed"] else "FAIL")
        extra = {k: v for k, v in c.items() if k not in ("name", "passed", "skipped")}
        print(f"  [{tag}] {c['name']}  {extra}")
    print(f"\nall_passed={ok}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
