"""Configuration-dependent control effectiveness matrix B(q_art).

B maps per-rotor thrust commands to body wrench [Fx, Fy, Fz, Mx, My, Mz].
Articulation cant β tilts each nacelle thrust axis; lever arms update with β.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from arw_articulated_twin.geometry import RotorStation, rotor_layout


@dataclass
class EffectivenessResult:
    b_matrix: np.ndarray  # (6, n_rotors)
    sigma_max: float
    sigma_min: float
    condition_number: float
    beta_deg: np.ndarray


def _spin_sign(spin: str) -> float:
    return 1.0 if spin.upper() == "CW" else -1.0


def rotor_thrust_axes(
    stations: list[RotorStation],
    beta_per_annulus: dict[str, float],
    spread_k: float = 0.62,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return positions (n,3), thrust directions (n,3), effectiveness scalars (n,)."""
    n = len(stations)
    pos = np.zeros((n, 3), dtype=np.float64)
    direction = np.zeros((n, 3), dtype=np.float64)
    eff = np.zeros(n, dtype=np.float64)
    max_arm = max(s.lever_arm_m for s in stations) if stations else 1.0

    for i, st in enumerate(stations):
        beta = math.radians(beta_per_annulus.get(st.annulus, 0.0))
        pos[i] = (st.x_m, st.y_m, st.z_m)
        # Thrust along local +Z body axis, tilted by ring cant β about Y (pitch ring).
        direction[i] = (math.sin(beta), 0.0, math.cos(beta))
        spread = 1.0 + spread_k * math.sin(beta) * (st.lever_arm_m / max_arm)
        eff[i] = spread
    return pos, direction, eff


def build_b_matrix(
    vehicle_code: str,
    beta_per_annulus: dict[str, float] | None = None,
    spread_k: float = 0.62,
) -> EffectivenessResult:
    """Assemble 6×N effectiveness matrix for current articulation state."""
    stations = rotor_layout(vehicle_code)
    if beta_per_annulus is None:
        annuli = sorted({s.annulus for s in stations})
        beta_per_annulus = {a: 0.0 for a in annuli}

    pos, direction, eff = rotor_thrust_axes(stations, beta_per_annulus, spread_k)
    n = len(stations)
    b = np.zeros((6, n), dtype=np.float64)

    for i, st in enumerate(stations):
        t = direction[i] * eff[i]
        r = pos[i]
        moment = np.cross(r, t)
        b[0:3, i] = t
        b[3:6, i] = moment

    # Hover-relevant wrench rows (Fz, Mx, My). Mz requires differential rotor torque / drag-yaw
    # and is zero for collinear thrust columns at β≈0.
    b_ctrl = b[2:5, :]
    norms = np.linalg.norm(b_ctrl, axis=0)
    norms = np.where(norms > 1e-12, norms, 1.0)
    bn = b_ctrl / norms
    gram = bn @ bn.T
    s = np.linalg.svd(gram, compute_uv=False)
    sigma_max = float(s[0]) if len(s) else 0.0
    sigma_min = float(s[-1]) if len(s) else 0.0
    cond = sigma_max / max(sigma_min, 1e-12)

    beta_arr = np.array([beta_per_annulus.get(s.annulus, 0.0) for s in stations])
    return EffectivenessResult(
        b_matrix=b,
        sigma_max=sigma_max,
        sigma_min=sigma_min,
        condition_number=cond,
        beta_deg=beta_arr,
    )


def sweep_articulation(
    vehicle_code: str,
    beta_deg: np.ndarray,
    annulus: str = "A",
) -> list[dict[str, float]]:
    """Log σ_max/σ_min vs articulation angle (validation §8)."""
    rows: list[dict[str, float]] = []
    for beta in beta_deg:
        result = build_b_matrix(vehicle_code, {annulus: float(beta)})
        rows.append(
            {
                "beta_deg": float(beta),
                "sigma_max": result.sigma_max,
                "sigma_min": result.sigma_min,
                "condition_number": result.condition_number,
            }
        )
    return rows
