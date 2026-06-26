"""Configuration-dependent control effectiveness matrix B(q_art).

Each rotor column maps a unit thrust command (N) to a body wrench
[Fx, Fy, Fz, Mx, My, Mz]:

    n_i      = (sin β_i, 0, cos β_i)              # ring-cant thrust axis
    F_i      = n_i                                # force per unit thrust
    M_i      = r_i × n_i + spin_i · κ · n_i       # lever moment + reaction-drag yaw

κ = k_Q / k_T is the per-thrust reaction-torque arm (m), giving yaw authority
from differential rotor drag even at β = 0; the geometric r × n term adds
β-dependent yaw as the ring cants.

Conditioning is computed on the four controlled DOF [Fz, Mx, My, Mz] (the
4×N block used in the ITP) WITHOUT per-column normalisation — normalising
removes the common eff·cos β magnitude and was why σ_max/σ_min was previously
constant in β. The condition number now genuinely varies with articulation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from arw_articulated_twin.geometry import RotorStation, rotor_layout

# Default reaction-torque arm κ = k_Q/k_T (m). Snidget Rev D: 15.8 N·m / 157 N ≈ 0.10 m.
DEFAULT_YAW_DRAG_RATIO = 0.10

# Controlled DOF rows used for the conditioning metric: Fz, Mx, My, Mz.
CONTROLLED_DOF_ROWS = (2, 3, 4, 5)


@dataclass
class EffectivenessResult:
    b_matrix: np.ndarray          # (6, n_rotors) full wrench map, unit-thrust columns
    b_controlled: np.ndarray      # (4, n_rotors) [Fz, Mx, My, Mz]
    sigma_max: float
    sigma_min: float
    condition_number: float
    beta_deg: np.ndarray
    yaw_drag_ratio: float


def _spin_sign(spin: str) -> float:
    return 1.0 if spin.upper() == "CW" else -1.0


def rotor_thrust_axes(
    stations: list[RotorStation],
    beta_per_annulus: dict[str, float],
) -> tuple[np.ndarray, np.ndarray]:
    """Return rotor positions (n,3) and unit thrust directions (n,3).

    Thrust tilts by the per-annulus ring cant β about body-Y.
    """
    n = len(stations)
    pos = np.zeros((n, 3), dtype=np.float64)
    direction = np.zeros((n, 3), dtype=np.float64)
    for i, st in enumerate(stations):
        beta = math.radians(beta_per_annulus.get(st.annulus, 0.0))
        pos[i] = (st.x_m, st.y_m, st.z_m)
        direction[i] = (math.sin(beta), 0.0, math.cos(beta))
    return pos, direction


def build_b_matrix(
    vehicle_code: str,
    beta_per_annulus: dict[str, float] | None = None,
    yaw_drag_ratio: float = DEFAULT_YAW_DRAG_RATIO,
    spread_k: float | None = None,  # deprecated, ignored; kept for call-site compatibility
) -> EffectivenessResult:
    """Assemble the 6×N effectiveness matrix for the current articulation state."""
    stations = rotor_layout(vehicle_code)
    if beta_per_annulus is None:
        annuli = sorted({s.annulus for s in stations})
        beta_per_annulus = {a: 0.0 for a in annuli}

    pos, direction = rotor_thrust_axes(stations, beta_per_annulus)
    n = len(stations)
    b = np.zeros((6, n), dtype=np.float64)

    for i, st in enumerate(stations):
        n_i = direction[i]
        r_i = pos[i]
        # Reaction-drag torque acts about the rotor spin axis (≈ thrust axis).
        drag_moment = _spin_sign(st.spin) * yaw_drag_ratio * n_i
        moment = np.cross(r_i, n_i) + drag_moment
        b[0:3, i] = n_i
        b[3:6, i] = moment

    b_ctrl = b[list(CONTROLLED_DOF_ROWS), :]          # (4, N), NO column normalisation
    s = np.linalg.svd(b_ctrl, compute_uv=False)
    sigma_max = float(s[0]) if s.size else 0.0
    sigma_min = float(s[-1]) if s.size else 0.0
    cond = sigma_max / max(sigma_min, 1e-12)

    beta_arr = np.array([beta_per_annulus.get(s.annulus, 0.0) for s in stations])
    return EffectivenessResult(
        b_matrix=b,
        b_controlled=b_ctrl,
        sigma_max=sigma_max,
        sigma_min=sigma_min,
        condition_number=cond,
        beta_deg=beta_arr,
        yaw_drag_ratio=yaw_drag_ratio,
    )


def sweep_articulation(
    vehicle_code: str,
    beta_deg: np.ndarray,
    annulus: str = "A",
    yaw_drag_ratio: float = DEFAULT_YAW_DRAG_RATIO,
) -> list[dict[str, float]]:
    """Log σ_max/σ_min vs articulation angle (validation §8)."""
    rows: list[dict[str, float]] = []
    for beta in beta_deg:
        result = build_b_matrix(vehicle_code, {annulus: float(beta)}, yaw_drag_ratio)
        rows.append(
            {
                "beta_deg": float(beta),
                "sigma_max": result.sigma_max,
                "sigma_min": result.sigma_min,
                "condition_number": result.condition_number,
            }
        )
    return rows
