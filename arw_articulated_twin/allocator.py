"""Reference allocation — pseudo-inverse only (public twin).

Proprietary WQP allocator runs off-board via ardupilot-bridge stub.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from arw_articulated_twin.effectiveness import EffectivenessResult, build_b_matrix


@dataclass
class AllocationResult:
    thrust_cmds: np.ndarray
    residual: np.ndarray
    success: bool


def pseudo_inverse_allocate(
    wrench_desired: np.ndarray,
    effectiveness: EffectivenessResult,
    thrust_min: float = 0.0,
    thrust_max: float = 500.0,
) -> AllocationResult:
    """Minimum-norm thrust allocation: u = B⁺ · w."""
    b = effectiveness.b_matrix
    b_pinv = np.linalg.pinv(b)
    u = b_pinv @ wrench_desired
    u = np.clip(u, thrust_min, thrust_max)
    residual = wrench_desired - b @ u
    success = float(np.linalg.norm(residual)) < 0.05 * max(np.linalg.norm(wrench_desired), 1e-6)
    return AllocationResult(thrust_cmds=u, residual=residual, success=success)


def hover_wrench(mass_kg: float, g: float = 9.80665) -> np.ndarray:
    w = np.zeros(6, dtype=np.float64)
    w[2] = mass_kg * g
    return w


def allocate_hover(
    vehicle_code: str,
    mass_kg: float,
    beta_per_annulus: dict[str, float] | None = None,
) -> AllocationResult:
    eff = build_b_matrix(vehicle_code, beta_per_annulus)
    return pseudo_inverse_allocate(hover_wrench(mass_kg), eff)
