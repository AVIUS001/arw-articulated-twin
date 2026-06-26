"""GPU kernels for per-rotor forces and B(q_art) — Warp with NumPy fallback."""
from __future__ import annotations

import numpy as np

from arw_articulated_twin.effectiveness import build_b_matrix
from arw_articulated_twin.rotor_model import thrust_from_omega
from arw_articulated_twin.snidget_params import RotorForceCoeffs

_USE_WARP = False
try:
    import warp as wp

    wp.init()
    _USE_WARP = True
except ImportError:
    wp = None  # type: ignore


if _USE_WARP:

    @wp.kernel
    def rotor_thrust_kernel(
        omega: wp.array(dtype=float),
        k_t: float,
        inner_omega: float,
        thrust_out: wp.array(dtype=float),
    ):
        i = wp.tid()
        w = omega[i]
        thrust_out[i] = k_t * w * w

    @wp.kernel
    def rotor_torque_kernel(
        omega: wp.array(dtype=float),
        k_q: float,
        spin: wp.array(dtype=float),
        torque_out: wp.array(dtype=float),
    ):
        i = wp.tid()
        w = omega[i]
        torque_out[i] = spin[i] * k_q * w * w

    def compute_thrusts_gpu(omega: np.ndarray, coeffs: RotorForceCoeffs, inner_omega: float = 0.0) -> np.ndarray:
        n = len(omega)
        omega_wp = wp.array(omega.astype(np.float32), dtype=float)
        out = wp.zeros(n, dtype=float)
        wp.launch(rotor_thrust_kernel, dim=n, inputs=[omega_wp, coeffs.k_t, inner_omega, out])
        return out.numpy()

else:

    def compute_thrusts_gpu(omega: np.ndarray, coeffs: RotorForceCoeffs, inner_omega: float = 0.0) -> np.ndarray:
        return np.array([thrust_from_omega(float(w), coeffs, inner_omega) for w in omega])


def recompute_b_matrix(vehicle_code: str, beta_per_annulus: dict[str, float]) -> np.ndarray:
    """Host-side B(q_art); suitable for logging and allocator refresh."""
    return build_b_matrix(vehicle_code, beta_per_annulus).b_matrix


def backend_name() -> str:
    return "warp" if _USE_WARP else "numpy"
