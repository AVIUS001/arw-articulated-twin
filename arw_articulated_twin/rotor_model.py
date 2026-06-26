"""Quadratic Snidget rotor force model with first-order motor lag."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from arw_articulated_twin.snidget_params import RotorForceCoeffs, inner_spool_uplift_factor


@dataclass
class RotorState:
    omega_cmd: float = 0.0
    omega: float = 0.0
    inner_omega: float = 0.0


def rpm_to_omega(rpm: float) -> float:
    return rpm * 2.0 * math.pi / 60.0


def omega_to_rpm(omega: float) -> float:
    return omega * 60.0 / (2.0 * math.pi)


def thrust_from_omega(omega: float, coeffs: RotorForceCoeffs, inner_omega: float = 0.0) -> float:
  """T_i = k_T · ω_i² with optional inner-spool uplift (modeled)."""
  base = coeffs.k_t * omega * omega
  uplift = inner_spool_uplift_factor(inner_omega, omega_to_rpm(omega))
  return base * uplift


def reaction_torque(omega: float, coeffs: RotorForceCoeffs, spin_sign: float = 1.0) -> float:
    """Q_i = k_Q · ω_i² · sign(spin)."""
    return spin_sign * coeffs.k_q * omega * omega


def motor_lag_step(omega: float, omega_cmd: float, tau_e: float, dt: float) -> float:
    """First-order lag: dω/dt = (ω_cmd - ω) / τ_e."""
    if tau_e <= 0.0:
        return omega_cmd
    alpha = min(dt / tau_e, 1.0)
    return omega + alpha * (omega_cmd - omega)


def step_rotors(
    states: list[RotorState],
    omega_cmds: np.ndarray,
    inner_omega: float,
    coeffs: RotorForceCoeffs,
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Advance motor states; return thrust (N) and body torque (Nm) per rotor."""
    n = len(states)
    thrust = np.zeros(n, dtype=np.float64)
    torque = np.zeros(n, dtype=np.float64)
    for i in range(n):
        st = states[i]
        st.omega = motor_lag_step(st.omega, float(omega_cmds[i]), coeffs.tau_e_s, dt)
        st.inner_omega = inner_omega
        spin = 1.0 if i % 2 == 0 else -1.0
        thrust[i] = thrust_from_omega(st.omega, coeffs, inner_omega)
        torque[i] = reaction_torque(st.omega, coeffs, spin_sign=spin)
    return thrust, torque
