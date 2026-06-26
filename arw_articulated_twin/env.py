"""Isaac Lab 3.0 articulated hover / maneuver environment (public reference twin)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from arw_articulated_twin.allocator import allocate_hover, pseudo_inverse_allocate
from arw_articulated_twin.effectiveness import build_b_matrix, sweep_articulation
from arw_articulated_twin.geometry import VEHICLES, rotor_layout
from arw_articulated_twin.rotor_model import (
    RotorState,
    motor_lag_step,
    omega_to_rpm,
    rpm_to_omega,
    thrust_from_omega,
)
from arw_articulated_twin.snidget_params import derive_k_t


@dataclass
class EnvConfig:
    vehicle_code: str = "AER8110-1"
    dt: float = 0.01
    episode_length_s: float = 10.0
    inner_rpm: float = 9000.0
    domain_randomization: bool = True
    dr_mass_frac: float = 0.05
    dr_k_t_frac: float = 0.08
    dr_tau_frac: float = 0.15
    seed: int = 42


@dataclass
class ArticulatedHoverEnv:
    """Standalone env (runs without Isaac Sim for validation / CI).

    When Isaac Lab is installed, wrap via ``isaaclab_ext.arw_articulated.isaac_env``.
    """

    config: EnvConfig = field(default_factory=EnvConfig)
    rng: np.random.Generator = field(init=False)

    def __post_init__(self) -> None:
        self.rng = np.random.default_rng(self.config.seed)
        self.reset()

    def reset(self) -> np.ndarray:
        v = VEHICLES[self.config.vehicle_code]
        self.vehicle = v
        self.layout = rotor_layout(v.code)
        self.n_rotors = len(self.layout)
        self.coeffs = derive_k_t(v.blade_count, v.rotor_diameter_m)
        self.mass = v.mass_kg
        if self.config.domain_randomization:
            self.mass *= 1.0 + self.rng.uniform(-self.config.dr_mass_frac, self.config.dr_mass_frac)
            self.coeffs = derive_k_t(v.blade_count, v.rotor_diameter_m)
            scale = 1.0 + self.rng.uniform(-self.config.dr_k_t_frac, self.config.dr_k_t_frac)
            self.coeffs = type(self.coeffs)(
                k_t=self.coeffs.k_t * scale,
                k_q=self.coeffs.k_q * scale,
                tau_e_s=self.coeffs.tau_e_s * (1.0 + self.rng.uniform(-self.config.dr_tau_frac, self.config.dr_tau_frac)),
                blade_count=self.coeffs.blade_count,
                diameter_m=self.coeffs.diameter_m,
                provenance=self.coeffs.provenance,
            )

        annuli = sorted({s.annulus for s in self.layout})
        self.beta = {a: 0.0 for a in annuli}
        self.states = [RotorState() for _ in range(self.n_rotors)]
        self.pos = np.array([0.0, 0.0, 1.5])
        self.vel = np.zeros(3)
        self.att = np.zeros(3)  # roll, pitch, yaw
        self.ang_vel = np.zeros(3)
        self.step_count = 0
        self.max_steps = int(self.config.episode_length_s / self.config.dt)
        return self._observe()

    def _observe(self) -> np.ndarray:
        return np.concatenate([
            self.pos,
            self.vel,
            self.att,
            self.ang_vel,
            np.array(list(self.beta.values()), dtype=np.float64),
            np.array([st.omega for st in self.states], dtype=np.float64),
        ])

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, dict[str, Any]]:
        """Action: [ω_cmd per rotor (rad/s), β_targets per annulus (deg)]."""
        v = self.vehicle
        n_ann = len(self.beta)
        omega_cmds = action[: self.n_rotors]
        beta_targets = action[self.n_rotors : self.n_rotors + n_ann]

        annuli = list(self.beta.keys())
        for i, a in enumerate(annuli):
            lim = v.beta_limit_deg
            self.beta[a] = float(np.clip(beta_targets[i], -lim, lim))

        thrusts = np.zeros(self.n_rotors)
        for i, st in enumerate(self.states):
            st.omega = motor_lag_step(st.omega, float(omega_cmds[i]), self.coeffs.tau_e_s, self.config.dt)
            thrusts[i] = thrust_from_omega(st.omega, self.coeffs, rpm_to_omega(self.config.inner_rpm))

        eff = build_b_matrix(v.code, self.beta)
        wrench = eff.b_matrix @ thrusts
        fz, fx, fy = wrench[2], wrench[0], wrench[1]

        # Point-mass + Euler (simplified)
        g = 9.80665
        az = (fz - self.mass * g) / self.mass
        self.vel[2] += az * self.config.dt
        self.pos += self.vel * self.config.dt
        self.pos[2] = max(self.pos[2], 0.0)

        target_z = 1.5
        omega_hover = math.sqrt(max(self.mass * 9.80665 / (self.n_rotors * self.coeffs.k_t), 1.0))
        omega_err = float(np.mean((omega_cmds - omega_hover) ** 2))
        reward = -abs(self.pos[2] - target_z) - 1e-4 * omega_err
        if self.pos[2] > 0.2 and abs(self.pos[2] - target_z) < 0.15:
            reward += 1.0

        self.step_count += 1
        done = self.step_count >= self.max_steps or self.pos[2] < 0.05
        info = {
            "wrench": wrench.tolist(),
            "sigma_max": eff.sigma_max,
            "sigma_min": eff.sigma_min,
            "condition_number": eff.condition_number,
            "beta_deg": dict(self.beta),
        }
        return self._observe(), float(reward), done, info

    def reference_hover_action(self) -> np.ndarray:
        """Trim action from pseudo-inverse allocator at β=0."""
        alloc = allocate_hover(self.vehicle.code, self.mass, self.beta)
        omega_cmds = np.zeros(self.n_rotors)
        for i, t in enumerate(alloc.thrust_cmds):
            omega_cmds[i] = math.sqrt(max(t / self.coeffs.k_t, 0.0))
        beta = np.array(list(self.beta.values()))
        return np.concatenate([omega_cmds, beta])
