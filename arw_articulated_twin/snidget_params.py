"""I.C.2E Snidget Rev D propulsion parameters (AV5008TM-HV).

Sources:
  - I-C2E_The_Snidget_Datasheet.pdf (Rev D)
  - I-C2E_PDA_PM_Bench_Lab_CSV_Cluster_RevD_2026-05-29
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

G = 9.80665
SNIDGET_PROPULSOR_OD_M = 0.70716
SNIDGET_HUB_OD_M = 0.26416
SNIDGET_INNER_ID_M = 0.12
SNIDGET_INNER_OD_M = 0.18


@dataclass(frozen=True)
class SnidgetMotorParams:
    motor_pn: str = "AV5008TM-HV"
    bus_v: float = 74.0
    nominal_rpm: float = 3000.0
    max_rpm: float = 3600.0
    nominal_torque_nm: float = 15.8
    r_pp_ohm: float = 0.105
    l_pp_mh: float = 0.14
    tau_e_s: float = 7.0e-3  # mechanical time constant (datasheet HV)
    peak_efficiency: float = 0.92
    inner_rpm_nominal: float = 9000.0
    inner_rpm_max: float = 9500.0
    inner_power_w: float = 3000.0
    pressure_ratio_nominal: float = 1.018
    provenance: str = "Measured"


@dataclass(frozen=True)
class RotorForceCoeffs:
    k_t: float  # N / (rad/s)^2
    k_q: float  # Nm / (rad/s)^2
    tau_e_s: float
    blade_count: int
    diameter_m: float
    provenance: str


def _bench_k_t_at_rpm(thrust_g: float, rpm: float) -> float:
    omega = rpm * 2.0 * math.pi / 60.0
    thrust_n = thrust_g * G / 1000.0
    return thrust_n / max(omega * omega, 1e-9)


def derive_k_t(blade_count: int, diameter_m: float, bench_rpm: float = 3000.0) -> RotorForceCoeffs:
    """Derive quadratic thrust coefficient from Rev D bench map, diameter-scaled.

    Bench reference: 3-blade outer-only @ 3000 rpm → 16015 g on 707 mm prop.
    Solidity scaling exponent 0.7 applied for 2- and 5-blade variants per datasheet.
    """
    bench_ref = {
        2: (11339.0, 2),
        3: (16015.0, 3),
        5: (21291.0, 5),
    }
    thrust_g, ref_blades = bench_ref[blade_count]
    k_t_full = _bench_k_t_at_rpm(thrust_g, bench_rpm)
    scale = (diameter_m / SNIDGET_PROPULSOR_OD_M) ** 2
    k_t = k_t_full * scale
    omega = bench_rpm * 2.0 * math.pi / 60.0
    k_q = 15.8 / max(omega * omega, 1e-9) * scale
    return RotorForceCoeffs(
        k_t=k_t,
        k_q=k_q,
        tau_e_s=SnidgetMotorParams().tau_e_s,
        blade_count=blade_count,
        diameter_m=diameter_m,
        provenance="Reconstructed",
    )


def hover_omega(vehicle_mass_kg: int | float, rotor_count: int, coeffs: RotorForceCoeffs) -> float:
    thrust_each = vehicle_mass_kg * G / rotor_count
    return math.sqrt(max(thrust_each / coeffs.k_t, 0.0))


def inner_spool_uplift_factor(inner_rpm: float, outer_rpm: float) -> float:
    """Simplified dual-spool recovery gain from bench steady-state map (modeled)."""
    if inner_rpm <= 0.0:
        return 1.0
    # 3-blade @ 3000 outer, 9000 inner: recovery_gain_pct ≈ 3.17%
    base = 1.0 + 0.0317 * min(inner_rpm / 9000.0, 1.05)
    return float(np.clip(base, 1.0, 1.08))


def provenance_table() -> list[dict[str, Any]]:
    motor = SnidgetMotorParams()
    coeffs_3 = derive_k_t(3, 0.70716)
    return [
        {"parameter": "primary_motor_pn", "value": motor.motor_pn, "unit": "", "provenance": "Measured", "source": "Snidget datasheet Rev D"},
        {"parameter": "primary_motor_bus_v", "value": motor.bus_v, "unit": "V", "provenance": "Measured", "source": "Bench cluster Rev D"},
        {"parameter": "tau_e", "value": motor.tau_e_s, "unit": "s", "provenance": "Measured", "source": "Datasheet mechanical time constant (HV)"},
        {"parameter": "propulsor_od", "value": SNIDGET_PROPULSOR_OD_M, "unit": "m", "provenance": "Measured", "source": "CAD AER8120 / datasheet"},
        {"parameter": "hub_od", "value": SNIDGET_HUB_OD_M, "unit": "m", "provenance": "Measured", "source": "Datasheet interface"},
        {"parameter": "k_T (3-blade, 707mm)", "value": coeffs_3.k_t, "unit": "N/(rad/s)^2", "provenance": "Reconstructed", "source": "Bench map @ 3000 rpm outer-only"},
        {"parameter": "k_Q (3-blade, 707mm)", "value": coeffs_3.k_q, "unit": "Nm/(rad/s)^2", "provenance": "Modeled", "source": "Nominal torque / ω²"},
        {"parameter": "inner_rpm_nominal", "value": motor.inner_rpm_nominal, "unit": "rpm", "provenance": "Measured", "source": "Bench rating summary"},
        {"parameter": "pressure_ratio_nominal", "value": motor.pressure_ratio_nominal, "unit": "ratio", "provenance": "Measured", "source": "Rev D PR envelope"},
        {"parameter": "articulation_beta_limit", "value": 25.0, "unit": "deg", "provenance": "Modeled", "source": "Reference hinge travel (public twin)"},
        {"parameter": "B(q_art) spread_k", "value": 0.62, "unit": "", "provenance": "Modeled", "source": "Generic articulation authority model"},
    ]
