"""Vehicle geometry anchored to Fusion CAD assemblies.

CAD references (ES-111 / AER8 job code):
  - AER8110-1  RING-WING ASSY — 19× PROPULSION UNITS SET A  (2-blade Snidget, ARW19)
  - AER8110-2  RING-WING ASSY — 19× PROPULSION UNITS SET B  (3-blade Snidget, ARW38)
  - AER8100-1  AIRCRAFT ASSY — AERIAL8 QUAD RING-WING EVTOL  (76 rotors, 4 annuli)
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

G = 9.80665

# Canonical ARW19 single-annulus layout (metres), from AER8110-1 / flight layout CSV.
ARW19_XY = np.array([
    [0.0, 0.0], [0.806, 0.0], [0.403, 0.698], [-0.403, 0.698], [-0.806, 0.0],
    [-0.403, -0.698], [0.403, -0.698], [1.4972, 0.4012], [1.096, 1.096],
    [0.4012, 1.4972], [-0.4012, 1.4972], [-1.096, 1.096], [-1.4972, 0.4012],
    [-1.4972, -0.4012], [-1.096, -1.096], [-0.4012, -1.4972], [0.4012, -1.4972],
    [1.096, -1.096], [1.4972, -0.4012],
])
R_OUT_M = float(np.max(np.hypot(ARW19_XY[:, 0], ARW19_XY[:, 1])))

# Twin-annulus lateral offset (AER8110-2 / ARW38), metres.
TWIN_ANNULUS_OFFSET_M = 1.45


@dataclass(frozen=True)
class VehicleConfig:
    code: str
    cad_drawing: str
    rotor_count: int
    annulus_count: int
    mass_kg: float
    rotor_diameter_m: float
    hub_od_m: float
    propulsor_od_m: float
    blade_count: int
    hover_rpm: float
    nominal_bus_v: float
    articulation_joint_count: int
    beta_limit_deg: float
    scale: float  # geometry scale vs canonical ARW19 XY


VEHICLES: dict[str, VehicleConfig] = {
    "AER8110-1": VehicleConfig(
        code="AER8110-1",
        cad_drawing="AER8110-1 RING-WING ASSY - 19X PROPULSION UNITS SET A",
        rotor_count=19,
        annulus_count=1,
        mass_kg=19.2,
        rotor_diameter_m=0.432,
        hub_od_m=0.26416 * (0.432 / 0.70716),
        propulsor_od_m=0.432,
        blade_count=2,
        hover_rpm=4550.0,
        nominal_bus_v=74.0,
        articulation_joint_count=1,
        beta_limit_deg=25.0,
        scale=1.0,
    ),
    "AER8110-2": VehicleConfig(
        code="AER8110-2",
        cad_drawing="AER8110-2 RING-WING ASSY - 19X PROPULSION UNITS SET B",
        rotor_count=38,
        annulus_count=2,
        mass_kg=78.0,
        rotor_diameter_m=0.432,
        hub_od_m=0.26416 * (0.432 / 0.70716),
        propulsor_od_m=0.432,
        blade_count=3,
        hover_rpm=5200.0,
        nominal_bus_v=74.0,
        articulation_joint_count=2,
        beta_limit_deg=25.0,
        scale=1.0,
    ),
    "AER8100-1": VehicleConfig(
        code="AER8100-1",
        cad_drawing="AER8100-1 AIRCRAFT ASSY - AERIAL8 QUAD RING-WING EVTOL",
        rotor_count=76,
        annulus_count=4,
        mass_kg=1210.0,
        rotor_diameter_m=1.12,
        hub_od_m=0.26416 * (1.12 / 0.70716),
        propulsor_od_m=1.12,
        blade_count=3,
        hover_rpm=2200.0,
        nominal_bus_v=74.0,
        articulation_joint_count=4,
        beta_limit_deg=20.0,
        scale=1.12 / 0.432,
    ),
}


@dataclass(frozen=True)
class RotorStation:
    rotor_id: str
    annulus: str
    ring: str
    x_m: float
    y_m: float
    z_m: float
    spin: str
    lever_arm_m: float


def _ell_arm(x: float, y: float) -> float:
    return float(np.hypot(x, y) + R_OUT_M)


def single_annulus_layout(scale: float = 1.0, annulus: str = "A", offset_xy: tuple[float, float] = (0.0, 0.0)) -> list[RotorStation]:
    ox, oy = offset_xy
    stations: list[RotorStation] = []
    rings = ["center"] + ["inner"] * 6 + ["outer"] * 12
    spins = ["CW", "CCW"] * 10
    for i, (xy, ring) in enumerate(zip(ARW19_XY, rings)):
        x, y = float(xy[0] * scale + ox), float(xy[1] * scale + oy)
        rid = f"{annulus}{i + 1:02d}"
        stations.append(
            RotorStation(
                rotor_id=rid,
                annulus=annulus,
                ring=ring,
                x_m=x,
                y_m=y,
                z_m=0.0,
                spin=spins[i % len(spins)],
                lever_arm_m=_ell_arm(xy[0], xy[1]) * scale,
            )
        )
    return stations


def rotor_layout(vehicle_code: str) -> list[RotorStation]:
    v = VEHICLES[vehicle_code]
    if v.annulus_count == 1:
        return single_annulus_layout(scale=v.scale, annulus="A")
    if v.annulus_count == 2:
        s = v.scale
        off = TWIN_ANNULUS_OFFSET_M * s
        left = single_annulus_layout(scale=s, annulus="L", offset_xy=(-off, 0.0))
        right = single_annulus_layout(scale=s, annulus="R", offset_xy=(off, 0.0))
        return left + right
    if v.annulus_count == 4:
        s = v.scale
        off = TWIN_ANNULUS_OFFSET_M * s
        quad = [(-off, -off), (-off, off), (off, -off), (off, off)]
        labels = ["FL", "BL", "FR", "BR"]
        out: list[RotorStation] = []
        for label, (ox, oy) in zip(labels, quad):
            out.extend(single_annulus_layout(scale=s, annulus=label, offset_xy=(ox, oy)))
        return out
    raise ValueError(f"Unsupported annulus_count for {vehicle_code}")


def articulation_lever_arms(vehicle_code: str) -> np.ndarray:
    """Per-rotor hinge lever arm (m) for ring cant kinematics."""
    return np.array([r.lever_arm_m for r in rotor_layout(vehicle_code)], dtype=np.float64)


def vehicle_summary(vehicle_code: str) -> dict[str, Any]:
    v = VEHICLES[vehicle_code]
    layout = rotor_layout(vehicle_code)
    return {
        "vehicle": v.__dict__,
        "rotor_count": len(layout),
        "rotors": [r.__dict__ for r in layout],
    }


def load_layout_csv(path: Path) -> list[RotorStation]:
    """Optional: load measured layout CSV (ARW flight-test format)."""
    import csv

    rows: list[RotorStation] = []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            x, y = float(row["x_m"]), float(row["y_m"])
            rows.append(
                RotorStation(
                    rotor_id=row["rotor_id"],
                    annulus=row["annulus"],
                    ring=row["ring"],
                    x_m=x,
                    y_m=y,
                    z_m=float(row.get("z_m", 0.0)),
                    spin=row["spin"],
                    lever_arm_m=_ell_arm(x, y),
                )
            )
    return rows
