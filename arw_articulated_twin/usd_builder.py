"""Procedural USD articulation asset builder for ARW ring-wing twin.

Generates OpenUSD assets with PhysicsArticulationRootAPI, revolute ring joints,
and per-rotor nacelle bodies. Rotor spin is visual-only (force applied to nacelle).
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import TextIO

from arw_articulated_twin.geometry import VEHICLES, rotor_layout


def _w(out: TextIO, indent: int, line: str) -> None:
    out.write("  " * indent + line + "\n")


def _write_header(out: TextIO, vehicle_code: str) -> None:
    v = VEHICLES[vehicle_code]
    _w(out, 0, "#usda 1.0")
    _w(out, 0, "(")
    _w(out, 1, f'custom string arw:vehicleCode = "{vehicle_code}"')
    _w(out, 1, f'custom string arw:cadDrawing = "{v.cad_drawing}"')
    _w(out, 1, f'custom int arw:rotorCount = {v.rotor_count}')
    _w(out, 1, 'custom string arw:propulsion = "I-C2E Snidget Rev D (AV5008TM-HV)"')
    _w(out, 1, 'custom string license = "Apache-2.0"')
    _w(out, 1, 'custom string notice = "Patent-pending articulation control — reference allocator only"')
    _w(out, 0, ")")
    _w(out, 0, "")
    _w(out, 0, 'def Xform "ARW" (')


def _write_physics_scene(out: TextIO) -> None:
    _w(out, 1, 'uniform token physics:approximation = "none"')
    _w(out, 1, 'custom vector3f physics:gravity = (0, 0, -9.81)')
    _w(out, 0, ")")
    _w(out, 0, "{")
    _w(out, 1, 'def PhysicsScene "physicsScene"')
    _w(out, 1, "{")
    _w(out, 2, 'uniform vector3f physics:gravityDirection = (0, 0, -1)')
    _w(out, 2, 'float physics:gravityMagnitude = 9.81')
    _w(out, 1, "}")


def _write_hub(out: TextIO, vehicle_code: str) -> None:
    v = VEHICLES[vehicle_code]
    r_hub = v.hub_od_m * 0.5
    _w(out, 1, 'def Xform "hub" (')
    _w(out, 2, 'prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysicsRigidBodyAPI", "PhysicsMassAPI"]')
    _w(out, 1, ")")
    _w(out, 1, "{")
    _w(out, 2, f"float physics:mass = {v.mass_kg * 0.35:.4f}")
    _w(out, 2, 'bool physics:rigidBodyEnabled = true')
    _w(out, 2, 'point3f physics:centerOfMass = (0, 0, 0)')
    _w(out, 2, f'def Cylinder "hub_geom" (radius = {r_hub:.4f}, height = {v.hub_od_m * 0.4:.4f})')
    _w(out, 2, "{")
    _w(out, 3, 'color3f[] primvars:displayColor = [(0.2, 0.2, 0.25)]')
    _w(out, 2, "}")
    _w(out, 1, "}")


def _write_ring_segment(out: TextIO, annulus: str, vehicle_code: str) -> None:
    v = VEHICLES[vehicle_code]
    beta_lim = math.radians(v.beta_limit_deg)
    _w(out, 1, f'def Xform "ring_{annulus}" (')
    _w(out, 2, 'prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]')
    _w(out, 1, ")")
    _w(out, 1, "{")
    _w(out, 2, f"float physics:mass = {v.mass_kg * 0.12 / max(v.annulus_count, 1):.4f}")
    _w(out, 2, 'bool physics:rigidBodyEnabled = true')
    _w(out, 2, f'def PhysicsRevoluteJoint "hinge_{annulus}" (')
    _w(out, 3, 'prepend apiSchemas = ["PhysicsDriveAPI"]')
    _w(out, 2, ")")
    _w(out, 2, "{")
    _w(out, 3, 'rel physics:body0 = </ARW/hub>')
    _w(out, 3, f'rel physics:body1 = </ARW/ring_{annulus}>')
    _w(out, 3, 'point3f physics:localPos0 = (0, 0, 0)')
    _w(out, 3, 'point3f physics:localPos1 = (0, 0, 0)')
    _w(out, 3, 'quatf physics:localRot0 = (1, 0, 0, 0)')
    _w(out, 3, 'quatf physics:localRot1 = (1, 0, 0, 0)')
    _w(out, 3, 'point3f physics:axis = (0, 1, 0)')
    _w(out, 3, f'float physics:lowerLimit = {-beta_lim:.5f}')
    _w(out, 3, f'float physics:upperLimit = {beta_lim:.5f}')
    _w(out, 3, 'float drive:stiffness = 8000')
    _w(out, 3, 'float drive:damping = 400')
    _w(out, 2, "}")
    _w(out, 2, f'def Torus "ring_tube_{annulus}" (majorRadius = {VEHICLES[vehicle_code].scale * 1.1:.4f}, minorRadius = 0.02)')
    _w(out, 2, "{")
    _w(out, 3, 'color3f[] primvars:displayColor = [(0.35, 0.35, 0.4)]')
    _w(out, 2, "}")
    _w(out, 1, "}")


def _write_nacelle(out: TextIO, parent: str, station_id: str, x: float, y: float, z: float, diameter: float) -> None:
    r = diameter * 0.5
    _w(out, 2, f'def Xform "nacelle_{station_id}" (')
    _w(out, 3, 'prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]')
    _w(out, 3, f'double3 xformOp:translate = ({x:.5f}, {y:.5f}, {z:.5f})')
    _w(out, 3, 'uniform token[] xformOpOrder = ["xformOp:translate"]')
    _w(out, 2, ")")
    _w(out, 2, "{")
    _w(out, 3, f"float physics:mass = {0.45 * (diameter / 0.432) ** 2:.4f}")
    _w(out, 3, 'bool physics:rigidBodyEnabled = true')
    _w(out, 3, f'def Cylinder "nacelle_geom" (radius = {r:.4f}, height = {diameter * 0.15:.4f})')
    _w(out, 3, "{")
    _w(out, 4, 'color3f[] primvars:displayColor = [(0.1, 0.55, 0.85)]')
    _w(out, 3, "}")
    _w(out, 3, f'def Xform "rotor_visual_{station_id}"')
    _w(out, 3, "{")
    _w(out, 4, f'def Disk "disk" (radius = {r:.4f})')
    _w(out, 4, "{")
    _w(out, 5, 'color3f[] primvars:displayColor = [(0.85, 0.85, 0.9)]')
    _w(out, 5, 'custom bool arw:visualSpinOnly = true')
    _w(out, 4, "}")
    _w(out, 3, "}")
    _w(out, 2, "}")


def build_usd(vehicle_code: str, output_path: Path) -> Path:
    """Write a USDA articulation asset for the given vehicle variant."""
    v = VEHICLES[vehicle_code]
    layout = rotor_layout(vehicle_code)
    annuli = sorted({s.annulus for s in layout})

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as out:
        _write_header(out, vehicle_code)
        _write_physics_scene(out)
        _write_hub(out, vehicle_code)
        for ann in annuli:
            _write_ring_segment(out, ann, vehicle_code)

        # Nacelles parented under ring segments
        for st in layout:
            parent = f"ring_{st.annulus}"
            _w(out, 1, f'over "ring_{st.annulus}"')
            _w(out, 1, "{")
            _write_nacelle(out, parent, st.rotor_id, st.x_m, st.y_m, st.z_m, v.propulsor_od_m)
            _w(out, 1, "}")

        _w(out, 0, "}")

    return output_path


def build_all_assets(assets_dir: Path) -> list[Path]:
    paths = []
    for code in ("AER8110-1", "AER8110-2"):
        slug = code.lower().replace("-", "_")
        p = build_usd(code, assets_dir / f"arw_{slug}.usda")
        paths.append(p)
    return paths
