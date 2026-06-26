"""Isaac Lab extension entry — wraps ArticulatedHoverEnv when Isaac Lab is present."""
from __future__ import annotations

try:
    from isaaclab.envs import ManagerBasedRLEnvCfg
    from isaaclab.utils import configclass

    ISAAC_AVAILABLE = True
except ImportError:
    ISAAC_AVAILABLE = False
    ManagerBasedRLEnvCfg = object  # type: ignore
    configclass = lambda x: x  # type: ignore


if ISAAC_AVAILABLE:

    @configclass
    class ARWArticulatedHoverEnvCfg(ManagerBasedRLEnvCfg):
        """Isaac Lab 3.0 task config for articulated ring-wing hover."""

        vehicle_code: str = "AER8110-1"
        usd_path: str = "assets/arw_aer8110_1.usda"
        num_envs: int = 64
        episode_length_s: float = 10.0
        physics_backend: str = "newton"  # or "physx" fallback
        enable_domain_randomization: bool = True
        articulation_maneuver_deg: float = 15.0

else:

    class ARWArticulatedHoverEnvCfg:
        vehicle_code: str = "AER8110-1"
        usd_path: str = "assets/arw_aer8110_1.usda"
        num_envs: int = 64
