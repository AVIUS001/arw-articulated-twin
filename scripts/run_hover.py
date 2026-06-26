#!/usr/bin/env python3
"""One-command hover bring-up (standalone, no Isaac Sim required)."""
from __future__ import annotations

import argparse

from arw_articulated_twin.env import ArticulatedHoverEnv, EnvConfig


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", default="AER8110-1")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--articulation-deg", type=float, default=0.0)
    args = parser.parse_args()

    cfg = EnvConfig(vehicle_code=args.vehicle, episode_length_s=args.seconds, domain_randomization=False)
    env = ArticulatedHoverEnv(cfg)
    action = env.reference_hover_action()
    if args.articulation_deg != 0.0:
        action[len(env.states)] = args.articulation_deg

    print(f"Vehicle: {args.vehicle} | rotors: {env.n_rotors} | mass: {env.mass:.2f} kg")
    print(f"Snidget k_T: {env.coeffs.k_t:.6e} N/(rad/s)² | τ_e: {env.coeffs.tau_e_s*1e3:.1f} ms")

    step = 0
    obs, reward, done, info = env.step(action)
    while not done:
        obs, reward, done, info = env.step(action)
        step += 1
        if step % 50 == 0:
            print(
                f"  t={step*cfg.dt:.2f}s z={env.pos[2]:.3f}m "
                f"σ_cond={info['condition_number']:.2f} β={info['beta_deg']}"
            )

    print(f"Done. Final z={env.pos[2]:.3f} m | reward={reward:.3f}")


if __name__ == "__main__":
    main()
