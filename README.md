# ARW Articulated Twin — Isaac Sim 6.0 + Newton

Articulated ring-wing digital twin for **Aerial MECHANICA / AVIUS** Product 1 open layer.
Per-rotor I.C.2E **Snidget** propulsion (AV5008TM-HV), configuration-dependent effectiveness **B(q_art)**, and a reference pseudo-inverse allocator only.

> **Patent-pending notice:** Articulation control laws, tuned RL/null-space policies, and proprietary WQP allocator are **not** included. Co-sim connects off-board via `bridge/ardupilot_bridge_stub.py`.

## CAD / vehicle variants

| Drawing | Rotors | Blade | Scale | Role |
|---------|--------|-------|-------|------|
| `AER8110-1` RING-WING SET A | 19 | 2 | 0.432 m | ARW19 flight-test geometry |
| `AER8110-2` RING-WING SET B | 38 | 3 | 0.432 m | ARW38 twin annulus |
| `AER8100-1` AERIAL8 QUAD | 76 | 3 | 1.12 m | Full-scale quad ring-wing |

## Stack (target)

- Isaac Sim 6.0 + Newton 1.0 (differentiable physics)
- Isaac Lab 3.0 RL task (`isaaclab_ext/`)
- OpenUSD articulation assets (`assets/`)
- NVIDIA Warp rotor-force kernels (`arw_articulated_twin/warp_kernels.py`)

## Quick start (< 45 min)

```bash
cd /Users/arinzeeze/NVIDIA_2026
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"

# Build USD assets (19 / 38 rotor)
python scripts/build_assets.py

# Standalone hover + articulation (no Isaac required)
python scripts/run_hover.py --vehicle AER8110-1 --seconds 5
python scripts/run_hover.py --vehicle AER8110-1 --articulation-deg 15

# Validation suite (§8)
python scripts/validate.py
```

## Isaac Sim / Isaac Lab (when installed)

```bash
# Set Isaac Lab python env, then:
export ARW_USD_PATH=$(pwd)/assets/arw_aer8110_1.usda
# PhysX fallback if Newton feature missing:
export ARW_PHYSICS_BACKEND=physx  # or newton

# Training entry (extension config):
python -c "from isaaclab_ext.arw_articulated.isaac_env import ARWArticulatedHoverEnvCfg; print(ARWArticulatedHoverEnvCfg())"
```

## Snidget propulsion data

Motor model uses **AV5008TM-HV** (not Maxon EC 69):

- τ_e = 7.0 ms (datasheet mechanical time constant)
- T_i = k_T · ω_i², Q_i = k_Q · ω_i²
- k_T reconstructed from `I-C2E_PDA_PM_Bench_Lab_CSV_Cluster_RevD_2026-05-29` steady-state map
- Inner spool uplift modeled at 9000 rpm / PR 1.018

Point external bench data via symlink:

```bash
ln -sf /Users/arinzeeze/2026_added_RESEARCH/I-C2E_PDA_PM_Bench_Lab_CSV_Cluster_RevD_2026-05-29 data/ic2e_bench
```

## Repository layout

```
arw_articulated_twin/   Core: geometry, Snidget params, B(q_art), env, USD builder
assets/                 Generated USDA articulation assets
bridge/                 ardupilot-bridge co-sim stub (interface only)
isaaclab_ext/           Isaac Lab 3.0 task config
scripts/                build_assets, validate, run_hover
configs/provenance.yaml Parameter provenance table (§7)
```

## IP boundary (§2)

**In:** USD asset, generic rotor model, reference allocator, Isaac Lab task, docs, provenance table.

**Out:** GNFCS core, tuned RL weights, proprietary allocator, validated passenger-scale B(q_art).

## External data paths

- Snidget datasheet: `I-C2E_The_Snidget_Datasheet.pdf`
- Bench CSV cluster: `I-C2E_PDA_PM_Bench_Lab_CSV_Cluster_RevD_2026-05-29`
- Flight layouts: `Reconstructed_Flight_Test_Data_ARW_2024_2025/configs/*_rotor_layout.csv`
