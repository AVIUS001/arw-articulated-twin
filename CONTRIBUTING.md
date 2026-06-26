# Contributing

Thank you for interest in the ARW Articulated Twin. This is the **open wedge** of the AVIUS stack — simulation, RL task, reference allocator, and provenance-labeled parameters only.

## What we welcome

- Bug reports with reproduction steps (`scripts/validate.py` output helps)
- Fixes to B(q_art), Snidget parameter derivation, or USD articulation assets
- Isaac Lab / Newton integration improvements (with version pins noted in PR)
- Documentation and validation that strengthen reproducibility

## What we cannot accept in this repo

Per the IP boundary in [README.md](README.md):

- Proprietary WQP / constrained allocators
- GNFCS control laws, tuned RL weights, or null-space policies
- Passenger-scale validated coefficients not cleared for public release

Those belong off-board, behind `bridge/ardupilot_bridge_stub.py`.

## Before you PR

```bash
pip install -e ".[dev]"
python scripts/validate.py
python scripts/check_gradients.py
```

All checks must pass. If you change `effectiveness.py`, confirm **condition number varies with articulation** — not just that it stays below a ceiling.

## Counsel / export control

Do not submit measured flight or bench data unless it is already cleared for public release. Default new parameters to **Modeled** in `configs/provenance.yaml`.
