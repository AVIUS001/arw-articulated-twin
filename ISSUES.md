# ISSUES — ARW Articulated Twin (pre-publication punch list)

Status legend: `[x]` done · `[ ]` open · `[~]` partial
Gate: **do not publish to GitHub until all P0 are closed.**

---

## P0 — blockers (correctness / release-gating)

- [x] **B(q_art) was not configuration-dependent.** Condition number was constant
  (1.5068) at every β because `effectiveness.py` column-normalised the controlled
  rows, erasing the common `eff·cosβ` magnitude, and dropped the yaw (Mz) row where
  ring cant actually appears.
  **Fixed** in the new `effectiveness.py`: unit-thrust force model
  (`F=n_i`, `M=r×n_i + spin·κ·n_i`), four controlled DOF `[Fz,Mx,My,Mz]`, SVD with
  **no** column normalisation. Condition number now varies (10.01→9.29 over 0–25°).
  Verify with `python scripts/check_gradients.py` (check 2).

- [x] **Differentiability claim was undelivered.** No gradient code existed despite
  the README selling Newton differentiable physics.
  **Fixed**: `scripts/check_gradients.py` adds a finite-difference-vs-analytic
  gradient check of `J(ω)=½‖B·T(ω)−w‖²` through the Snidget thrust model and
  B(q_art) (max rel err ~3e-10), plus an optional Warp autodiff cross-check that
  runs when `warp-lang` is installed.

- [x] **Update `scripts/validate.py` `check_b_articulation`.** Replaced the old
  non-decreasing `sigma_max` assertion with the variation guard from
  `check_gradients.py` (`rel_change > 0.02` and all `cond < 30`). Gradient checks
  are wired into the main suite.

- [ ] **Counsel gate not yet cleared for this asset.** Extend
  `Open_Release_Scoping_Memo_BLG` to cover the USD asset, the Isaac Lab task, the
  reconstructed Snidget k_T values, and any bundled bench/flight data — green/amber/red
  per item — before the repo goes public.

---

## P1 — accuracy / honesty (fix before first push)

- [x] **"152-rotor" claim is misleading.** Renamed to `76-rotor scaling` in
  `validate.py`; shape `[6,76]` matches the test name.

- [x] **README quick-start hardcoded machine paths.** Replaced with relative
  clone instructions; bench-data symlink marked optional.

- [ ] **Add the PX4/Gazebo consistency cross-check (scope §8).** No test compares this
  twin against `ARW_PX4_SITL_Gazebo_Scope` hover trim / step response at 19/38 with
  articulation locked. Add one so the "step up from PX4" claim is evidenced, not asserted.

- [ ] **`κ` (yaw_drag_ratio) is a single default (0.10 m).** Derive it per-vehicle from
  the actual `k_Q/k_T` of the Snidget coeffs rather than a constant, and label its
  provenance in `provenance.yaml`.

---

## P2 — repo hygiene (before push)

- [x] **`.venv/` not tracked.** `.gitignore` excludes `.venv/`, `__pycache__/`,
  `*.egg-info/`, `validation_report.json`, `.DS_Store`.
- [ ] **Pin versions** for `isaaclab` / `warp-lang` to the Isaac Sim 6.0 / Newton 1.0
  line actually targeted, so a cloner resolves the intended stack.
- [ ] Add a top-level `CITATION.cff` and a short `CONTRIBUTING.md` before going public.

---

## DoD scorecard (scope §11)

| DoD item | State |
|---|---|
| Counsel-approved scope | [ ] open (P0) |
| Clone→build→run hover (+1 maneuver) < 45 min | [x] |
| B(q_art) recomputation verified; σ logged vs β | [x] fixed |
| Consistency with PX4/Gazebo twin at 19/38 | [ ] open (P1) |
| Differentiable-gradient check passes | [x] added |
| 19→152 scaling run | [~] 76 only (P1) |
| Provenance table complete; no IP-sensitive params | [x] |
| License + patent notices; reference allocator only | [x] |

---

## Verified-good (no action)

- SNIDGET (AV5008TM-HV) substitution real; k_T reproduces 16015 g bench point to 0.0%.
- USDA articulation tree real: ArticulationRootAPI hub, revolute hinges, drive
  stiffness/damping, ±25° limits, nacelles parented to canted ring.
- IP boundary holds: pseudo-inverse reference allocator only; WQP behind bridge stub;
  full-text scan finds no GNFCS / WQP / null-space / dithering in shipped source.
- Apache-2.0 license + patent-pending notices present down to USDA metadata.
