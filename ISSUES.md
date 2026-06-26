# ISSUES — ARW Articulated Twin

Status legend: `[x]` done · `[ ]` open · `[~]` partial

Public repo: https://github.com/AVIUS001/arw-articulated-twin

---

## P0 — release blockers (correctness)

- [x] **B(q_art) not configuration-dependent** — fixed; cond 10.0→9.3 over 0–25°; variation guard in suite.
- [x] **Differentiability undelivered** — `scripts/check_gradients.py`; FD vs analytic ~7×10⁻¹⁰.
- [x] **`validate.py` articulation test** — asserts variation, not a trivial bound.
- [ ] **Counsel gate** — extend Open Release memo to cover USD asset, Isaac Lab task, reconstructed Snidget k_T. Internal gate; repo is public pending sign-off.

---

## P1 — accuracy / roadmap

- [x] **"152-rotor" naming** — renamed to 76-rotor scaling check; 152 on product ladder as roadmap.
- [x] **README machine paths** — relative clone instructions.
- [ ] **PX4/Gazebo consistency cross-check** at 19/38, articulation locked.
- [ ] **Per-vehicle κ = k_Q/k_T** in `effectiveness.py` + `provenance.yaml`.
- [ ] **57 / 114 / 152 rotor USD assets** per product ladder.

---

## P2 — polish

- [x] `.venv/` gitignored.
- [ ] Pin `isaaclab` / `warp-lang` to Isaac Sim 6.0 / Newton 1.0 line.
- [x] `CONTRIBUTING.md` added.
- [ ] `CITATION.cff`.

---

## DoD scorecard

| Item | State |
|------|-------|
| Counsel-approved scope | [ ] internal |
| Clone→build→run < 45 min | [x] |
| B(q_art) verified; σ vs β | [x] |
| PX4/Gazebo consistency | [ ] |
| Differentiable gradient check | [x] |
| 19→152 scaling | [~] 76 in repo; 152 roadmap |
| Provenance table | [x] |
| License + reference allocator only | [x] |

---

## Verified-good

- Snidget AV5008TM-HV; k_T reproduces 16015 g bench @ 0.0% error.
- USDA articulation tree: hub root, revolute hinges, ±25° limits.
- IP scan clean: no GNFCS / WQP / null-space in shipped source.
- Apache-2.0 + patent-pending notices in USDA metadata.
