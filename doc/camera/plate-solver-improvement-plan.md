# Plate Solver Improvement Plan

Forward-looking plan to improve `CameraPlateSolver` on three axes — **robustness, simplicity,
performance** — derived from the 2026-06-16 investigation. Self-contained so it can be picked up
in a fresh worktree. Historical context and the abandoned seed-unification analysis are in
`plate-solver-notes.md`.

## Current state (the baseline to protect)

- The solver is `plugins/feature/camera/cameraplatesolver.cpp` — a ~24.5k-line single translation
  unit (one inline `SolverContext` class), ~93 `k*` threshold constants, ~145 accept/reject sites,
  4 regime-specialised seed engines + a shared triangle subroutine.
- **Validated suite baseline (do not regress):** REAL **48** / FISHEYE **56** / WIDE **27** /
  RAND **83** / RAND2 **148** (the five corpora in `plugins/feature/camera/test/`). The REAL suite
  is the trustworthy gate; the synthetic oracle is imperfect.
- **Already settled (do not re-litigate):**
  - `blindquad` engine deleted (zero marginal contribution, validated).
  - Seed-engine unification **abandoned** — the engines are regime-specialised, not redundant
    (proven via ablation + probes; see `plate-solver-notes.md`). Not a simplification target.
  - Metamorphic + negative robustness pass is **green** — the solve is robust to noise / rotation /
    translation / dimming, and rejects garbage (noise, blobs) in guided and blind modes. So
    pixel-level robustness is fine; the real fragility is *numerical* (cross-build) and *geometric*
    (near-zenith degeneracy).

## Guiding principles

1. **Measure first.** Probe cheaply before building; this session's biggest wins were avoided
   mistakes (the seed unification) caught by measurement.
2. **Validate every change against the full 434-case corpus** + the ablation + robustness harnesses
   below. The REAL suite is the gate; watch the synthetic deltas for churn.
3. **Prefer ULP-robust discrete criteria at decision points** (counting / margins) over continuous
   LM minima — the cross-build divergence flips outcomes exactly at sharp boundaries.
4. **Behaviour-preserving where possible; flag-gate behaviour changes; keep per-step rollback.**

## Test / validation harness (already in `plugins/feature/camera/test/`)

- Full corpus: run `featurecamera_star_tests.exe` on `star-tests.csv`,
  `star-tests-synthetic-fisheye.csv`, `-wide.csv`, `-rand.csv`, `-rand2.csv`.
- Seed ablation / marginal-contribution parity: `seed-ablation.ps1`
  (env toggles `SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_SEEDS`, `..._DISABLE_QUAD_INDEX`).
- Metamorphic + negative robustness: `gen_robustness_tests.py` + `run-robustness.ps1`.
- Profiling: env `SDRANGEL_CAMERA_PLATE_SOLVER_PROFILE=1` (+ `QT_LOGGING_RULES=*.debug=true`);
  per-stage timings appear in the `stages=` field of the result line.
- Build: `cmake --build build-qt6 --target featurecamera_star_tests featurecamera --config Release -j 8`.

---

## Workstreams (priority order)

### WS0 — Threshold sensitivity sweep + centralise constants  *(cheap, do first; low risk)*
**Goal:** find which of the ~93 `k*` constants are fitted to a single test image (overfit), and make
the tuning surface visible.
**Approach:** (a) pull all `k*` threshold constants into one documented config block; (b) script a
sweep that perturbs each ±10%, re-runs the suite, and flags any constant whose wiggle flips exactly
one test — that's a fitted-to-one-image smell, a candidate for replacement by a principled criterion.
**Validation:** diagnostic only (no behaviour change from the centralisation if done as a pure move).
**Risk:** low. **Payoff:** the map of where the overfitting actually lives — informs WS3.

### WS1 — Kill the cross-build ULP divergence  *(high robustness value)*
The wide-6/7/8/9 saga (GUI DLL vs test EXE producing ULP-different trig that flips discrete match
decisions on marginal solves) was patched with band-aids (seed-anchored grid, Az/El pin, clamped
polish). Fix the cause:
- **WS1a — harness links the shipped binary.** Today the test exe recompiles `cameraplatesolver.cpp`
  independently, so the suite does **not** test the GUI's actual `featurecamera` binary. Make the
  harness link the same object/DLL. Removes the divergence-by-independent-compilation class *and*
  makes the suite trustworthy. **Risk:** build-level, low algorithm risk.
- **WS1b — decision-boundary margins + deterministic tie-breaks.** At every accept/reject gate and
  match-assignment, require a *margin* to cross (hysteresis) and break ties deterministically, so a
  ~1e-13 difference can never flip a verdict. Generalises the match-count-grid lesson.
  **Risk:** moderate (touches decisions; margins need calibrating against the corpus).
**Validation:** GUI and harness produce identical verdicts on wide-7/8/9; full suite unchanged.
**Follow-on:** once solid, the seed-anchored / Az-El-pin band-aids can likely be simplified away.

### WS2 — Near-zenith pose representation  *(robustness + perf; high risk)*
**Goal:** structurally retire the Az↔Roll degeneracy that recurs near zenith (wide-6/7/8/9) instead
of patching it per-case, and retire the `rollAliasCheck` (~42s) + `rollRecovery` (~28s) stages it
forces.
**Approach:** represent/solve pose as a rotation matrix or quaternion (no separate fragile roll
axis), or solve az+roll as a combined parameter near high elevation.
**Validation:** full suite; specifically the near-zenith wide-fisheye cases; measure the perf delta.
**Risk:** high (core pose representation) — but pays off on robustness *and* perf *and* simplicity.

### WS3 — Acceptance-layer consolidation  *(the big prize: simplicity + robustness + anti-overfit)*
**Goal:** replace the ~145 hand-tuned accept/reject sites and their thresholds with one principled
criterion. `poseFalseAlarmLogOdds` (a statistical false-alarm score) already exists — make it the
primary gate (+ a margin from WS1b).
**Approach (staged, never big-bang):**
1. Add faLogOdds (+margin) as the primary accept criterion, with the existing gates kept as a
   secondary safety net; verify the suite is unchanged.
2. Ablate the legacy gates one at a time (like the seed-engine ablation): for each, disable it and
   confirm the corpus + the negative-test suite are unchanged; delete the ones that prove redundant.
3. Keep only the gates that demonstrably catch cases faLogOdds misses; fold their thresholds into
   the WS0 config block.
**Validation:** full corpus + `run-robustness.ps1` negatives (no new false positives) + the WS0
sensitivity sweep (gates should stop being ULP/overfit-fragile).
**Risk:** highest; do after WS0 (know the overfit map) and WS1 (margins in place).

### WS4 — Prebuilt catalogue index  *(performance; large, optional)*
**Goal:** replace the ~52k-evaluation wide-fallback grid, fix blind deep-narrow matching (the
`codeMatches=0` problem — brightest-K detection != brightest-K catalog in a deep field), and speed
blind solves.
**Approach:** astrometry.net-style healpix-tiled quad index, mag 12–15, on disk.
**Risk:** large infrastructure project — only worthwhile if blind *narrow* solving becomes a product
goal. (Guided solving does not need it.)

### WS5 — Split the single TU  *(simplicity / maintainability; low behavioural risk)*
Behaviour-preserving split of the ~24.5k-line `SolverContext` into themed units (catalog I/O,
projection, search, acceptance, Siril network, orchestration). Mechanical churn, no behaviour change;
do when maintainability/compile-time cost bites.

---

## Suggested order & worktree notes

Order: **WS0 → WS1 → WS3 (with WS2 as an enabler) → WS4 / WS5 as capacity allows.**
WS0 and WS1a are cheap and unlock the rest; WS3 is the headline win but depends on them.

Each workstream on its own commit/branch, validated before merge. Before branching the worktree,
commit this plan and the current clean (blindquad-deleted) state so both are present in the new tree.

## Out of scope / settled
- Seed-engine unification (abandoned — regime-specialised, see notes).
- Re-adding deleted engines (`blindquad`) or k2 distortion (reverted, net-negative).
