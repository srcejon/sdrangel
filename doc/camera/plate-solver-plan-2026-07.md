# Plate Solver — Updated Plan (2026-07-04)

Supersedes the forward-looking parts of `plate-solver-improvement-plan.md` (WS0–WS5) and
`plate-solver-review-implementation-plan.md` (Phases 0–8) for new work. Both remain the record of
what was done and what was settled. This plan reflects what the 2026-07 review implementation
actually *proved*, compares the design against other plate solvers, and reorders priorities
accordingly.

## What the recent work established (evidence, not opinion)

Landed on `plate-solver-review` (all validated, REAL never below the 47/48 branch baseline):
Siril range-cache 32 GB bound (was 148 GB, filled the disk); 7 of 8 review defects; flag-gated
detector S1; P-A final-pass seed cache (−15–25% on dense narrow cases); P-C comparator memoization.

Three experiments produced **decisive negatives**, and they all point at the same structural fact:

1. **D4 (fix the intransitive comparator bands):** a *correct* strict-weak-ordering reshuffled
   near-ties → WIDE 27→21, mode4 +4. The near-tie ordering the solver currently ships is
   arbitrary (technically UB), yet cases depend on it.
2. **S1 (fix the wrong blob area):** physically correct → synthetic fisheye +8, REAL −4.
3. **S1+S3 (also fix the wrong noise σ):** the 4σ threshold multiplier turns out to be calibrated
   against the *wrong* σ; correcting σ alone over-thresholds → fisheye gain gone, WIDE −6.

**Conclusion: the solver sits in a compensating-error equilibrium.** Its ~93 constants and ~145
accept sites are co-tuned around existing biases, so *any* single-variable correction — even a
plainly right one — regresses some corpus. Improvements can only land as jointly-recalibrated
packages, and packages can only be tuned against a validation corpus that is trustworthy and
cheap to run. Today it is neither:

- The synthetic fisheye oracle is weak (documented; it has now misled tuning twice).
- REAL is trustworthy but small (48) and **not hermetic** — pollux flipped PASS→FAIL purely from
  live Siril catalog-content drift, with byte-identical code.
- The dense-regime gate (RAND2, 150 cases, 100% Siril) is too slow/network-bound to run per-change.

**Therefore the binding constraint is measurement, not solver code.** That is the reordering below.

## What other plate solvers do (and what to take from them)

| Solver | Design | Lesson for us |
|---|---|---|
| **astrometry.net** | Blind solve via geometric hashing of 4-star "quad codes" against pre-built HEALPix-tiled index files; acceptance is a single **Bayesian odds ratio** (Sutherland & Saunders 1992 foreground/background likelihood per star, with a distractor fraction), tuned so false positives are ~impossible. SIP polynomial distortion fitted after the rigid solve. BSD-licensed. | We already have the skeleton: the vector quad-code engine *is* their hashing, `poseFalseAlarmLogOdds` *is* their verifier (shadow mode), WS4 *is* their index. The difference: they use the statistical verifier as **the** decision, we use ~145 hand-tuned gates with the verifier only logged. Their per-star mixture formulation (not our per-match sum) is why a single threshold works for them — WS3 measured that our current formulation doesn't separate (accepts 1.5–1204 overlap rejects 45–143). The fix is the formulation, not the idea. |
| **Tetra3 / cedar-solve** (ESA star-tracker heritage) | Tiny pattern database of 4-star patterns hashed by scale-invariant edge ratios; built for **wide-field/all-sky** cameras; solves in ms; explicitly tolerant of moderate distortion (can fit distortion per solve). Apache-licensed. | Closest published design to our niche (all-sky fisheye pointing). Confirms two of our own findings: brightest-K patterns work when both pools are shallow (our deep-narrow `codeMatches=0` problem is a *pool mismatch*, not a geometry limit), and distortion must be handled *in the feature*, not by loosening epsilon. Their approach to fisheye = solve on the central region / include distortion in the pattern fit — see A3 below. |
| **ASTAP** | Quad/tetrahedron hashing against a local star DB (HNSKY/G17); popular, fast, ships as one exe with a simple CLI. | The pragmatic benchmark. If blind narrow deep-field ever becomes a product goal, invoking ASTAP (or astrometry.net index files) as an optional external engine is far cheaper than WS4 and battle-tested. Easily A/B-able against our solver via the harness. |
| **Siril / PlateSolve2·3 / watney** | Near-position (seeded) solving against local catalogs; watney is an astrometry.net-index-compatible reimplementation. | Guided solving against a good local catalog is standard; our guided path is competitive and more featureful (alt-az native, roll, lens k1, observer time/place). Nothing to adopt urgently. |

**Where we are genuinely differentiated (keep in-house):** alt-az native solves with observer
time/location, guided modes with roll/FoV priors, ultra-wide fisheye (160°+) with lens
calibration recovery, and tight SDRangel integration. astrometry.net/ASTAP do *not* serve the
180° equidistant all-sky case well — that niche is ours and worth the bespoke engine.

**Where we are re-inventing at a disadvantage (consider delegating or copying exactly):** blind
narrow deep-field (WS4 territory, `codeMatches=0`), and the acceptance layer (their single
statistical verifier vs our 145 gates).

## Updated plan

### Progress (2026-07-05): real corpus built + first findings

Track 0b is under way and already paying off. Staged real-data corpus in
`plugins/feature/camera/test/images-real-fisheye-staging/` (gitignored, like the synthetic sets):
- **GMN/RMS Perth** wide-field (6 cameras, platepar ground truth incl. distortion) →
  `star-tests-real-gmn.csv`; **UCalgary TREx RGB** 180° fisheye (skymap az/el ground truth) →
  `star-tests-real-trex.csv`. Anchors derived rigorously (platepar star_list poly-fit / skymap
  inversion + peak-snap, all visually verified); method + caveats in the corpus README.

Findings that already change priorities:
- **GMN 4/6 PASS.** Every frame `solved=true`; only the external anchors caught the wrong poses —
  demonstrating on real data that a self-consistent oracle over-passes. Discovered the **GMN roll
  convention: solver roll = −platepar `rotation_from_horiz`** (az/el map directly).
- **The star DETECTOR, not the plate solver, is the first fisheye bottleneck.** Raw TREx frames give
  `detections=1..3, solved=false` (0/16) — the S1 ~1-2 px area-gate bug — vs 200–760 with a plain 4σ
  detector. With a contrast stretch the detector recovers 40–76 and the solver attempts real solves.
- **Detector-V2 (S1) is VALIDATED on real fisheye.** Stretched LUCK subset: V2 off → 1/4 PASS; V2 on
  → 3/4 (60–76 det). This is the trustworthy evidence the synthetic oracle could not give: the S1
  fix that regressed synthetic-REAL genuinely helps the real fisheye regime. → **A2 is now a green
  light** (as a jointly-tuned package that protects narrow REAL), not a deferred maybe.
- **A residual real fisheye-accuracy gap remains** (full TREx V2-on = 4/16; solver solves but to a
  wrong pose ~75%). Measurable against ground truth for the first time. Confounds to remove first:
  the approximate near-zenith TREx seed and the equidistant-fisheye lens-model match.

**A2 landed (context-gated detector-V2, commit d2305e6d9, still flag-gated).** Iterated the package:
the fisheye benefit and the narrow-REAL regression are the SAME mechanism (the pixel-count area
differs ~20% from the polygon area for every blob, and the dense-narrow solver heuristics are co-tuned
around the legacy polygon area). Admission-vs-metadata separation was tried and reverted — it protects
REAL but loses the fisheye gain (the recovered stars need usable metadata to be matched, and the
existing bright stars' metadata matters too). Resolution: since the small-star problem is specifically
a wide/fisheye phenomenon, apply V2 ONLY in wide (fov>=30) / fisheye-projection contexts, narrow
rectilinear untouched. **Result: REAL 47->46 (only wide-9), FISHEYE mode1 +3 / mode4 +5, WIDE 0, real
TREx 0/16 -> solving.** V2-off byte-identical. Same fisheye gain as full-V2 at 1 REAL regression instead
of 4.

**A2 SHIPPED default-on (commit d259d9cdb).** The wide-9 block was diagnosed as a weak-oracle mode-2
azimuth alias, not metadata drift: the seed-anchored candidate finds the true azimuth but is pinned by
`lockSeedDirection` and loses to a higher-match alias. Fixed by extending the rot-vec LM to
elevation-seeded wide fisheye (commit 26839fe13), which unpins and refines in the camera frame — wide-9
then solves cleanly (az 39.9, rms 6.30, 187 matched). With that in place V2 was flipped to default-on
with a `SDRANGEL_CAMERA_STAR_DETECTOR_DISABLE_V2` kill-switch. **Final validated numbers:** default
(V2-on) REAL **47/48** (verdict set identical to the pre-V2 baseline — wide-9 passes either way),
synthetic FISHEYE mode1 35->38, mode4 40->45, WIDE 27 (net 0, one swap wide30-003↔wide30b-001), real
all-sky corpus solving; kill-switch (DISABLE_V2=1) reproduces the legacy baseline byte-identical across
REAL/FISHEYE/WIDE. No REAL regression — the "Harden wide-9, then default-on" plan is complete.

### Track 0 — Measurement first (the unlock; do before any further solver tuning)

- **0a Hermetic catalog for the harness. SHIPPED** (commit pending, worktree `-review`). Added the
  offline gate `SDRANGEL_CAMERA_PLATE_SOLVER_OFFLINE`: both Siril SPCC network egress points
  (`fetchSirilRangeFromSource`, `prefetchSirilMergedRanges`) refuse+`qWarning` on a cache miss instead
  of fetching, so any incompleteness fails loudly. Added `SDRANGEL_CAMERA_PLATE_SOLVER_CACHE_DIR` to
  point the whole catalog root at a versioned snapshot. Tooling: `test/hermetic.ps1` (run a split
  offline, report offline-miss count + baseline diff) and `test/refresh-catalog-snapshot.ps1` (build a
  portable REAL-subset snapshot — the full AppData cache is ~11 GB of accumulated ranges, so the
  snapshot is gitignored/regenerable like the corpus images). **Verified:** 32/48 REAL cases use the
  Siril SPCC (Gaia DR3) path; under OFFLINE all 48 solve from the warm cache with ZERO network requests
  and the verdict set IDENTICAL to the online baseline — REAL is now a hermetic gate and pollux's 1
  failure is confirmed deterministic (frozen cache), not a network artifact. *Cheap; highest value per
  line.* Follow-on: build+commit-or-archive the portable snapshot so a fresh checkout is hermetic too
  (currently hermetic against this machine's warm AppData cache).
- **0b Real-fisheye corpus.** The synthetic fisheye oracle is disqualified for tuning. Capture
  20–30 real all-sky/wide frames (the rig exists — the `stars-wide-*` sources) across pointings/
  times, with named-anchor validation like the REAL suite. This is the gate every fisheye
  accuracy item (detector V2 package, quad-epsilon, FoV sweeps, k2 re-probe) is blocked on.
- **0c Richer per-run metrics.** Emit per-case CSV (verdict, pose deltas vs truth where known,
  rms, matches, timeMs) instead of grepping PASS/FAIL. Binary verdicts hide accuracy drift; per-case
  timeMs is the only timing signal robust to this machine's mid-run sleeps.
- **0d Cheap dense gate.** A stratified ~20-case RAND2 subset (past failures + depth-instability
  cases) as the standing per-change gate; full RAND2 only pre-merge. With 0a it's also offline.
- **0e Grow REAL + negatives.** Every new capability needs corpus rows before code (the
  wide-guided fisheye gate shipped with zero coverage until mode-4 rows were added). Add the
  wrong-FoV and more near-boundary negatives WS3 identified as missing — they're what reclassified
  `residual` from "inert" to "load-bearing".
  **PARTLY DONE (2026-07-05, commit pending).** Harness now supports an `expectSolved` column: a row
  with `expectSolved=0` PASSES iff the solver REJECTS it, so negatives are proper standing gates
  instead of being read inverted (REAL byte-identical — rows without the column default to
  `expectSolved=true`). Added `test/star-tests-negatives-fov.csv` (grossly-wrong trusted FoV on real
  fields). **It immediately earned its keep — two findings:** (1) **wrong-FoV FALSE POSITIVES** — the
  solver *accepts* a spurious solve at a grossly wrong trusted FoV on DEEP fields: pollux (true 1.27°)
  solves at FoV 0.4°, stars-narrow-5 (true 1.29°) solves at FoV 6.0°. This is a depth-induced FoV-alias
  (the deep catalog offers enough coincidental matches at the wrong scale), analogous to the
  depth-induced roll-alias — the real gap the `fov` gate should close. The two too-far-the-other-way
  cases (pollux 5.0°, narrow-5 0.35°) correctly reject. (2) **cost/fragility** — a grossly-wrong FoV on
  a deep field is pathologically expensive (pollux@5° took 51 s, matched 544 before rejecting) and the
  run intermittently aborts; extreme-FoV inputs stress the search. The mixture verifier (1a) is the
  natural discriminator for the false positives (a wrong-scale solve should score low) — the fix task
  should A/B it there. Follow-ups: fix the wrong-FoV accept gap; add fast SPARSE-field wrong-FoV
  negatives (deep fields are too slow for a routine gate); convert the garbage negatives to
  `expectSolved=0` too.

### Track 1 — Acceptance/selection modernization (the astrometry.net lesson)

- **1a Reformulate the verifier** from the current per-match log-odds *sum* to a per-detection
  foreground/background **mixture with a distractor fraction** (Sutherland–Saunders as
  astrometry.net implements it), normalized so sparse-correct and dense-correct solves are
  comparable. Keep it in shadow mode; success criterion = clean separation of the WS3 accept/reject
  bands that the current formulation fails.
  **DONE (shadow) + measured; NOT ready to gate (commit b7956006d).** `poseVerificationLogOdds`
  implements the per-detection mixture and is recorded as `verify.mixtureLogOddsMilli` alongside
  `faLogOdds`; REAL verdict set IDENTICAL. **Finding (via the 0c metrics + shadow log):** the
  DETECTION-basis mixture does not separate — it goes *more* negative for several correct deep-field
  solves (stars-narrow-3 mix −31361, ngc-2403 −141356, galaxy-m101-1 −94647) than for the pollux
  FAIL (−23216). Cause: deep/nebular fields return hundreds of detections below catalog depth that
  can never match, and the per-unmatched-detection penalty `log(f)` scales with raw detection count,
  so it over-penalizes correct dense solves. **Corrected (commit pending):** the bug was charging
  `log(f)` per unmatched detection — under the proper model an unmatched detection is background under
  BOTH hypotheses, so it carries ZERO evidence. Dropping that term (matched detections contribute
  `log(1 + fg/rho_bg)` against the empirical background density; unmatched contribute nothing) fixes
  the over-penalty: every correct REAL solve is now positive (narrow-3 −31361→10551, ngc-2403
  −141356→98965, m101 −94647→103111). **But REAL cannot demonstrate accept/reject separation:** its
  one FAIL (pollux, mixture 92001) sits mid-range among the passes because pollux fails the harness's
  *named-anchor oracle* (a specific expected star missing), not a false-alarm test — its match quality
  is genuinely good, so no verifier should reject it. A verifier's value shows only on wrong-*pose*
  false-positives. **Next: A/B the mixture vs faLogOdds on RAND2 (wrong-roll aliases) + the
  garbage/near-boundary negatives** — wrong poses should score low, correct high. Until that
  separation is demonstrated, 1b/1c stay blocked. The mixture is now well-formed and stays shadow-only.
  **Negatives measured (2026-07-05):** guided neg-blobs (garbage) scores mixture 49073 — INSIDE the
  correct-solve range (narrow-3 10551 … wide-1 41k) — while still being rejected by the existing gate
  stack. So the mixture is a useful *feature* but NOT a sufficient single-scalar accept/reject gate;
  1b/1c/1d must COMBINE it with the current checks, not replace them with it. Separately, blind
  neg-blobs solves `solved=true` in BOTH V2-on and legacy (identical) — a PRE-EXISTING blind-mode
  garbage false positive at the solver level (harness still flags it FAIL), unrelated to V2 or the
  1a/A1 work; tracked as its own robustness item.
- **1b Use it for candidate *selection* first** (ranking, not accept/reject): the notes already
  identified verifier-driven selection as the principled fix for the catalog-depth instability
  (m51@16's roll-58 crowd-out) and pollux is the same wrong-roll class. A/B against 0d + REAL.
- **1c Then revisit D4 properly:** rank on the single verifier scalar with deterministic
  tie-breaks. This *replaces* the arbitrary band ordering rather than trying to faithfully
  preserve it (D4 proved the current order is load-bearing but meaningless — the fix is a
  principled key, not quantum calibration).
- **1d Gate ablation continues** (WS3 pattern) only after 1a–1c, when the verifier can absorb the
  load-bearing gates' role.

### Track 2 — Fisheye/distortion accuracy (blocked on 0b)

- **A1 Lens inversion fixes** (Newton undistortion with convergence exit; reject the non-monotonic
  k1 fold region). Low-risk, genuinely wrong today, validate on 0b + REAL. Then, and only then,
  the **k2 re-probe** (the original net-negative trial predates these and was likely confounded).
- **A2 Detector V2 as a jointly-tuned package** (σ fix + threshold-multiplier recalibration +
  field-relative hot-pixel + quality-gated small-star admission), tuned against 0b with REAL as
  the gate. Single-variable landing is proven impossible; don't retry it.
- **A3 Distortion-aware blind features:** generate detection quad codes under 2–3 k1 hypotheses
  (bounded ×3 cost) instead of k1=0, targeting the documented `codeMatches=0` root cause on 160°
  fisheyes — the Tetra3-style answer, and the already-refuted alternative (loosen epsilon) stays
  refuted.
  **RULED OUT by measurement (2026-07-05) — do NOT implement as specified.** Two independent blockers:
  (1) NO corpus has a nonzero k1 — synthetic fisheye mode1, real TREx, and real GMN rows are ALL k1=0
  (their curvature is the equidistant/equisolid *projection*, which the blind unproject already inverts
  because the projection TYPE is known in blind mode; k1 is a separate radial term that is simply zero).
  So a k1 sweep has nothing to sweep and cannot be validated. (2) Even the premise is wrong: on
  FISHEYE-mode1, `vectorQuadCodeMatches=0` for 49/50 cases AND yet 38/50 PASS — of those 38, only ONE
  used vector-quad; the other 37 solve via **brightpair** (the distortion-robust blind seeder). So
  vector-quad is not the blind-fisheye path at all — fixing its codes would duplicate brightpair, not
  add recall. The 12 mode1 FAILs are the previously-investigated Cat-C cases (weak oracle / clustered
  named anchors / genuinely sparse), not a seeding-code failure. **To ever revisit A3 meaningfully you
  first need real blind-fisheye rows with a FITTED nonzero k1** (the GMN platepars carry distortion
  coefficients that were set to 0 during row construction — populating them is a 0b/0e corpus task).

### Track 3 — Performance (continue the proven tier)

P-A/P-C removed the redundant-recompute tier. Next, in order:
- **T1 Re-profile** (post-P-A/P-C) on the slowest cases; the prior ranking (outer retry ladder ~30 s
  worst case, rollAliasCheck, acceptance, rollRecovery) predates these changes.
  **DONE (2026-07-05, aggregated across REAL from profileSummary).** `searchBestPose` DOMINATES:
  64.3 s total / 684 ms avg — ~3.5× the next stage. Then search.guided-direction 18.2 s (a subset of
  searchBestPose), acceptance 13.8 s, rollAliasCheck 13.2 s (12 passes), rollRecoveryFinalPass 13.0 s
  (already parallel), full-refine 12.3 s, solve.catalog 11.6 s (I/O, now offline), fovPinnedFinalPass
  5.6 s, coarse-refine 1.9 s. **Implication:** the biggest win is parallelizing/pruning searchBestPose
  itself, but that is the core multi-hypothesis search with the ULP-divergence history — highest value,
  highest determinism risk. The safe T2 targets (rollAliasCheck 12 passes, the lens grid) are each only
  ~13 s, so a determinism-preserving win there is modest. A go/no-go on accepting parallelism in
  searchBestPose (with deterministic index-order reductions + a byte-identical REAL gate) is the real
  decision.
- **T2 Parallelize the embarrassingly-parallel serial stages** with deterministic index-order
  merges: rollAliasCheck's 12 final passes, rescue pass-1, the recenter-ladder batches, the
  125-point lens grid. The worker-context pattern exists; P-A's cache is per-context (thread-safe
  by construction).
- **T3 ULP-affecting micro-items** (projectVector trig elimination; **pin the Wahba convention
  with a unit test** and delete the 4× fallback) — one at a time, full A/B, revert on any REAL delta.
- Skip P-E/P-F/P-G (documented non-bit-identical wrinkles, marginal value).

### Track 4 — External-engine decision (product call, not engineering call)

If **blind narrow deep-field** matters as a product feature: prototype invoking ASTAP (single exe,
CLI) or an astrometry.net-index-based solve for that regime only, behind the existing solver as a
fallback, and A/B it via the harness before committing to WS4 (an in-house HEALPix quad index).
If it doesn't matter: explicitly drop WS4 and the `stars-wide-2` blind rows as non-goals.
Everything else stays in-house — it is the differentiated part. (Verify licences at adoption time:
astrometry.net BSD, Tetra3 Apache, ASTAP GPL-family; SDRangel is GPLv3 so all are compatible.)

### Robustness items worth scheduling regardless

- **R1 Bound the roll-recovery retry loop** — WS3 pass-2 found it can run unbounded (a RAND2 case
  hung only with the rollAlias gate ablated, i.e. it's one gate away from a live hang today).
- **R2 Nested-QEventLoop re-entrancy** (N4): marshal Siril fetches to a worker thread with a
  blocking wait; the star-detector thread should not pump events mid-solve.
- **R3 Unit tests for the geometry primitives** (Wahba convention, projector round-trip,
  undistortion convergence) — the solver has zero fast tests below the corpus level; these are the
  pieces where a silent sign/convention slip is most expensive.

## Process changes ("what we should do differently")

1. **Stop tuning against the synthetic fisheye corpus.** It is a churn generator, not an oracle
   (misled the seed ablation once, the detector work twice). Use it only as a smoke test until 0b.
2. **Hermetic gates.** WS1a made the *code* identical GUI↔harness; 0a extends that to the *data*.
   A gate that can flip from a Hugging Face content change (pollux) is not a gate.
3. **Packages, not variables**, whenever touching co-tuned constants — with the package's tuning
   corpus named up front.
4. **Prefer replacing heuristics with the statistical verifier over calibrating more constants.**
   Every deep-dive (depth instability, D4, WS3) independently arrived at this; it's also how the
   reference implementations solved the same problem.
5. **Record accuracy, not just verdicts** (0c) — a solve can stay PASS while its pose quietly
   degrades 2 px; nothing currently notices.
6. **Merge the review branch back** to the camera branch once RAND2 is re-validated offline —
   11 commits of validated work shouldn't drift.

## Suggested order

**0a → 0d → R1 → T1/T2 (perf while corpora build) → 0b/0c/0e → 1a–1c → A1–A3 → 1d → T3 → Track 4
decision.** Track 0 items are days-not-weeks and everything else compounds on them.
