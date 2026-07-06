# Plate Solver — Updated Plan (2026-07-04)

Supersedes the forward-looking parts of `plate-solver-improvement-plan.md` (WS0–WS5) and
`plate-solver-review-implementation-plan.md` (Phases 0–8) for new work. Both remain the record of
what was done and what was settled. This plan reflects what the 2026-07 review implementation
actually *proved*, compares the design against other plate solvers, and reorders priorities
accordingly.

## Next steps (planned 2026-07-06, after the Tracks 0/1/2 sweep)

The tracks below carry the detailed findings; this is the forward queue. Everything tractable has
landed (R1 budget, 0a hermetic, 0b real corpus, 0c metrics, 0e negatives, 1a shadow verifier, A1
Newton undistortion, A2 detector-V2 default-on, neg-blobs floor). What remains are three root-caused
hard problems plus housekeeping, in priority order:

- **P0 — Bank the branch (~30 min, low risk).** Merge `plate-solver-review` →
  `copilot/create-camera-feature-plugin` (~19 validated commits, zero REAL regression) and rebuild
  `featurecamera.dll` so the GUI picks up detector-V2 default-on, the neg-blobs brightness floor, and
  the R1 solve budget. The worktree branched from the camera branch, so this should be near
  fast-forward; verify with git first.
- **P1 — Wrong-FoV: FoV-escape retry (~half day, medium risk).** Both photometric-gate attempts are
  refuted (see Track 1a): a grossly-wrong TRUSTED FoV yields a fully self-consistent wrong solve. The
  tractable shape is NOT a gate but an ESCAPE, modelled on the existing depth-escape/deepen-escape
  ladder: after a narrow trusted-FoV solve, re-solve at FoV ×{0.25, 0.5, 2, 4} and adopt an
  alternative only if decisively better (matches/rms/faLogOdds margin — the same adopt style as
  rollAliasCheck). First action is a MEASUREMENT: from the 0c metrics + logs, find a cheap trigger
  signature for the two FPs (e.g. matched/candidates fraction — narrow-5@6.0 matched only 629/8580 =
  7%) so the escape doesn't cost seconds on every legitimate solve. Validate: both committed
  wrong-FoV negatives reject, REAL/RAND2 unregressed, per-case timeMs bounded.
  **IMPLEMENTED then REFUTED by RAND2 (2026-07-06) — reverted.** Built the escape exactly as specced:
  a completeness trigger (matched/detection < 0.05 OR matched/candidate < 0.10), re-solve at a
  ratio-estimated FoV ×{1,0.7,1.4}, adopt only a healthy result, else REJECT the wrong-scale solve.
  On the two FPs it worked beautifully — narrow-5@6.0 RECOVERED the true pose (fov 6.0→1.27, rms 1.37,
  507 matches) and pollux@0.4 was rejected — and REAL was byte-identical (escape fired 0×). BUT RAND2
  broke it: a correct SPARSE deep field has the SAME low matched/detection as a wrong-small-scale solve
  (synth-rand-a-001: 5/5 candidates matched at rms 0.375 — a perfect solve — yet m/det 0.032 → falsely
  rejected), and a-002/003 got dragged to wrong wider FoVs and adopted. The completeness signal
  OVERLAPS correct sparse solves (pollux 0.027 vs rand-a-001 0.032) — not separable. **A real wrong-FoV
  fix needs a SCALE-INDEPENDENT verifier (the true pose's inter-star angular geometry matches the
  catalogue at exactly one scale; a wrong scale mismatches), not a match-completeness heuristic — this
  is the same normalized-verifier gap as Track 1, now with a second hard corpus (the FPs) to gate it.**
  Do not re-attempt the completeness heuristic.
- **P2 — TREx wrong-pose: true-pose diagnostic (~1–2 h, no risk — measurement only).** The fisheye
  failures are wrong-pose ACCEPTANCES (Track 0b triage). Split search-vs-acceptance the established
  way: take 3–4 failing TREx rows, add mode-4 (FovAzElRoll) variants seeded with the skymap-derived
  truth, and run. Solver accepts truth → search problem (wrong basin outscores true one on real
  all-sky data). Solver rejects truth → acceptance problem (gates mistuned for real fisheye). The
  outcome defines the actual fisheye work; do not start solver changes before this.
  **DIAGNOSTIC DONE (2026-07-06) — it is ONE phenomenon: rotation-about-boresight aliases with NO
  alias defense on wide fields.** The TREx rows are mode-3/mode-1 pairs; mode 3 already trusts the
  truth direction, so no new runs were needed. Mode split: mode-3 (truth az/el) 2 PASS / 7
  wrong-pose-accepted / 1 not-solved; mode-1 (blind) 2 PASS / 8 wrong-pose-accepted. The mode-3
  failures divide into boresight-correct cases (azD 0.4–2.6°) and az-alias cases (azD 29–114° at el
  83–88.5°) — but near zenith an azimuth error IS a rotation about the boresight, and the
  boresight-correct cases show the smoking gun: **anchor displacements are pure rotations about the
  image centre at constant radius** (eltanin: 301 px error = 101° rotation, radius 198→192 px;
  mirach: −64°/−125° rotations, radius 190→158/174 px). All TREx cameras point near-zenith, mode 3
  pins the boresight, and rotation is the one unconstrained DOF. **Code-level root cause:
  `hasCompetitiveRollAlias` early-returns for `!isNarrowField` — the entire rotation-alias defense
  (and the rollRecovery pass, same gate) is narrow-only, so wide all-sky solves accept whichever
  rotation matches most faint stars coincidentally.** The defined fix: extend the rotation-alias
  check (bright-weighted faLogOdds compare, the proven narrow mechanism) to wide direction-seeded
  solves, with a per-alias roll refinement so the coarse 30° offsets converge onto the true rotation
  (measured rotation errors are 60–125°, i.e. within the offsets' reach after refine). Validate on
  TREx mode-3 (expect several of the 7 to fix), REAL wide rows (near-zenith — must not start
  rejecting correct solves), FISHEYE-mode4, WIDE.
  **FIX ATTEMPTED + REFUTED — and the refutation flips the diagnosis: the TREx anchors are probably
  MISCALIBRATED, not the solver wrong (2026-07-06).** Extended the alias-adoption path to wide
  (adopt-only, narrow rejection skipped) + a 10° wide roll grid so competitors survive the filter;
  REAL stayed byte-identical. But on the failing TREx mode-3 cases the adoption never fires because the
  **winner OUT-SCORES the anchor-"true" rotation on bright-weighted faLogOdds** (luck_20251220_1200:
  winner roll −126 faLogOdds 36.3 vs the roll-133 competitor — the true rotation per the anchors —
  only 26.2). Three facts together indict the corpus, not the solver: (1) the winner's az/el already
  equal the seed truth (240.3/86.7 vs 242.9/86.03) — only ROLL differs; (2) the winner is a genuine
  52-match fit that the solver's own faLogOdds/score prefer over the anchor rotation; (3) the anchor
  errors are PURE rotations about the image centre. That is the exact signature of a per-camera ROLL
  offset baked into the skymap→pixel anchor derivation (the TREx `.sav` skymap's rotation reference),
  i.e. the solver is finding the correct pose and the committed anchors are rotated. **Next (before any
  more fisheye solver work): validate the TREx anchors** — re-derive them / independently astrometric-
  check one frame, or diff the per-camera solver-roll-vs-anchor-roll offset for consistency within a
  camera. If confirmed, TREx's 4/20 is a corpus bug and the real solver pass rate is much higher; fix
  the anchor derivation, not the roll-alias defense. (Bright-weighted faLogOdds cannot arbitrate
  rotations in a dense all-sky field regardless — a wrong rotation still lands many bright catalogue
  stars — so the wide rotation-alias defense was the wrong lever even if the anchors were right.)
  **CORRECTION (same session): the anchor-miscalibration hypothesis is WEAKENED — do not assume a
  corpus bug.** Consistency check: within the luck camera, frames 060030 and 090030 PASS while 030030
  and 120030 FAIL. A systematic per-camera anchor roll offset would fail ALL luck frames, so luck's
  anchors are sound (the per-frame roll differences are just sidereal sky rotation over the capture
  times). So the failing frames are GENUINE wrong-roll acceptances: on these (likely sparse/noisy)
  all-sky frames the solver lands on a roll that out-scores the true roll on EVERY available metric
  (faLogOdds, match count, rms) — the true pose genuinely fits worse in the data. This is the same
  no-clean-discriminator wall hit by the mixture verifier (garbage), the wrong-FoV FPs, and the P1
  completeness heuristic: for these hard data-limited cases there is no scale/rotation-independent
  signal that separates right from wrong. **Realistic conclusion: the remaining TREx fisheye failures
  are largely intractable with the current feature set — not a quick fix and possibly not worth
  chasing.** The one concrete residual action is still worth doing: spot-check 1–2 TREx anchors
  astrometrically to rule the corpus fully in or out before writing them off.
- **P3 — Verifier separation study on RAND2 (~1 h compute, no risk).** 1b/1c are blocked on
  demonstrating wrong-pose separation. RAND2's known wrong-roll/alias tail is the right labelled set:
  run RAND2 offline (chunked) with the shadow metrics and compare mixture/faLogOdds on correct vs
  wrong-roll cases. Separates → 1b (verifier-driven candidate *selection*) becomes viable for the
  catalog-depth-instability class. Doesn't → close 1b/1c as blocked pending a better formulation.
  **DONE (2026-07-06) — it SEPARATES; 1b is unblocked (with a caveat).** On the RAND2 cases that solved
  offline (cache-limited to 67/150, but the split is what matters): correct (PASS, n=67) mixture
  min 7407 / p25 10562 / median 12489 / max 405k; wrong-pose-accepted (FAIL-but-solved, n=4) mixture
  6459–9786 — **all 4 below the correct p25, 0/4 overlap**; faLogOdds separates the same way (wrong
  ≤5376 vs correct p25 7466). This is the wrong-ROLL class, exactly what 1b targets, and it vindicates
  the original premise. **Reconciles the earlier "mixture is not a gate" negatives:** neg-blobs
  (garbage) and wrong-FoV (wrong-scale) score HIGH because they have coincidentally-good match sets;
  the wrong-ROLL class has genuinely worse star agreement and scores LOW. So the mixture is a
  discriminator for candidate SELECTION within a case (rank correct-roll above wrong-roll), NOT a
  universal accept/reject gate. **1b action:** wire the mixture as the ranking key for competing
  direction-seeded candidates (the roll-alias / catalog-depth-instability class), NOT as a global
  floor. Caveat: only 4 wrong cases here (offline cache limit) — confirm on a warmed-cache RAND2 run
  (P5) before committing a selection change. The elevation-seed/blind-grid faLogOdds compare from
  P2's fix (wide rotation-alias defense) is the same mechanism applied to a different regime.
- **P4 — T2: searchBestPose restructure (dedicated session, high care).** The 64 s bottleneck.
  Spec: batch-parallel evaluation of the guided-direction grid (~18 s first target) in CANONICAL
  order with the serial reduction/early-exit logic applied post-batch (byte-identical by
  construction; wasted evals past an early exit are the price), using a PERSISTENT pool (member of
  CameraPlateSolver, not per-call — the per-call spawn overhead is what made the lens-grid trial
  1.7× slower). Gate: REAL byte-identical + per-case timeMs down on narrow-7/dense cases.
  **DONE (2026-07-06, commit pending).** Parallelised the guided-direction grid: the no-early-stop
  branch (`!allowGuidedEarlyStop`, the dominant cost) is a fixed evaluation set, so it is flattened to
  a `(fov,el,az,roll)` point list, evaluated on per-worker `SolverContext`s (each builds its own
  blind-grid cache per cell, `evaluatePoseFromPrecomputedCatalog` is write-free), then the reduction
  (logging, candidate-pool insertion, best-tracking) is REPLAYED serially in canonical order — the
  same code path as the serial `evaluateSeedFromCache`, so byte-identical by construction. The
  early-stop branch keeps the original serial loop. Used the existing `refinementWorkerThreadCount`
  sizing (not a per-call `QThreadPool` object churned each solve, unlike the failed lens-grid trial —
  here one pool per searchBestPose call over ~thousands of evals amortises). **Measured: guided-direction
  25958→5932 ms across REAL (4.4×), REAL 47/48 byte-identical.** (Note: further gains want the
  persistent-member pool + parallelising the elevation-seed and blind grids too — follow-on.)
- **P5 — Housekeeping (fill-in).** 0d stratified ~20-case RAND2 fast gate; convert
  `robustness-neg*.csv` to `expectSolved=0` and add fast sparse-field wrong-FoV negatives (deep-field
  ones take ~50 s each); build+archive the portable hermetic catalog snapshot so a fresh checkout is
  hermetic; keep the plan/memory current.

P1/P2/P3 are independent of each other; P2 and P3 are pure measurement and can interleave with
anything; P4 is standalone and should not share a session with behaviour-affecting solver changes.

### Verifier study Tier 1 (2026-07-06): per-candidate labelled dataset — first results

Infrastructure landed: `SDRANGEL_CAMERA_PLATE_SOLVER_CANDIDATE_DUMP=1` makes `evaluateFinalMatchPass`
emit one machine-readable line per final-pass candidate (pose, D/C/M, match-tightness ladder
m2/m4/m8/m16, rms/med/max, radius, image size, bright stats — raw quantities only, features are
engineered offline). `test/verifier_study.py` labels candidates against corpus truth (a PASSING
case's anchor-certified accepted pose upgrades to full-pose truth incl. roll; FAIL cases can only
mint WRONG labels; negative rows are WRONG-throughout), reconstructs PowerShell's 120-col stderr
wrapping, clusters candidates into pose hypotheses (best-M evaluation per cluster = "finalist"),
and reports per-class separation. One collection run (REAL + TREx + negatives) = 44.5k labelled
candidates → ~2.9k labelled finalists.

**Findings (finalists, M>=6):** (1) **Excess-over-chance at the radius ladder (`eocBest`) has real
within-case RANKING value in the narrow regime** — real/RIGHT median 1.8 with p90 182 vs real/WRONG
median 0.1 p90 11, wrongscale/WRONG rarely exceeds ~1. Corroborates P3: verifier-as-selection (1b)
is supported by 1.9k finalists, not just 4 cases. (2) **All-sky rotations remain inseparable**
(allsky RIGHT −0.1 vs WRONG −0.5, tightness identical): with C≈776 candidates on a 553×480 frame
the chance model saturates, and the real lens-model error spreads TRUE matches into the same
8–11px band as coincidences — count/tightness features cannot work there even in principle.
(3) Naive per-evaluation labelling conflates pose correctness with evaluation-stage fit quality —
the cluster-finalist granularity is the right one (lesson baked into the script). **v2 path:** the
all-sky class needs features absent from this dump — residual-field coherence (per-match dx,dy
vectors: true-pose residuals fit a smooth radial/rotation field, coincidence residuals don't) and
hold-out prediction. Extending the dump with compact per-match residuals for finalists is the next
concrete step; also worth re-running collection on RAND2 (adds the wrong-roll class at scale).

**VERIFIER PROGRAM — CONCLUDING SUMMARY (2026-07-06, after the Tier-2 strengthened test).** With the
corpus doubled (trex2: 10 new frames, 3 PASS — same rotation-alias failure class confirmed by pose
deltas), the combined all-sky labelled set (17 RIGHT / 437 WRONG cluster-finalists) settles bpFrac:
the population-level signal is real (RIGHT p10 0.12 vs WRONG median 0.00) but the decisive
WITHIN-CASE ranking test fails — max-bpFrac picks the RIGHT finalist in only **1/5** cases where one
exists. That was the last designed feature. Program totals: ~45k labelled candidates; features
evaluated = count, rms, tightness ladder, multi-radius excess-over-chance, brightness fractions both
directions, magErr, rank error, mixture, faLogOdds, residual-field coherence, hold-out; wiring
attempts gate-refuted = P1 FoV-escape (RAND2), P2-fix wide alias adoption (winner out-scores truth),
1b mid-selection (RAND2: 66 vs 67). **Conclusion: with match-statistics features, all-sky rotation
selection is NOT solvable — the information is not in the match set at this star density and lens
accuracy. The narrow wrong-roll class is already handled at the final level (rollAlias adopt). Stop
verifier feature work; the remaining paths are physical, not statistical: (a) a better lens model for
all-sky (reduce the true pose's 8–11px residual band so tightness separates), or (b) external
bright-star identification (match the handful of first-magnitude stars by absolute brightness +
pattern, independent of the solve).** The labelled dataset + tooling stay for future re-tests.
**CLOSING INSIGHT (RAND2 dump, 224k candidates):** in the narrow regime, within-case ranking by
eocBest picks the RIGHT finalist in **21/21** testable cases — but RIGHT labels exist only for
PASSING cases (certified poses), so this shows eocBest *perfectly agrees with already-correct
selections* (it would be a safe final-level confirmation) while the FAILING cases contribute no
RIGHT finalists at all. This explains every selection-level refutation at once (P3 separation but 1b
zero gains; 21/21 only on passes).

**CORRECTED BY THE rollRecovery ABLATION (2026-07-06) — it is a DISCRIMINATION limit, not a search
gate.** I added a behaviour-inert `SDRANGEL_CAMERA_PLATE_SOLVER_FORCE_ROLL_RECOVERY` flag that
bypasses the `selectionHasStrongSupport` gate so the 21-offset roll grid runs on the failing tail,
plus `rollrecovery_reach.py` which labels each dumped candidate against exact CSV truth (reach = a
candidate at the true az/el/roll/fov; select = the accepted verdict at truth) AND compares fit
strength at truth vs the selected/best pose. Two runs:
- **RAND2 (83 offline fails):** forcing recovery flips 0/83 to PASS; the true pose is geometrically
  "reached" 83/83 but its median match count is **1** — because RAND2 is 100% Siril and OFFLINE
  starves the true-pose sky region of catalogue. This is the same offline artifact as the 67/150
  number, NOT evidence about search. (The apparent "search failure" in the prior insight was reading
  this starvation.) Several fails also show a wrong-roll pose with 30–73 matches at rms 0.2–1.8 while
  the CSV-"true" roll fits ~10 loosely — i.e. the RAND2 offline corpus cannot be trusted for this
  question (starvation + high-elevation az/roll degeneracy + possible render-roll convention).
- **REAL (48, HYG local catalogue — trustworthy, no starvation):** forcing recovery makes REAL
  *worse*, 46/48 vs 47/48 (regresses m101), and pollux still fails.

**AGGREGATE conclusion (stands): forcing more roll search is net-harmful on trustworthy data, so
`selectionHasStrongSupport` is PROTECTIVE and stays; no code change warranted.** This matches the
verifier program's wall from the search side.

**PER-CASE narrative CORRECTED (2026-07-06, roll-truth + FoV/roll sweeps) — do NOT cite the earlier
"true pose out-competed by a wrong roll" story for pollux/m101; it rested on a labelling flaw.**
`rollrecovery_reach.py` compared each candidate's roll to the CSV `roll` column, but for **mode-3
(FovAzEl) rows that column is a PLACEHOLDER (0), not the true roll** — mode 3 does not seed roll.
Consequences once corrected:
- **m101's true roll is ~87°** (its mode-6 CurrentSettingsOnly row carries roll=87.2 and PASSES). So
  the "wrong-roll 178-match pose at roll 87" I had flagged is actually the CORRECT pose, and it IS the
  match-count winner (178 > the roll-0 alias's 94). m101 is NOT a discrimination example — it solves
  correctly in normal operation; only *forced* recovery destabilised it.
- **pollux has no tight solution at ANY pose or scale — it is a hard/degenerate image, not a
  discrimination case.** A mode-4 roll sweep (−165…180° step 15) and a mode-3 FoV sweep (0.6…2.8°)
  both hold **rms ≈ 15.6–16.7 at every roll and every FoV**; match count just scales with field size
  (14→463 stars falling inside the 24px radius) while rms never approaches the ~2px of a real lock. So
  the 157-vs-238 "true vs wrong" was two equally-loose ~16px coincidental fits — there is no tight true
  pose being out-competed. pollux's single validation anchor (Pollux itself, centre) can't constrain
  roll regardless.
So on the trustworthy REAL corpus there is NO clean "true tight pose loses to a wrong pose" case; the
genuine discrimination failures are the **all-sky TREx fisheye** wrong-roll acceptances, plus the
verifier program's feature-space result. State it that narrowly. Ablation flag reverted
(byte-identical); `rollrecovery_reach.py` kept but its roll labels are only valid where CSV roll is
real (mode 4/5/6 or synthetic with a trustworthy render roll — NOT mode-3 placeholders).

**PHYSICAL-LEVER PROBES (2026-07-06) — BOTH refuted for their target; measured, not assumed.**
- **Lever A (better lens model): REFUTED for the narrow cases.** The pollux sweeps above show rms flat
  ~16 across all rolls AND all FoVs — no distortion/scale sweet spot exists to shrink, and a 1.27°
  rectilinear field has negligible radial distortion anyway. Lever A's only real domain is all-sky
  fisheye — and there the TREx *skymap already provides a near-exact per-pixel lens* (Tier-2 used it),
  so "better lens model" for TREx is DATA PLUMBING (feed the skymap/OcamCalib intrinsics as fixed
  per-camera calibration) not a modelling breakthrough; it helps only calibrated cameras, not blind.
- **Lever B (absolute bright-star ID via COLOR): REFUTED as a standalone roll discriminator on TREx.**
  Probe (`images-real-fisheye-staging/trex/color_probe.py`, samples RAW LINEAR h5 RGB, not the stretched
  PNG; 12 frames, 92 bright stars el>30°): pooled Spearman |measuredColor vs catalog B–V| = **0.22**
  (best-populated frames 0.33), correct sign, some frames −0.6/−0.7 — but **below the 0.5 bar**, and the
  apparent "collapse at wrong roll" is mostly a PRESENCE/geometry cue (73% of roll-90 positions land on
  empty sky), which is exactly what the solver already uses and what fails. Limiter = data quality
  (8-bit RGB, ~6–13 bright stars/frame), not color in principle; a higher-bit-depth camera *might* do
  better. Verdict: color could serve as a WEAK PRIOR alongside geometry, but will not by itself break
  the wrong-roll degeneracy. Deliverables kept: color_probe.py + color_probe_results.csv (92 rows).
  **DEEP-STACK FOLLOW-UP (2026-07-06) — the 0.22 was a real color ceiling, NOT SNR-limited, so MORE
  8-bit TREx data will not help.** Re-ran with a full ~20-frame per-minute median stack (drift 0.65px
  < 1px so no registration needed): pooled |trueCorr| moved only 0.22→0.27 (well-populated 0.33→0.36),
  still far below 0.5, and the SNR≥5 star count was UNCHANGED at 92 — the limiter is the NUMBER of
  Vmag<3.5 stars above 30° per fisheye frame (~6–13), not photon noise, and the true-pose color scatter
  is a SYSTEMATIC (camera spectral response + 8-bit quantization) that averaging can't remove. Where
  stars are plentiful (n≥13) color DOES reach |ρ|≈0.8 — so color is viable IN PRINCIPLE, just not with
  this camera's few-bright-stars/8-bit data. **CONCLUSION on downloading more data: it does NOT help
  the accuracy problem** — pollux is a bad image, TREx's lens is already known (skymap), and color's
  ceiling is the camera hardware, not exposure/quantity. Real progress needs a DIFFERENT sensor
  (higher bit depth + more resolved bright stars) or the skymap-intrinsics code path — not more frames.
  Deep-stack deliverables: color_probe.py (`--deep`/`--stack N`/`--noreg`) + color_probe_results_deepstack.csv.

**★ ROOT CAUSE OF THE ALL-SKY FAILURES FOUND (2026-07-07) — it's a HANDEDNESS/MIRROR bug, NOT a
discrimination limit. This overturns "accuracy investigation closed."** While prototyping the
skymap-as-fixed-intrinsics path, a fit of the solver's EXACT parametric projector to the TREx skymaps
(`images-real-fisheye-staging/trex/skymap_fit.py`) showed:
- **MIRRORED equidistant reproduces the skymap at 0.26px median** (max 0.9px), with clean physical
  near-zenith params (halfHFov 87.3°, k1 −0.046). **Non-mirrored fits at 184px for ANY projection
  type or fov.** (An earlier unbounded fit's "equisolid 2.7px" was an ARTIFACT — the optimiser
  absorbing the mirror via a flipped boresight; with an explicit mirror flag + physical bounds the
  truth is unambiguous: the lens is equidistant, the IMAGE is mirrored.)
- **Mechanism:** the solver's projector is orientation-PRESERVING (a rotation, no reflection). An
  up-looking all-sky camera images the sky REFLECTED, so no pure roll/pose can fit it — the solver
  lands on the least-bad wrong-roll pose at 6–14px. This is exactly the "genuine wrong-roll
  acceptances at 100–200px anchor error" the TREx failures always showed.
- **END-TO-END CONFIRMED:** horizontally flipping the TREx images (+ anchor x→W−1−x) and re-solving:
  **trex 4→10/20 PASS, trex2 3→5/10**, with rms dropping widely (trex2 median passing rms 12.8→8.5;
  luck-0223 12.8→6.8, luck-0115 12.9→6.1). The residual (still ~6–14px, not 0.26px) is the solver's
  blind fit not nailing fov/k1/pp — which is where the FIXED-INTRINSICS finishing step closes the gap.
- **The solver IGNORES the existing `m_flipX`/`m_flipY` settings** (they only drive the display/capture
  pipeline: camera.cpp / cameraimageprocessor.cpp; the solver never reads them). So a user who flips
  their all-sky feed for display still solves against un-flipped geometry — or, as here, the saved
  frames are mirrored and nothing corrects it.

**FIX (the real, cheap, high-value one — the first genuine accuracy win of this arc):**
1. **Mirror-aware projection** — negate the x-axis pixel mapping in createProjector/unprojectPixelToVector
   when the image is mirrored. Minimal (a sign gated on a flag), byte-identical when off. Wire it to
   the existing `m_flipX` (make the solver honor the flip settings) and/or a lens "mirrored" flag.
2. **Auto-handedness** for blind/uncertain solves — try both mirror states, keep the tighter fit — so
   users who don't know their camera is mirrored still solve. (All up-looking all-sky cameras have
   this property; it's a general feature, not a TREx quirk.)
3. **Fixed intrinsics** (the original P-lever) as the finishing step — feed the skymap-fitted
   equidistant fov/k1/pp so the mirror-corrected solve tightens from ~8px to ~1px and passes broadly.
Validate mirror-OFF byte-identical on REAL/FISHEYE/WIDE (none are mirrored) and the TREx gain with it
ON. This is now the top forward item, above the P0 merge.

**CORRECTION to the earlier "every lever measured, investigation closed at evidence floor" line: that
stands for the NARROW dense-field cases (pollux/RAND2) and for COLOR, but is WRONG for the all-sky
TREx class — those failures were a handedness bug with a concrete fix, not a fundamental limit.**

**NET (2026-07-06): every lever — verification, search-expansion, selection-reranking, better lens
model, color ID — has now been measured and none cracks the all-sky rotation class with the current
cameras/corpus. The accuracy investigation is closed at its evidence floor. Genuinely new signal would
require BETTER DATA (higher-bit-depth RGB for color; a per-camera fisheye calibration for TREx-class
rigs), not more algorithm on the existing frames.**

**v2 DONE (2026-07-06): residual-field coherence REFUTED for all-sky; missed-bright-prediction is
the first (weak) all-sky signal.** The dump now carries subsampled per-match residual vectors
(`res=x:y:dx:dy;…`, finalists only) and the study computes field-coherence R² + even/odd hold-out
(linear+radial 5-term basis, tiny built-in least-squares). Result on cluster-finalists (M≥10):
allsky RIGHT R² 0.10 vs WRONG 0.11, hold-out 1.12 vs 1.13 — **identical**. Why the physics didn't
transfer: at 24 px greedy matching in ~776-candidate density, even the TRUE pose's match set is
contamination-dominated, so its residual field is as noisy as a wrong pose's; the smooth lens-error
signal is swamped by wrong correspondences. (In the narrow regime coherence does show a tail:
real/RIGHT p90 0.72 vs WRONG 0.21 — but eocBest is stronger there anyway.) **The one feature with
any all-sky signal is `bpFrac` (missed-bright-prediction, already in the v1 dump): allsky RIGHT
median 0.12 (p10 0.12) vs WRONG median 0.00 (p90 0.25)** — wrong rotations explain the detections
(bdFrac 1.0 for everyone: dense mag-6 catalog) but strand the brightest PREDICTED stars on empty
sky, exactly the designed asymmetry. Overlapping and only n=6 RIGHT finalists, so: usable as a
within-case RANKING input at best, and needs more RIGHT all-sky data (Tier-2: more TREx frames)
before any solver wiring. **Verifier program status: narrow regime → eocBest/mixture ranking (1b)
is validated and implementable; all-sky regime → only bpFrac shows signal, pursue via ranking +
more data; single-scalar cross-class gating remains impossible with match-statistics features.**

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
  **COMMITTED as a durable benchmark (2026-07-05, commit 029f69659).** TREx 180° all-sky fisheye
  (20 rows) + GMN/RMS wide rectilinear (6 rows), with their external ground truth (GMN platepars incl.
  fitted distortion + star_list, CALSTARS, metadata, READMEs); the large frames/skymaps/masks stay
  gitignored (the CSV rows carry the derived anchors). **It is a hard CHALLENGE set, not a green gate:
  current pass rate trex 4/20 + gmn 3/6 = 7/26** — it documents where the solver still fails on real
  fisheye/wide and is the Track-2 accuracy target. **k1-fit outcome (definitive):** attempted to
  populate real nonzero k1 from the GMN platepars, but the solver's OWN joint lens recovery returns
  **k1 = 0** on every passing GMN case (AU000C/D/G solve at k1=0). An empirical platepar `star_list` fit
  gave spurious ~0.05–0.10 only because the optical axis had to be approximated (the platepar `RA_d` is
  not the star-frame centre; a proximity-weighted axis fakes a radial trend). So **the real corpus has
  near-zero residual k1 in the solver's rectilinear/fisheye convention — there is nothing meaningful to
  populate**, which also confirms why A3's k1-sweep had no target. A proper platepar→k1 fit would have
  to solve axis + k1 jointly, and even then the solver already recovers it. Follow-up worth more than
  k1: triage the 19 failing rows (solver-failure vs strict named-anchor oracle) to sharpen the gate.
  **Triage done (2026-07-06):** of the 19 fails, **16 are solved=1 but oracle-rejected** (9
  position-mismatch, 7 missing-stars) and only **3 are genuine search failures** (solved=0). So the
  corpus is chiefly an ACCURACY gate: the solver finds a pose but the named anchors land off. The
  position-mismatches concentrate in the TREx 180° fisheye — pose accuracy degrades at the field edge
  where the real lens deviates from a pure equidistant model. This is a Track-2 projection-accuracy
  signal (edge distortion), NOT a seeding gap. Next: measure the per-anchor error magnitudes to split
  "borderline oracle (25-40px, tighten model)" from "real pose error (100px+)".
  **Per-anchor measurement DONE (2026-07-06) — the edge-distortion hypothesis is REFUTED; it is
  wrong-pose acceptance.** Parsed the 25 mismatched TREx anchors (`position mismatches:` lines, 553x480
  frames): **19/25 are gross (>=100px)**, only 2 borderline. Errors are large at ALL radii (inner
  meanErr 72px, mid 209px, edge 196px) — NOT an edge-growing pattern, so a higher-order radial term (k2)
  would not fix it. Shift-vector analysis on the only statistically-meaningful frame (6 anchors) is
  SCATTERED (mean shift 36px vs 205px scatter), i.e. the anchors land in different wrong directions —
  the solved POSE is in a wrong basin, not a uniformly-offset/edge-warped correct pose. (The anchors are
  trustworthy: overlays trace coherent constellations.) **Conclusion: the fisheye failures are wrong-pose
  ACCEPTANCES on hard 180deg all-sky frames — the solver accepts a plausible-looking but wrong pose that
  matches many stars coincidentally, the same class as the wrong-FoV/roll aliases. The fix is better
  all-sky search (find the true basin) or better wrong-pose rejection (verification), NOT a distortion
  model.** This redirects Track 2 away from k2/distortion and toward the Track-1 verifier problem (which
  the shadow measurements show the mixture cannot yet solve — garbage/wrong-scale poses score high).
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
  **Wrong-FoV gap is FUNDAMENTAL — no photometric gate fixes it (2026-07-06, second pass).** Chased a
  `brightDetectionMagnitudeError` discriminator (search candidates for the wrong-FoV FP show magErr ~7,
  vs <=1.5 for any correct solve) and added a magErr>3.0 reject to the narrow brightness gate. It was a
  NO-OP: REAL/FISHEYE/WIDE byte-identical AND both FPs still solved. Root cause: the high magErr is only
  in REJECTED search candidates; the FINAL ACCEPTED pose has acceptable magErr, rank error, and
  geometry — a grossly-wrong TRUSTED FoV produces a *fully self-consistent* wrong solve that passes every
  photometric gate (it matches a coherent set of catalogue stars at the wrong scale). Reverted. **So the
  only real fix is FoV-RANGE VERIFICATION — don't fully trust the seed FoV; search a bounded FoV range
  and adopt the best-verified scale (astrometry.net does not trust an input scale). That is a design
  change with regression/perf risk to legitimate trusted-FoV solves, deferred as its own item.** The
  earlier mixture attempt below is also part of this refutation.
  **Wrong-FoV discriminator REFUTED by measurement (2026-07-06):** the mixture does NOT separate the
  wrong-FoV false positives either — pollux@0.4 scores 16070 (ABOVE the correct-solve minimum 10551) and
  narrow-5@6.0 scores 465891 (matched 629 stars at the wrong scale, mid-range among correct solves). A
  grossly-wrong trusted FoV on a deep field yields a large SELF-CONSISTENT coincidental match set, which
  the mixture (rewards matched, unmatched contribute 0) rates highly. So neither a mixture floor nor the
  per-match faLogOdds fixes the wrong-FoV gap; it needs a dedicated FoV-plausibility check (e.g. matched
  angular spread vs pixel spread, or bright-star geometric scale consistency), which is a separate,
  higher-risk mechanism deferred as its own item. The mixture verifier remains shadow-only.
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
  **IMPLEMENTED (2026-07-06, uncommitted pending RAND2 A/B).** In `applyRefinementResult`'s refined-
  finalist keep-best (cameraplatesolver.cpp ~1116): for narrow direction-seeded solves with UNKNOWN
  roll, when challenger and incumbent finalists disagree on roll by >5°, a bright-weighted
  faLogOdds margin ≥15 (the proven rollAlias adopt margin) overrides the count/score comparator in
  either direction (adopt a bright-better challenger / veto a bright-worse one); non-decisive
  comparisons keep legacy order exactly. REAL: 47/48 IDENTICAL, override fired 0× (inert there).
  Decision gate: post-1b RAND2 vs the pre-1b baseline (collected with the candidate dump) — expected
  effect is on the wrong-roll tail (a-005/c-009 class); any PASS→FAIL = revert.
  **REFUTED BY THE GATE (2026-07-06) — reverted.** Post-1b RAND2: 66/150 vs baseline 67 — one loss
  (synth-rand-c-011), ZERO gains, with the override firing 529 times. Why the offline separation
  didn't transfer: the P3/Tier-1 statistics hold at the FINALIST level, but the keep-best injection
  point sees mid-refinement comparisons whose statistics differ, and the wrong-roll FAILURES mostly
  lose before this comparison (no gains available there), while perturbing 529 comparisons flipped
  one previously-good downstream chain. **Key realisation: final-level verifier arbitration ALREADY
  exists and is shipped — `hasCompetitiveRollAlias`'s bright-logOdds adopt path is exactly 1b applied
  at the correct (fully-refined, final) level.** So 1b-as-mid-selection is closed; the verifier
  program's remaining lever is extending the FINAL-level arbitration's reach (more alias hypotheses
  at final level), not reranking mid-pipeline. Do not re-attempt the keep-best injection.
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
  **ASSESSED (2026-07-06) — no safe quick win; here is the map for the dedicated effort.** The
  proven deterministic pattern is `QThreadPool` + per-worker `SolverContext(m_owner)` +
  `copySearchStateFrom(*this)` + serial index-order merge (see `buildBrightPairBlindSeeds` ~6147 and
  the `evaluateRecoveryPosesParallel` lambda in solve() ~1309, already used for rollRecovery/fovPinned).
  Applying it to the remaining stages runs into two walls:
  (1) **The 64s is in `searchBestPose`'s core** (narrow-guided multi-hypothesis, blind grids), which is
  NOT a pure map — it is riddled with early-exit `break`s and candidate-pool mutation, so a naive
  parallel map evaluates poses the serial version skips and the keep-best diverges. Safe parallelism
  there needs restructuring to split the pure evaluate-grid phase from the early-exit refinement.
  (2) **The clean map+reduce stages are either small-N or state-uncertain.** `hasCompetitiveRollAlias`
  (~10682) IS a clean map+reduce (12 independent rollOffsets → `evaluateFinalMatchPass` + a reduction:
  keep-best-adoptable-by-logOdds first-on-tie / OR-ambiguous / last-ambiguous-reason, no cross-iteration
  break) — but at only 12 items × ~12 ms across ~94 calls, per-call threadpool spawn overhead would eat
  the ~11 s. The **125-point lens grid** (cameraplatesolver.cpp ~872, pure `evalLens` argmax via
  `isBetterSeedAnchored`, no early-exit) is the best-shaped safe target (N=125, ~1.25 s/wide-case), BUT
  it is only byte-identical if `evaluatePose` is state-PURE across the 125 sequential calls — if it
  accumulates search state (e.g. `m_weakModeNormalizationPixels` via max()) that a later call reads, the
  serial loop is order-dependent and per-worker copies diverge.
  **AUDIT + EMPIRICAL TRIAL DONE (2026-07-06) — lens grid is safe but a NET SLOWDOWN; reverted.** The
  purity audit PASSED: the copied scoring-preference members are written only in solve()/searchBestPose
  setup (~190, ~12073), never during pose evaluation, and `evaluatePose`'s only member write is its own
  per-context projected-catalog scratch (overwritten each call). So the lens grid WAS parallelized with
  the per-worker-context + serial index-order argmax pattern and **validated byte-identical (REAL 47/48
  IDENTICAL)** — determinism is fine. BUT it ran **~1.7x SLOWER** on every wide case (wide-7 238->382 ms,
  wide-8 246->429, wide-9 2670->4731): the grid is a cheap, frequently-reinvoked stage, so the
  per-invocation `QThreadPool` spawn + `copySearchStateFrom` per worker dominates the ~ms of compute
  saved. Reverted. **Conclusion (now empirical): the safe/clean stages are NOT worth parallelizing — the
  wide lens path is already fast (~240 ms) and is not the bottleneck; the 64 s is in the NARROW-guided
  `searchBestPose`, which is early-exit-laden and cannot be parallelized without restructuring its pure
  evaluate-grid phase apart from the refinement, AND using a persistent/shared thread pool (not
  one-per-call) to avoid the spawn overhead this trial exposed.** That is the genuine T2 effort — a
  focused project, not a drop-in; a rushed version risks the byte-identical GUI==harness guarantee
  (the WS1a saga).
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
