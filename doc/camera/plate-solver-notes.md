# Plate Solver — engineering notes

Working notes on `plugins/feature/camera/cameraplatesolver.cpp`, captured while
reviewing the acceptance logic and the `featurecamera_star_tests` corpus. See
`AGENTS.md` for how to build/run the test target.

Test corpus: `plugins/feature/camera/test/star-tests.csv` (42 cases as of last update;
was 26). Current status: **30 pass / 12 fail.** The latest +5 are Seestar S50 FITS
frames (M97/Pollux/NGC2403/M82/M83) converted to JPG with machine-generated ground
truth via `fits_to_testcases.py` (see below). Run with the env/PATH setup in
`AGENTS.md`; the per-case line includes `stages=…` which now carries a
`verify.faLogOddsMilli` metric (see below).

### nebula-c11 (@mag19) — deep-catalog roll ambiguity (do NOT tune the roll-alias guard for it)
The seed/winner pose (roll 0) is essentially correct — its bright named stars land
within 12–38 px of detections (HIP 115542 within 11.9 px). But at mag 19 a 300° roll
alias coincidentally matches slightly *more* faint stars at *lower* RMS, tripping the
`aliasIsStrictlyBetterFit` guard → rejection. Attempted fix: also require the alias to
match `matchedBrightProjectedStars >= winner` — this **regressed** (reintroduced the
m101@16 false positive *and* didn't fix c11), because at mag 16–19 the bright-projected
set already contains hundreds of mid-bright stars both rolls match, so it doesn't
discriminate the correct roll. Reverted. The correct fix is the magnitude-weighted
verifier (`−log(density)` weights genuinely rare bright stars heavily) — a count/RMS
guard cannot separate the correct roll from faint coincidences in deep fields.

**Verifier-based roll tie-break (implemented).** `hasCompetitiveRollAlias` now, before
its count/RMS logic, compares the winner's and competitor's `poseFalseAlarmLogOdds`
and dismisses the alias if the winner's is clearly higher (margin 5.0). This makes
roll-ambiguity resolution evidence-based: the rare bright stars dominate the log-odds,
so whichever roll lands them on detections wins. It is safe (a faint-coincidence wrong
winner has *lower* log-odds than a correct bright-matching alias, so it never wins the
comparison) and validated (20/31, no regressions; m31/m51 now accept rolls closer to
truth). It does NOT flip m101@15 or c11 — debugging showed their *winners do not match
the bright stars better than the alias*, i.e. the correct roll is not among the search
candidates at all. So those two (and c11 with the new, denser image) are **search
failures** — the correct pose is never found — not disambiguation failures. A
tie-break among found candidates cannot rescue them; that needs search-side work
(bright-anchored roll search in deep fields).

**Search-side diagnosis (conclusive).** The bright-anchored roll search already exists
(`RollConsensus`, ~250 lines: radial roll-invariant pairing of bright catalog stars
with bright detections, brightness/reliability/named-star priority scoring, ~160 roll
candidate evaluations). For m101@15 it *does generate the correct roll* — debugging
shows the winner is roll 30° (102 faint matches) while the correct roll ~90° is exactly
the competitor that triggers the "ambiguous roll" rejection. So the failure is
**winner selection**, not generation: count/RMS/seed scoring keeps the faint-coincidence
roll, and the bright-correct roll then looks like a competing alias and the solve is
rejected. c11 (mag19) is the harder variant — neither candidate matches the bright HIP
stars, i.e. the correct roll may not survive into the final candidate set at all.

**"Adopt the bright-better roll alias" (implemented, safe, currently inert).**
`hasCompetitiveRollAlias` now also handles the *adopt* direction: it scans all roll
offsets, and if a competitor's `poseFalseAlarmLogOdds` exceeds the winner's by ≥15
(`kRollAdoptLogOddsMargin`, i.e. it matches rare bright stars the winner misses), it
hands that alias back (`betterAlias` out-param) and the caller adopts it
(`selectedFinalPass`/`selectedFinalPassForAcceptance` reassigned; the const-refs alias
those members so they reflect the adopted pose; the alias already passed geometry /
direction-seed / brightness-consistency viability inside the function). Validated:
**20/31, no regressions** — the safety property holds (a wrong alias can't
out-bright-match a correct winner, so it never fires on the 20 passing cases).

BUT it does not fire for m101@15 or c11, and debugging shows why: **the correct roll is
filtered out *before* the disambiguation stage.** For m101@15 the ambiguity trigger is
the roll-150° alias, not the correct ~90° — the correct roll has too few matches at
mag15 (relative to the faint-coincidence winner roll-30°) to survive the competitor
viability checks. So the correct pose is lost upstream; a tie-break/adoption among the
surviving candidates can't recover it.

**Real remaining blocker (search-side, upstream) — precise mechanism.** The winner is
chosen by `isBetterEvaluationForMode` (count/RMS/score) at every search stage. In deep
fields the faint-coincidence pose (slightly-off az/el, roll 30°, 102 faint matches)
outranks the correct (az/el, roll ~87°, fewer but bright matches). The correct roll IS
generated but loses every count/RMS comparison; the roll-alias/adoption stages can't
recover it because they only perturb roll around the *winner's* (wrong) az/el, while the
correct pose is a different point in pose-space already discarded.

The fix must inject bright-weighted evidence (`poseFalseAlarmLogOdds`, which weights
rare bright stars) into candidate *selection*. The provably-safe form is an **additive
rescue**: only when a narrow-guided solve would otherwise fail, re-rank a refined
candidate pool by the verifier and try the bright-best one (passing cases never reach
the failure path, so they cannot regress). Implementation requirements, all needed:
- Keep a *refined* candidate pool alive to the failure returns (~line 20255+). Today
  `coarseCandidates` (line 19247) is consumed into `rescoredCandidates` ~line 19562 and
  is neither refined nor live at the failure points.
- Per top-K pool candidate (K~16, by match count): run `refinePoseFromMatches` →
  `evaluateFinalMatchPass` → `poseFalseAlarmLogOdds`. Perf-sensitive (c11 is already
  ~55 s), so cap K and only for narrow guided-direction solves.
- Adopt the best only if its bright-weighted log-odds clears a strong absolute bar AND
  it passes geometry / min-match / direction-seed acceptance, to avoid false positives.
- Expected to fix m101@15 (recover roll ~87°); c11 only if its correct roll survives
  into the pool at all (uncertain — at mag19 it may never be generated competitively).

This is the deepest change in the solver. The verifier disambiguation (dismiss + adopt)
is already in place and will select/adopt the right pose the moment the rescue surfaces
it into the candidate set.

**Verifier calibration (implemented) + bright-anchored roll-sweep rescue (implemented,
safe, partially effective).** `poseFalseAlarmLogOdds` was recalibrated from a clamped
per-match sum into two principled terms:
- **Count-surprise** (Poisson log-likelihood ratio): observing K matches when chance
  expects λ = min(D, C, D·meanDensity·πr²) is strong evidence regardless of per-match
  tightness. This rewards "many matches" so a dense correct solve is not tied by a
  sparse bright-only coincidence.
- **Tightness-weighted bright-rarity bonus**: each match adds
  `log(projectedCount / #stars-at-least-this-bright) · exp(-d²/2σ²)` (σ = r/2), so tight
  matches to rare bright stars dominate.
It now takes a `detectionCount` arg (for λ). Drives the live dismiss/adopt roll
tie-break and the rescue below.

The roll-sweep rescue (`brightAnchoredRollRescue`): on the ambiguous-roll failure, sweep
roll at the seed direction (36 × 10°), rank by the calibrated log-odds, refine the best,
adopt if it beats the rejected pose by ≥15 and has acceptable bright support.

Result: **20/31, no regressions.** The calibration fixed the earlier regression — the
count-surprise term makes m51-2's correct 260-match pose decisively outrank the sparse
26-match coincidence that the *uncalibrated* rescue wrongly adopted (that version was
20→19; this one holds 20/31). But it still does **not** fix m101@15 or c11: at mag 15–19
the count-surprise term also rewards the dense *wrong* rolls (hundreds of faint matches),
and the bright-rarity bonus, though correct in direction, isn't decisive enough to make
the sparser-but-bright correct roll win outright. Forcing it (heavier bright weight)
would risk false positives on the cases the margin protects.

**Magnitude-limited count-surprise (tried) + rescue (reverted) — the conclusive
finding.** The count-surprise was further limited to *bright* matches only (mag ≤ 13)
so dense-faint wrong rolls earn no count-surprise. Still 20/31, still no fix for
m101@15 / c11. Debugging m101@15 was decisive: its wrong winner roll-30° has
`winnerLogOdds 44.6` and *dismisses* its own aliases as weaker — i.e. **the fully
calibrated verifier ranks the wrong roll ABOVE the correct one.** Reason: m101 is a rich
field, so the wrong roll *coincidentally matches bright stars too*, and a sum of
per-match brightness/rarity evidence cannot tell a coincidental bright match from a true
one.

This is the ceiling of the whole verifier approach: it scores *whether* bright stars are
matched, not whether they sit in the right **geometric configuration**. Distinguishing
roll-30° from roll-90° requires the relative arrangement of the bright stars
(quad/triangle invariants — astrometry.net's actual blind-solve method), which a
per-match sum structurally cannot capture. So bright-weighted verification, however
calibrated, is the wrong tool for deep/rich-field roll disambiguation.

**Final state of the verifier work:** kept the calibrated `poseFalseAlarmLogOdds`
(magnitude-limited count-surprise + tightness-weighted bright-rarity) and the *dismiss*
roll tie-break (safe, keeps a correct winner, improves m31/m51 pose quality). **Reverted
the roll-sweep rescue** — proven inert on the corpus and adding a per-failure roll sweep,
because the verifier can't surface the correct deep-field roll for the reason above. The
real fix for the deep/rich-field roll cases (m101@15, c11, narrow-7/8/9, m51@14) is
**geometric quad/triangle-invariant matching on the bright stars**, not more verifier
calibration.

**Geometric matching investigation (conclusive).** That machinery already exists and is
comprehensive: `buildTriangleSignature`/`buildQuadSignature`, signature buckets, ratio
matching, blind AND bright-guided modes (`brightGuidedTriangleMode`), seed evaluation,
and anchor verification. This is not a "build it" task. The precise failure for m101@15
is the seed-verification gate (~line 6919): a geometric seed is accepted only if >=3 of
its 3 star correspondences are spatially consistent (`countProjectedAnchorSupport`,
within ~match radius). Profile: `triangleSeedValid=580`, `triangleSeedMaxAnchors=2`,
`triangleVerifiedSeeds=0` -- of 580 valid geometric seeds, none had all 3 corners
consistent. In a rich field the signatures match a flood of catalog triangles
(`triangleRatioMatches=230499`) and the correct bright-star triangle's seed never
reaches 3 consistent anchors.

The lever is therefore the seed-verification step, which runs on EVERY solve. Relaxing
it (3->2 anchors, wider anchor tolerance) to admit m101@15 would also admit a flood of
2-anchor coincidences in dense fields -- high false-positive risk, likely regresses the
20 passing cases. Must be done with instrumentation, not a blind tune. Concrete first
step: instrument *why* the correct bright-triangle seed reaches only 2 anchors --
(a) never formed (bright detections not brightness-sorted into the first 16),
(b) never matched (signature ratio tolerance / bucketing), or (c) matched but the 3-star
pose places the 3rd corner just outside the anchor tolerance. Each implies a different
targeted fix; tuning the gate without knowing which is the regression trap.

Corpus now 34 cases (user added stars-narrow-7/8/9): **20 pass / 14 fail**; the 3 new
cases are the same known classes (bright-support over-rejection on a populous pose, and
deep-field ambiguous roll).

NB: with the **updated** c11 image (2777 detections, mag19) the bright HIP stars are
matched by *neither* candidate roll (0° vs 295°, both ~1375–1389 faint matches at RMS
~15.4) — a genuine 2-fold ambiguity the search can't break. The user confirmed the CSV
`starPositions` (467,1284)/(581,146) are correct, so the solver's pose is genuinely
wrong; the correct roll simply isn't being found among the faint-coincidence-dominated
candidates.

## Changes made (all validated against the full suite)

1. **Roll-alias false-positive fix** (`hasCompetitiveRollAlias`). A wrong-roll pose
   could be accepted when a soft "seed consistency" score favoured it over a
   geometrically *better* alias. Now: if a roll alias has comparable match count and
   **strictly lower RMS** (`aliasIsStrictlyBetterFit`), the soft dismissals are
   blocked and the ambiguous winner is rejected. Fixed galaxy-m101 @maxMag16
   (was accepting roll ≈ −2.6° when the truth is ≈ +87°).

2. **Acceptance-logic consolidation** (behaviour-preserving). The thicket was largely
   duplicated structure, now factored:
   - `ResidualGates` + `passesResidualGates` + `directionSeedResidualGates` — the
     blind / direction-seed / elevation-seed accept checks and the direction-seed
     reason string share one min-matches→RMS→median→max shape.
   - `sparseGuidedMaxRms` — the match-count→RMS ladder (was duplicated).
   - `calibrationMagnitude` + `isBetterByGeometricTieBreak` — the `isBetter*Evaluation`
     comparators' shared tie-break tail.
   - `kNarrowFieldMaxFovDegrees` + `isNarrowField` — replaced 82 bare `m_fov <= 5.0`.
   - `isNarrowGuidedDirectionSolve`, `usesSeedProjectedBrightGate` — recurring guards.
   - `hasDenseFinalEvidenceOverridingSeedRadial` — the seed-radial override, unified
     across `hasWeakNarrowGuidedBrightSupport` and
     `hasAcceptableGuidedFinalBrightnessConsistency` (the latter was missing it, which
     killed the correct m101@16 pose). Bright-mag cap widened 1.0→1.30 (narrow frames
     routinely contain a saturated bright star with unreliable magnitude).

3. **Match-tightening** (`tightenNarrowFinalPass`). The full match radius (~24 px)
   admits coincidental associations in dense catalogs, biasing the least-squares pose
   and leaving a uniform ~r/2 residual floor (this is matching contamination, **not**
   lens distortion — confirmed: residual is flat vs. radius from image centre, see
   below). After the final pass, the pose is re-matched at a shrinking radius
   (0.5r, 0.33r) and re-fit on the tight inlier core. Adopted **only if** it keeps a
   comparable match count and strictly-not-worse RMS, so it can't destabilise solves
   that are already good. Fixed all four "correct-but-imprecise" cases
   (m51 @15/17/20, m51-2 @16) with zero regressions: +4 passes.

   NB: the distortion-in-the-fit hypothesis was **falsified** — binning the m51@15
   matched residuals by radius from image centre gives a flat ~12–18 px at all radii.
   Radial distortion and plate-scale error both grow with radius; this doesn't. So
   `k1`/`fov` calibration would not help and was not enabled.

## Robust verifier (shadow mode — not wired to the accept decision)

`poseFalseAlarmLogOdds` computes the log-odds that the matched set is a true alignment
vs. a chance coincidence (foreground Gaussian / background catalog-density model; cf.
astrometry.net's verification step). It is the principled replacement for the
hand-tuned bright-support heuristics — bright-star weighting falls out of the density
term automatically. It currently only records `verify.faLogOddsMilli` for analysis.

**Why it is not yet deployed as the accept gate:** on the 26-case corpus a single (or
even two-parameter) threshold cannot reproduce the current decisions without razor-thin
margins. Concretely (post-tightening): the lowest correct PASS (`wide-1`, sparse) has
total log-odds 42 / ~6.0 per match, while a rejected candidate (`m101@15`) has 143 /
1.4 per match, and the ultra-dense correct `m31` has 471 / **0.77** per match (its
catalog is so deep the per-match evidence is crushed). A `(total ≥ T₁) OR
(perMatch ≥ T₂)` rule can fit 18/19 only at `T₁≈150, T₂≈4.6`, where `narrow-3`
(wrong, 4.5/match) is rejected by a **0.1 margin** — overfit to 26 points.

**Deployment criteria (when to swap in the verifier and delete the heuristics):**
- A larger calibration corpus (hundreds of cases). The metric is logged per solve, so
  every future run accumulates data.
- And/or a more sophisticated statistic that handles the dynamic range — most likely a
  proper Poisson count-surprise term plus a magnitude-limited background density (the
  naive deep-catalog density under-credits rich fields like m31).
- Validate in shadow that it reproduces all current accept decisions before deleting
  `hasWeakNarrowGuidedBrightSupport`, `hasAcceptableGuidedFinalBrightnessConsistency`,
  `hasPoorNoRollSeedRadialSupport`, and the bright-support constants.

## Saturated-core detection — bloomed bright stars (`camerastardetector.cpp`)

`stars-narrow-8` (Jabbah, σ Sco, mag ~2.9) failed because the *named test star was
never detected*, even though the pose solved correctly. Diagnosis (instrumented the
contour loop): Jabbah blooms into a large flat-topped saturated region (~21k px² at a
gray>150 threshold), but the **background estimator reads an elevated local background
under the bloom**, so its residual ≈ 0 and it never reaches `thresholdMask`. Near
Jabbah (586,963) the only surviving contour was a 1×1 speck. Consequently:
  - raising `m_starMaxArea` was **inert** (no large contour exists to keep);
  - a contour-based saturated-core extraction was also inert (same reason).

The bloom is only visible in the **gray image** (it still holds the saturated 255
core), not in the residual. Fix: a supplementary pass after the main contour loop
thresholds `gray >= saturationThreshold`, finds each saturated core, and emits a
detection for any core **not already covered** by an existing detection (centroid
within ~core-radius → deduped, so normal bright stars are not double-counted). Cores
smaller than 6 px (saturated hot pixels) or larger than `m_maxContourAreaBound`
(overexposed frames) are skipped. Skipped on the CUDA fast-path where gray was not
downloaded. Result: `narrow-8` detects Jabbah at ~(586,963) and **passes**, detections
1079→1081, **zero regressions** (27/37). The pass is generally useful — any heavily
bloomed bright star the residual path loses is now recovered.

## Speed — distortion-seed dedupe in the fov-pinned recovery grid

Profiling the slowest solves (cluster-m4 116s, narrow-5 73s, m31 44s; total suite
527s) showed the bulk of run time is the guided-triangle search plus two brute-force
recovery grids. The fov-pinned recovery (`useNarrowKnownFovRecovery`) evaluates a
`distortion × az × el × roll` cartesian product of full `evaluateFinalMatchPass`
calls. Its `distortionSeeds` list was `{best.distortionK1, 0.0}` — and
`best.distortionK1` is almost always `0.0` for narrow fields, so the **entire
az/el/roll grid was evaluated twice for identical results**. Deduping the seed list
(safe: `evaluateFinalMatchPass` is a pure function of its inputs, so the dropped
evaluation is bit-identical) cut total suite time **527s → 467s (~11%)** with **zero
change to pass/fail** (27/37). cluster-m4 116→87s, narrow-5 73→48s.

### Parallelised the recovery grids (implemented — identical results)

Both recovery grids are large sets of *independent* `evaluateFinalMatchPass` calls
(fov-pinned: distortion×az×el×roll; roll-recovery: az×el×roll plus an optional
`refinePoseFromMatches` + re-evaluate). They are now evaluated across worker threads
using the same strided-`QThreadPool` + per-worker-`SolverContext` pattern as the
candidate-refinement stage (`copySearchStateFrom`): build the pose list sequentially,
evaluate poses in parallel into per-index slots, then **merge/select sequentially in
index order**. All acceptance/selection logic stays in the serial merge (workers do
only the pure per-pose computation), so the outcome is bit-identical to the serial
loop. A generic `evaluateRecoveryPosesParallel(count, evalFn)` helper drives both grids;
thread count comes from the existing `refinementWorkerThreadCount` heuristic.

Result (combined with the distortion-seed dedupe above): suite **527s → 167s (~3.2×)**
with **zero change to pass/fail** (27/37). cluster-m4 116→11s, narrow-7 89→20s,
m31 44→11s.

### Tried and reverted: fov-pinned skip-guard

Giving the fov-pinned pass the same "selection already strong" skip-guard that
roll-recovery uses (`selectionHasStrongSupport`) **regressed cluster-m4 and m51-2**
(27→25). Those cases reach the fov-pinned pass with a selection that *looks* strong by
named-anchor/dense-guided metrics, yet the fov-pin re-search (re-pinning fov to the
known value and re-fitting) is what actually lands the accepted pose. So the guard is
unsafe for the fov-pinned pass — reverted. (The shared `selectionHasStrongSupport`
predicate is retained as the refactored form of roll-recovery's original three flags,
which is behaviour-identical.)

Remaining speed candidate (not done): the dominant cost is now the guided-triangle
search (~54k candidates on cluster-m4); coarse-to-fine candidate pruning would help but
risks changing which pose is found.

## Remaining failures — all search-stage (`solved=false`), pre-existing

These are upstream of acceptance/refinement: the search does not find/select the
strong correct pose.

| Group | Cases | Diagnosis |
|---|---|---|
| Sparse field | narrow-1, narrow-3 | Only ~4–6 of ~27 projected candidates match → spurious coincidental pose; true dense pose never found. **narrow-1 RESOLVED 2026-06-04 — ground truth was NOT suspect** (the brightest detection, flux 12730, saturated, is Edasich at (513,860) = CSV (514,861)); it was a ~0.31° *elevation* seed-offset case that the recenter could not reach (sparse + elevation). See the seed-offset section below; now PASSES. |
| Catalog-depth instability | m51@14, m51@16, m101@15 | Anti-monotonic (m51 solves at 13/15/17/20, fails 14/16; m101 fails 15, solves 16/20). Search lands on a weak/wrong-roll local optimum at specific depths (m51@16: 83 matches at roll −38° vs correct −2.8°). The correct pose demonstrably exists at adjacent magnitudes → this is a selection/stability problem, the most tractable of the three. |
| Hard blind fisheye | stars-wide-2 (mode 0 Blind, mode 1 Fov) | 165° fisheye, pose never converges. Needs blind triangle/quad indexing. |

Recommended order if/when search work is taken on: catalog-depth stability first
(pose is findable), then sparse-field seeding, then blind fisheye. Validate against a
larger corpus — 26 cases is too few to tune search without overfitting.

### Catalog-depth instability — deep dive (2026-06, no clean fix found)

Instrumented m51@14/15/16 + m101@15 (identical image/seed, only `maxMagnitude`
differs; mode 3 `FovAzEl` so roll is searched). Findings:

- **m51@15 (PASS):** the primary guided-direction search finds roll −1.5°, 150 matches,
  directly (run1 `initial`). No recovery pass runs.
- **m51@16 (FAIL):** the primary search instead locks onto **roll +58° / 90 matches**
  (rms 16.1) and is rejected (`solved=false`). The bright triangle/consensus *seeds*
  are identical to mag15 (the bright catalog is capped at
  `kNarrowGuidedBrightCatalogMaxMagnitude`, independent of `maxMagnitude`), so the
  divergence is **downstream**: with the denser mag16 full catalog a spurious roll-58
  candidate accumulates enough matches to crowd out / outrank the roll-0 candidate
  before/within refinement. roll-0 at mag16 should match ≥150 (it does at mag15), so the
  roll-0 candidate is simply not surviving candidate selection at this depth.
- **m51@14 (FAIL):** different problem — too sparse (catalog 19k stars, only 25 matches);
  no strong pose exists to find. This is a sparse-field case, not depth instability.
- **m101@15 (FAIL):** true roll ≈ **87°** (cf. row 23, which solves at mag20 with roll
  87.2). The primary search finds it at mag16/20 but not mag15.

**Roll-recovery is NOT the lever (ruled out experimentally).** A temporary
`FORCE_ROLL_RECOVERY` hook that runs roll-recovery unconditionally (bypassing the
`selectionHasStrongSupport` skip) changed the full suite by **0 cases** (still 27/37,
just slower). Why it can't rescue these:
  1. roll-recovery sweeps `seed-roll(0) ± {…,30,45}` only, so it **cannot reach m101's
     87°**;
  2. when it does reach m51@16's roll-0 region, its refinement lands at roll 7.75° /
     61 matches (not −1.5° / 150) — **not precise enough** for the named-star pixel
     check (corner stars at ~900 px radius × sin(9°) ≈ 140 px error ≫ 24 px tol);
  3. the **unconditional bright-catalog retry** (`retryDenseNarrowDirectionWithBright
     Catalog()` at the end of `solve()`) then *replaces* a good run1 (96 matches) with a
     worse retry (61 matches) via `solvedBrightRetryHasUsefulAnchorSupport` /
     seed-consistency scoring — i.e. the retry orchestration actively degrades it.

**Conclusion:** this is emergent instability from the interaction of (a) density-
sensitive candidate selection in the primary search, (b) a limited/imprecise recovery
net, and (c) retry orchestration that can prefer a worse solved result. There is **no
single low-risk gate** to flip — the earlier fov-pinned skip-guard attempt regressed 2
cases instantly, confirming how finely balanced the heuristic stack is. The principled
fix is verifier-driven candidate **selection** (magnitude-weighted `poseFalseAlarm
LogOdds`, already logged in shadow mode) so the rare-bright-star-consistent roll-0
candidate outranks the dense-coincidence roll-58 one regardless of depth — but that
needs the larger calibration corpus noted above before it can replace the count/score
heuristics without overfitting. Deferred rather than risk the balanced suite.

## Growing the test corpus from FITS (`fits_to_testcases.py`)

`plugins/feature/camera/test/fits_to_testcases.py` turns a folder of plate-solvable
FITS frames (e.g. Seestar S50 `images/new/*.fit`) into ready-to-paste `star-tests.csv`
rows with **machine-generated, validated ground truth**:

1. reads capture metadata from the FITS header (`DATE-OBS` as UTC, `SITELAT/LONG`,
   `NAXIS`, `XPIXSZ`/`FOCALLEN`, mount `RA/DEC`);
2. renders an asinh-stretched **8-bit RGB PNG** (the harness loads via
   `cv::imread(IMREAD_COLOR)`, so linear 16-bit must be stretched down);
3. plate-solves the FITS with ASTAP, builds an astropy WCS from the `.ini` CD matrix;
4. queries the brightest Hipparcos stars in the field (Vizier `I/239/hip_main`,
   `_RAJ2000/_DEJ2000`), projects them to pixels, and **validates** each lands on a
   near-saturated pixel (sanity check on WCS + orientation);
5. converts ASTAP RA/Dec to the az/el seed (mount-pointing `RA/DEC` for realism, or
   exact via `--seed-from astap`); emits rows with named-star `starPositions`.

**Two hard-won gotchas:**
- **Orientation/parity.** Write the PNG in the FITS array's *native* orientation and
  project with the raw `world_to_pixel` output — do **not** `flipud`. A vertical flip
  mirrors the sky parity; ASTAP still solves it (and projected stars still land on
  bright pixels, so a brightness check alone does *not* catch it) but the SDRangel
  solver is fixed-parity and matches the mirrored frame at the wrong roll with rms ~16.
  Native orientation drops rms to <6 and the cases solve.
- **Named-star choice.** Skip the ultra-bright target (Vmag < 3.5): a saturated mag-1
  star (e.g. Pollux) is detected but the matcher won't cleanly label its bloom, so it
  makes a bad reference. The script falls back to a wider magnitude window only if a
  field has no moderate star.
- Keyring shim (`collections.Callable = collections.abc.Callable`) is required before
  importing astroquery on Python 3.10+.

Validation on the 5 Seestar frames (M97/Pollux/NGC2403/M82/M83), native orientation:
exact seed → **4/5 pass** (rms 0.28–5.2; only Pollux fails, saturated-label gap);
realistic mount seed → **3/5 pass** (NGC 2403 additionally flips to a 180° roll alias
from the ~1° seed error — the same roll-disambiguation weakness). The passes' sub-pixel
rms confirm the generated positions/fov/orientation are correct; the failures are real
solver weaknesses, making these good corpus additions (regression guards + targets).

**Committed:** all 5 added to `star-tests.csv` with the realistic **mount-pointing
seed** and **JPG** images (quality 95; matches the rest of the corpus and avoids
committing the ~12 MB `.fit` source files, which are kept under `images/new/` and
git-ignored). Result: suite 27/37 → **30/42**, no regressions to the existing 27
(new passes m-97/m-82/m-83; new targets pollux + ngc-2403). To regenerate or extend:
`python fits_to_testcases.py --seed-from mount` (drop more FITS in `images/new/`).

## Synthetic test cases (`synthetic_testcases.py`)

A second generator renders **synthetic** star fields with *exact, free* ground truth
into a separate suite (`star-tests-synthetic.csv` + `images-synthetic/`), run on demand:
`featurecamera_star_tests plugins/feature/camera/test/star-tests-synthetic.csv`.

It renders **real catalog stars** (Gaia DR3, so the SDRangel solver can actually match
them) at a *chosen* camera pose. Per scene (a real sky region by RA/Dec + a roll):
az/el is derived at the configured location/time; horizontal roll → equatorial CROTA2
via the parallactic angle; a TAN WCS (same positive-determinant / native-orientation
parity as the real frames) projects Gaia stars to pixels; Gaussian PSFs are drawn with
a magnitude→peak stretch + background/read-noise; bright in-frame Hipparcos stars give
the named-star ground truth (exact, since we place them). `--mags` emits one row per
test magnitude (depth sweeps); `--seed-jitter` perturbs the az/el seed.

Because we place every star, positions are exact by construction — so a solve failure is
unambiguously the *solver's* fault, and the same field can be re-rendered at controlled
depth/roll/density. This is the corpus to grow toward the hundreds of cases the
verifier-driven selection needs.

**Validation / what the default 12-case set shows** (4 scenes × mags 12/14/16):
- `synth-cyg-r0` solves at **rms 0.24 px** at all depths → confirms parity/WCS/positions
  are correct (the generator is sound).
- `synth-cyg-r60` (same field, rolled 60°): **FAIL@12, FAIL@14, PASS@16** — the
  catalog-depth instability reproduced synthetically.
- `synth-cas-r0/r90` (denser, ~4.7k Gaia stars): fail at all depths via the **180° roll
  alias** (solver finds roll ≈2° where ≈180° is correct) — the dense-field roll
  disambiguation weakness. The roll sweep is exact (cyg-r0→solver −179.9°,
  cyg-r60→−119.9°, a clean 60° step), modulo a constant convention offset, which is why
  `expectedRoll` is left blank and the named-star positions carry the orientation.

Same gotchas as the FITS generator (native orientation / positive-det CD; keyring shim).
Dense fields need low-galactic-latitude pointings; high-latitude fields are too sparse
and lack bright HIP stars for named references.

### Generator parity audit (2026-06-04, conclusive): the generator is SOUND — there is no
parity quirk.

The earlier "pointing-dependent projection/parity quirk" suspicion (cyg solves, cas does
not) was **wrong** — same pattern as narrow-1's "suspect ground truth." Worked the geometry:
the solver's solved roll satisfies a single constant convention `solverRoll = sceneRoll −
180°` for *both* fields:

| scene | scene roll | parallactic | solver roll | sceneRoll−180 |
|---|---|---|---|---|
| cyg-r0 | 0 | +42.5 | −179.9 | −180 ✓ |
| cyg-r60 | 60 | +42.5 | −119.9 | −120 ✓ |
| cas-r90 | 90 | −86.0 | **−90.0** | −90 ✓ |
| cas-r0 | 0 | −86.0 | locks 0° (wrong) | −180 (not found) |

**cas-r90 @mag12 solves correctly** (rms 1.9, named stars matched) at exactly the predicted
roll −90 — proving the generator's cas WCS/parity is correct. The cas-r0 / deeper-cas
failures are the solver's **180° roll-alias weakness in dense fields** (depth-dependent: the
0°/180° alias coincidentally out-matches the true roll as the catalog deepens), i.e. the
same class as ngc-2403 — a real solver bug the synthetic corpus correctly exposes, not a
generator defect. So every built-in scene has a proven-correct orientation; the generator is
trustworthy ground truth.

### Random-scene generation (`--random N`) for a tuning corpus

`synthetic_testcases.py --random N` replaces the 4 built-in scenes with N random pointings
at random rolls. Pointings are biased to the Milky Way (`--gal-lat-max`, default 22° |b|,
computed locally — no network), rejected below the horizon (`--min-el`), too sparse
(`--min-stars` rendered), or carrying no in-frame named HIP star — so every emitted case has
usable exact ground truth. `--random-seed` makes it reproducible; the CSV image path now
follows `--out-images` (was hard-coded). Output git-ignored (`star-tests-synthetic*.csv`,
`images-synthetic*/`). Example:
`python synthetic_testcases.py --random 100 --min-el 25 --render-mag 14 --mags 13 \
  --out-csv star-tests-synthetic-rand.csv --out-images images-synthetic-rand`

NB on what random fields reveal: at **zero** seed jitter a large fraction of random
moderate-depth fields still fail — dominated by the 180° roll-alias (random roll exercises
roll disambiguation far harder than the real corpus, which is mostly roll≈0) plus low-el
sparse fields. So for *seed-offset* tuning, filter to scenes that solve cleanly at zero
jitter first (clean baselines), then sweep jitter on those; the roll-alias failures are a
separate (verifier/geometry) workstream.

**First validation of the 100-image corpus (mag 13, zero jitter): 62/100 pass**, all 38
failures `solved=false` (zero false positives — the accept gate is *safe*, just sometimes
too strict). Two failure classes:
- **9 correct fits wrongly rejected** (rand-004/005/056/063/069/076/079/082/083): 55–89
  matches at rms 0.15–0.30 px with the named star matched, rejected for *"weak bright-anchor
  support."* These faint Milky-Way fields have no bright/saturated star (brightest ~mag 7–8)
  and the bright-anchor gates discard an overwhelming faint-star fit.
- **~29 roll-alias / sparse fails** (rms 11–16): no true pose found — the deeper
  roll-disambiguation / verifier workstream (same class as ngc-2403).

**Fix (2026-06-04): `hasOverwhelmingFaintGuidedSupport` accept override.** A narrow
guided-direction final pass with `matches ≥ max(minMatches+20, 30)` at `rms ≤ min(1.5,
0.12·matchRadius)` and bounded max error is a true alignment by overwhelming statistical
evidence (scores of stars cannot land sub-pixel by chance — what `poseFalseAlarmLogOdds`
formalizes), so it bypasses the *brightness* gates (`weakNarrowGuidedBrightSupport`,
`hasAcceptableGuidedFinalBrightnessConsistency`) while still requiring the FoV and
direction-seed *quality* gates. The change is purely additive to acceptance (only adds
bypass paths), so no previously-accepted solve can regress. Validated: **real suite 36/42
unchanged (zero regressions); synthetic 62 → 71/100** (all 9 recovered; Group-B rms-11–16
cases — even rand-027/052 with 40–55 matches — do NOT trigger it, so no false positives).
Also benefits real faint/high-latitude fields the existing corpus doesn't cover.

**The 180° roll-alias IS partly the (pure) catalog-depth instability.** Testing the dense
roll-alias failures across depth (zero seed offset) showed several solve the SAME field/seed
at a shallower catalog: rand-007/027/008 FAIL@13 but PASS@11 and @12 (rand-052/099 fail at
all depths — genuinely hard). So in a rich field a deep catalog's faint stars feed a
coincidental roll-alias that out-counts the true roll; a brighter catalog removes the
coincidences and the true (sub-pixel) pose wins. This is the *pure* depth instability the
original m51@16 framing conflated with seed offset — here it is real, with exact ground
truth and no seed error.

**Fix (2026-06-04): depth-escape retry** (`CameraPlateSolver::solve`, end of the retry
chain). When a dense narrow direction-seeded solve is still unsolved, retry one then two
magnitudes shallower (floor mag 10) and adopt only a *solved* result. Gated to already-failed
dense (`>128` detections) narrow solves, so it cannot regress a passing case; acceptance
still requires a tight well-supported fit, so a wrong shallow pose is rejected → no false
positive. Recovers the depth-dependent dense roll-aliases (rand-007/008/027/045 now pass at
mag 13). The existing `retryDenseNarrowDirectionWithBrightCatalog` only retried *down to* mag
13 and was skipped for maxMag ≤ 13 — this fills that gap. **Real suite 36/42 unchanged;
synthetic 71 → 75/100, zero false positives, zero regressions.**

**Corpus state after the accept-gate + depth-escape fixes: 75/100**, all remaining failures
rms > 10 (no true pose found at any depth).

**Depth-escape density-gate relaxation (2026-06-04): >128 → >64 detections.** Testing the
remaining failures across depth (mag 9–12) showed the depth-alias also strikes *moderate*
fields that the `denseNarrowDirectionSolve` (>128) gate wrongly excluded: rand-015 (117 det),
rand-028 (105), rand-032 (87) all solve at mag 11/12 but not 13. Lowered the depth-escape
gate to a moderate floor (`>64`); still adopts only a *solved* result, so still cannot
regress/false-positive. **Real suite 36/42 unchanged; synthetic 75 → 82/100 (+7: the >64
gate recovered more moderate-density depth-fixable fields than the initial 3-case sample
showed), zero false positives.**

**The "hard core" was NOT a blind-solve problem — it was a GENERATOR PRECESSION ARTIFACT
(2026-06-04, conclusive).** The 18 dense failures *looked* like a blind-solve frontier
(`triangleSeedMaxAnchors=1`, `quadSeedEvaluations=0` — true bright-star configuration never
recovered). But measuring the seed→solved-pose offset across the 81 passing synthetic cases
showed a systematic **mean dAz = −0.36°, dEl ≈ 0** (median angular offset 0.37°). That is
exactly **26 years of J2000→epoch-of-date precession**: the generator computed the camera
seed from the catalogues' J2000 RA/Dec (`_RAJ2000`), while the SDRangel solver works at
epoch-of-date — so *every* synthetic case carried a precession-sized seed offset. Verified
two ways: (1) precessing rand-001's seed J2000→2026 predicts dAz=−0.232/dEl=−0.168, matching
its observed seed→solved offset (−0.23/−0.16) to 0.01°; (2) re-solving the 18 hard cases with
**precession-corrected seeds → 20 of 21 PASS** (rand-022: 107 matches, rand-078: 141; only
the genuinely-sparse rand-066 at 53 detections still fails). The dense fields failed because
the precession offset *plus* their roll-alias exceeded the recenter's reach; with the correct
epoch-of-date seed they solve directly. **The solver was never the problem here — blind-quad
(Stage A) is unnecessary for this class.** Same recurring pattern as the cas-parity and
narrow-1 false alarms: a suspected solver weakness that was a test/data artifact.

**Fix:** `synthetic_testcases.py` now precesses the field centre J2000→epoch-of-date
(`precess_j2000_to_date`) for the seed az/el (rendering/WCS/named-star pixels stay in the
J2000 frame; only the camera-pointing seed needs to be at date). Corrected corpus solves
~99/100 (only sparse rand-066 remains).

**Blind quad-hash solve — NOT needed for the narrow/dense class** (a full scope doc was
written then deleted once the "hard core" proved to be the precession artifact above). The
sorted-edge-ratio `QuadSignature` is collision-prone in dense uniform-brightness fields, and
an astrometry.net-style 4-number geometric-hash code `(xC,yC,xD,yD)` in the A,B frame would
be the distinctive replacement — but the only remaining case that genuinely needs seedless
blind solving is **wide-field fisheye (stars-wide-2, 165°)**. Revisit the quad-hash approach
only if that is prioritised; a prototype `buildQuadHashCode` was implemented and reverted
(the solver file is back to baseline).

## Verifier diagnostic — is it a selection or a search problem? (conclusive)

Instrumented the solver (temporary `STAR_DEBUG_POSE` probe + `STAR_DEBUG_CANDIDATES`
dump, since reverted) to settle whether the catalog-depth/roll instability is fixable by
*selecting* with the magnitude-weighted verifier (`poseFalseAlarmLogOdds`) or needs
*search* work. Test case: m51@16 (fails; the same image solves at mag 13/15/17/20).

Probing the **known-correct pose** (from m51@15's solution: az 93.006, el 72.763,
roll −1.56, fov 1.29) vs the solver's **selected** wrong pose:

| pose | matches | rms | faLogOdds |
|---|---|---|---|
| selected (wrong winner) | 90 | 16.1 | **48** |
| known-correct (probe) | **260** | 12.2 | **173** |

So **the verifier is an excellent, reliable discriminator** — the correct pose scores
~3.6× higher and would clearly win a verifier-ranked selection.

**But** dumping all 289 refined candidates showed the correct pose is **not among them**:
- the closest candidate to the true centre (az 93.29, roll −1.27) is ~120 px off and gets
  only 92 matches / rms 17 / faLogOdds 45 — the search never refines onto the true centre;
- the highest-match candidates (133, 120) sit at **inflated fov (1.55 vs true 1.29)** —
  off-centre local optima that grab matches by spreading the field.

**Conclusion:** this is primarily a **search/refinement convergence** problem, not pure
selection. The verifier alone can't pick a candidate that isn't generated. The fix is
two-part, and both parts want the *same* objective:
1. **Refine toward the verifier optimum** (re-match + refit the top-K candidates to
   maximise `poseFalseAlarmLogOdds`, with **fov pinned to the known value** so the
   inflated-fov local optima are removed) — this pulls the ~120 px-off candidate onto the
   true centre (→260 matches).
2. **Select by the verifier**, not by raw match count / RMS (which inflated fov games).

### Follow-up experiments (conclusive): it's a SEED-GENERATION failure at depth

Pursued the fix with three further shadow experiments (all reverted):

1. **Verifier-guided fov-pinned grid search around the seed** (coarse az/el ±1° step 0.25,
   roll 0–360 step 6; then fine). For m51@16 the coarse best landed at the *seed* az 94.0
   (fa 62), stepping right over the true peak at az 93.006 (fa 173). The true pose is a
   **sharp peak** (~0.016° az / ~1° roll precision); an affordable grid can't sample it and
   a fine-enough grid over the full roll range is infeasible. Grid rescue is the wrong tool
   (this is exactly why plate solvers use geometric hashing, not grid search).

2. **Dumped all 289 refined candidates** for m51@16: the *closest* to the true centre is
   ~160 px away (az 93.29, 92 matches, fa 45); the highest-match candidates sit at inflated
   fov 1.55. There is **no candidate within refinement range of the true pose.**

3. **Coarse-to-fine match-radius refinement** from that closest candidate (match at 2.5×→1×
   radius, fov pinned, re-fit each step): it stays put (az 93.29, rms ~16, fa ~42) — a 2.5×
   radius (60 px) can't bridge the 160 px gap without swamping the fit with contamination
   (246 matches at rms 39). No refinement trick recovers it.

**Conclusion (root cause pinned):** the magnitude-weighted verifier is an excellent
*discriminator* (true 173 vs all candidates ≤76), but the true pose is never produced as a
candidate at mag 16 — so neither verifier-selection, grid rescue, nor radius refinement can
help. The same image *does* seed the true basin at mag 13/15/17/20. Therefore the defect is
in **geometric seed generation / early candidate survival at catalog depth**: at mag 16 the
denser catalog either crowds the true bright-anchored seed out of the candidate budget, or
full-catalog free-fov refinement drifts the good seed into a coincidental optimum before it
is scored. The real fix lives there (depth-robust seed ranking, and/or fov-pinned
verifier-guided refinement applied *at the seed stage*, before drift), not in the
accept/select tail. This is a substantial, higher-risk change to the triangle/quad matching
internals — the next investigation should instrument the seed stage (pre-refinement) to
distinguish "true seed never generated" from "true seed generated then drifted".

### ROOT CAUSE (pinned to one line): the bright-anchor seed degrades with catalog depth

Instrumented the pre-refinement seed pool (`coarseCandidates`/`rescoredCandidates`,
`STAR_DEBUG_SEEDS`, reverted) and flagged seeds near the true pose (az 93.006 / el 72.763
/ roll −1.56):

- **m51@15 (PASSES):** an *anchored* seed at az 93.010 / el 72.749 / roll −1.348 — ~3 px
  from truth → refines straight to the solution.
- **m51@16 (FAILS):** the nearest anchored seed is az 93.293 / el 72.823 / roll −1.067 —
  **~151 px** from truth → refinement can't bridge it (confirmed earlier).

So a near-true anchored seed *is* generated at both depths, but its **accuracy collapses
from ~3 px to ~151 px** as the catalog deepens. The cause is structural:
`searchGuidedAnchorPose` runs on a catalog **rebuilt to `fullSearchMaxMagnitude`**
(cameraplatesolver.cpp ~19390), and its `findGuidedAnchorPairs` correspondence is chosen
over that full, depth-dependent `localVisibleStars` set. At greater depth the denser star
field admits a slightly different/wrong bright-star correspondence, shifting the seed pose
~150 px — beyond the refinement basin.

**This is the whole instability** (m51@14/16, m101@15, ngc-2403, synthetic cas): the
verifier discriminates the true pose perfectly, but it never becomes a candidate because
the anchor seed that should hit it is thrown off by catalog depth.

**Fix (next implementation, scoped but core-path so do carefully):** make the bright-anchor
*correspondence and pose* depth-invariant — compute the anchor pairs/pose from a **capped
bright catalog** (e.g. the same `kNarrowGuidedBrightCatalogMaxMagnitude` cap the triangle
seeds already use), independent of `m_plateSolveMaxMagnitude`, then verify/refine against
the full catalog. The brightest stars don't change with depth, so the seed would stay ~3 px
at every magnitude. Validate against the full real suite (30/42, no regressions) and the
synthetic depth sweep. Risk: this touches `searchGuidedAnchorPose`/`findGuidedAnchorPairs`,
used by every direction-seeded solve — needs incremental, well-validated changes.

**Attempt 1 (INERT — reverted).** Capped `allowedCatalogIndices` (the catalog the anchor
poses are scored/refined against in `searchGuidedAnchorPose`) to
`kNarrowGuidedBrightCatalogMaxMagnitude` for `useFaintNarrowAnchors`. **No effect**: the
m51@16 anchor seed stayed at az 93.293 (151 px), matchCount unchanged. Findings that update
the model:
  - `findGuidedAnchorPairs` already caps anchor stars at `min(maxMag, 10)` → the anchor
    *pairs* are identical at mag 15/16.
  - `selectLocalVisibleStars` keeps the **brightest** 2048 (sorts by magnitude), so bright
    anchors are never dropped by the proximity cut — `localVisibleStars` is the same bright
    set at both depths.
  - So the residual depth-dependence is **deeper**, inside `evaluateAnchoredPose` /
    `refineGuidedAnchorSeedWithLm` (both take the full `catalogContext`) and/or
    `guidedAnchorSearchScore`'s ranking of which anchor pose is "best". And separately, even
    the near-true anchor seed loses final selection to the roll-58 **triangle** alias.
  - Net: the fix is **multi-part** (depth-invariant anchored-pose LM/scoring **and**
    verifier-based candidate selection), not the single catalog swap first assumed. Next
    diagnostic: trace `catalogContext` use inside `evaluateAnchoredPose`/
    `refineGuidedAnchorSeedWithLm` to find why the *same* anchor pair refines to 3 px at
    mag 15 but 151 px at mag 16.

Note (SUPERSEDED 2026-06-04): the "pointing-dependent parity quirk" suspicion (cyg solves,
cas does not) was investigated and **disproven** — the generator is sound (cas-r90 solves
correctly at the predicted roll, so its parity is right; the cas-r0/deep-cas failures are
the solver's 180° roll-alias weakness). See the "Generator parity audit" subsection above.

### REFRAMING (2026-06-04, conclusive): it is SEED-OFFSET sensitivity, NOT catalog depth

All of the prior "depth" analysis above is on the wrong axis. Controlled experiments on the
*current* code (each: identical image, identical maxMag pair, the ONLY change is the seed
azimuth/elevation) settle it:

| case | seed az/el | maxMag 15 | maxMag 16 |
|---|---|---|---|
| m51-1 | az 93.0 (on truth) | PASS | **PASS** |
| m51-1 | az 94.0 (~1° off)  | **FAIL** (roll 58°) | FAIL (roll 58°) |
| m101-1 (mode 3) | az 291.93 / el 70.69 (on truth) | PASS | PASS |

So **at the correct seed every depth solves; at a ~1° offset seed every depth fails.** The
failing `star-tests.csv` rows are confounded: they pair the "failing" magnitudes with
offset seeds (m51 mag14→az92, mag16→az94, while the passing mag13/15/17/20 all use az93;
m101 mag15→az292.35 which is 0.42° off, mag16→az292.0 which is 0.07° off). The
anti-monotonic "13/15/17/20 pass, 14/16 fail" pattern is therefore an artifact of which
seed each row carries — **not** a depth phenomenon. ngc-2403 was already flagged as a "~1°
seed error" 180°-roll-alias case, so it fits the same mechanism.

**Mechanism (m51-1 @16, seed az94, `STAR_DEBUG_ANCHOR_SEEDS` dump, reverted):** the
near-true bright-anchor seed *is* generated — `searchGuidedAnchorPose` anchor#31 produces
az 93.29 / el 72.82 / roll −1.07 (truth az 93.006 / roll −1.56). `anchorAlignedPoseFromPixel`
pins the anchor star exactly (anchorD=0), so the ~150 px residual comes from the roll-seed
grid granularity: the nearest roll seed is ~0.5° off true roll, and roll about the pinned
anchor shifts the field center ~150 px. At that 150 px offset the wide anchor match radius
(up to 128 px, 4× multiplier) fills the LM's `fixedMatches` with *coincidental* catalog
associations (rms ~40, m=7), so `refineGuidedAnchorSeedWithLm` has no gradient toward truth
and returns the seed unchanged (PRE==POST). Meanwhile the denser roll-58° alias accumulates
more matches and wins selection. The magnitude-weighted verifier still discriminates the
true pose perfectly, but it never becomes a candidate within the refinement basin.

This *unifies* the earlier "true pose never generated as a candidate" finding with the
real cause: the seed offset (not depth) is what throws the anchor seed ~150 px off and
keeps the true basin out of the candidate pool.

**Fix direction (supersedes the depth-invariant-anchor-catalog plan above):**
seed-offset robustness. The additive az/el seed-jitter the solver needs **already exists**
as the "recenter" retry in `CameraPlateSolver::solve` (it jitters az/el by ±0.75·fov and
±fov, re-runs the whole solve with a jittered `settings` copy, and keeps the best — a
−0.9675° az jitter on the m51 @16/az94 case lands at az≈93.03 and solves at 257 matches).

**Implemented fix part 1 (2026-06-04): removed the `skipRecenterForStrongRejectedAnchor`
guard.** The recenter was being suppressed by `hasStrongRejectedNarrowAnchorCandidate`,
which mis-classified the roll-58° alias as "strong" (it coincidentally matched 3 named
bright anchors at low RMS — `[RECENTER] solved=0 matched=90 skip=1 namedAnchors=3
namedRms=12.48`). That guard blocked the jitter that finds the true pose. The recenter loop
already keeps the current result as its floor and only adopts a clearly-better *solved*
pose, so always attempting it is safe. The guard was introduced inside a large rework
(commit a262f98c0), not as a targeted regression fix. Removing it (and the now-unused
predicate): **m51 @16 now PASSES** (roll −1.50, 257 matches), suite 30/42 → 31/42.

**Implemented fix part 2 (2026-06-04): finer recenter az tier.** The recenter offset grid
was ±0.75·fov / ±fov (±0.97° / ±1.29°), which *overshoots* a sub-fov seed error. m101 @15 is
only ~0.42° (0.33-fov) off, so the nearest jitter point (0.55° away) is outside the
roll-87° basin — instrumentation showed the recenter finding az≈291.82 but roll≈0 instead
of 87. Added a finer az tier (±0.33·fov, ±0.5·fov) *after* the existing coarse offsets (so
cases that already resolve early-stop unchanged) and raised `maxRecenterAttempts` 4 → 8.
m101 @15 now solves at the −0.33·fov attempt (seed az 291.92 → roll 86.0, 145 matches).
- **m101 @15 now PASSES**, and **nebula-c11 (mag19) now PASSES too** (an unexpected bonus —
  the finer tier lands its deep-field true pose at a sub-fov offset).
- Full real suite **31/42 → 33/42, zero regressions**. Remaining 9 fails: narrow-1/3/4/7,
  wide-2 (×2 modes), m51 @14, pollux, ngc-2403 — all pre-existing.

**Implemented fix part 3 (2026-06-04): pollux — test-harness name reconciliation (NOT a
solver bug).** Investigation showed pollux already *solves cleanly* (rms 1.38, 470 matches)
and the bright star IS detected, solved, and labelled — the saturated-core recovery pass
(added for narrow-8) emits a detection at (568,961), 6.7 px from the CSV's (562,958), with
`solved=1 label='Pollux'`. The earlier "won't cleanly label its bloom" characterization is
now outdated. The only failure was the test's name-based `missingExpectedStars`: the solver
labels the star by its proper name **"Pollux"** while the CSV ground-truth names it
**"HIP 37826"**, and the HYG diagnostic catalogue (`hyg_v42_reduced.txt`, `name|ra|dec|mag`)
has no HIP cross-reference, so neither name-match nor projection could bridge them. Fix
(`camerastartests.cpp` only): when an expected star carries an explicit pixel position, a
solved detection within tolerance of that position counts as "found" regardless of the
solver's synonym label (`expectedStarPositionSatisfied`). The position check is already the
authoritative assertion (`expectedStarPositionMismatches`); this stops the redundant
name check from failing on synonyms. **pollux now PASSES; suite 33/42 → 34/42, zero
regressions.** General: fixes any future proper-name vs catalogue-id divergence.

**Implemented fix part 4 (2026-06-04): elevation + sparse-field recenter (narrow-1,
narrow-4).** Two discoveries: (a) the recenter's elevation offsets sat *past* the attempt
budget (`maxRecenterAttempts` reached only the azimuth offsets), so the recenter was
**azimuth-only in practice** — it could never recover an elevation seed error; (b) the
recenter was gated to `denseNarrowDirectionSolve` (>128 detections), excluding sparse
fields. stars-narrow-1 is both: 29 detections and a ~0.31° (0.24-fov) *low elevation* seed
error. Its ground truth was verified CORRECT (brightest detection = Edasich at the CSV
position); at the corrected seed it solves at 25 matches / rms 0.30. Fix: gate the recenter
on `narrowDirectionRecenterEligible` (narrow + direction + !roll, any star count), add an
elevation offset tier (±0.33/±0.5/±0.75-fov) mirroring the azimuth tier, and raise the
budget 8 → 14 so the elevation tier is actually reached. Cases that resolve on an azimuth
offset still early-stop before the elevation tier, so m51 @16 / m101 @15 / c11 are
unchanged. **narrow-1 AND narrow-4 now PASS; suite 34/42 → 36/42, zero regressions.**
Tradeoff: an unsolved narrow direction-seed solve now runs up to 14 recenter solves (cost
paid only when the user seed is outside the basin; an accurate seed solves immediately and
never recenters). A quality-based early-stop for sparse solves (stop once a tight low-RMS
pose is found, not only at ≥80 matches) would trim this but risks early-stopping on a
coincidental tight pose in dense fields — deferred.

**Still failing — deferred, NOT seed problems:**
- **ngc-2403**: the recenter now *does* reach the true-roll region (two different recentered
  seeds both converge to az 317.71 / roll ≈4.2 with ~190 matches), but **acceptance rejects
  it** (rms ≈16.4, `solved=0`). This is an accept-gate problem, not a seed/recenter one, and
  carries higher regression risk; the notes already flag ngc-2403 as a synthetic-parity-
  adjacent weakness. Needs accept-gate work, not more jitter.
- **m51 @14**: genuinely sparse (catalog ~19k, only ~25 matches) — no strong pose exists;
  sparse-field problem, not seed offset.
- **narrow-3** (sparse), **narrow-7**, **wide-2** (165° fisheye, blind solve): pre-existing,
  unrelated classes. narrow-3 may be a larger seed offset or genuinely too sparse — worth a
  ground-truth + seed-offset check like narrow-1 before assuming it's a solver limit.

## Structural note (not started)

`CameraPlateSolver::SolverContext` is one ~18.5k-line inline class body (lines ~63 to
~18640) — the entire solver in a single translation unit because the class is declared
in the `.cpp`. Splitting it (private header + themed TUs: catalog I/O, projection,
search, acceptance, Siril network, orchestration) is a behaviour-preserving structural
task, deferred.
