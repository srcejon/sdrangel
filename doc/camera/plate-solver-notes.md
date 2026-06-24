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

## ngc-2403 — match-radius deep dive (2026-06-08, conclusive: no viable fix exists)

Follow-up to the "still failing — accept-gate problem" note above: investigated whether
the `m_plateSolveMatchRadius`/`m_plateSolveFinalMatchRadius` (default 24 px) is the real
lever, since ngc-2403 is unusually dense (1178 detections, ~400 catalog stars projected)
and the rejected pose's RMS (~16.4 px) sits suspiciously close to the radius itself —
the classic signature of **matching contamination**: a radius loose enough that ~20
wrong-pose candidates accidentally accumulate match counts (170–230) and RMS (15–17 px)
similar to the true pose, so the true pose never wins selection (and, per the discussion
above, isn't even reliably generated as a refined candidate).

**Five attempts, all instrumented/validated against the full 42-case suite:**

| # | Approach | Scope | ngc-2403 result |
|---|---|---|---|
| 1 | Baseline (24 px) | — | FAIL: matched=217, rms=16.8, solved=false, degenerate pose |
| 2 | Global 24→8 px (blunt sanity check) | search + seed + refine | **PASS**: matched=380, rms=0.24, solved=true, roll −32.83° (= ground truth) |
| 3 | `candidateDiscriminationAffinity` — separate, *tighter* scoring/discrimination radius layered on top of the existing match radius (only re-ranks already-generated candidates) | refinement/selection only | FAIL — proves discrimination/re-ranking isn't the bottleneck: the correct pose is never *in* the candidate pool to rank |
| 4 | Progressive radius annealing (2 variants — ICP/EM-style alternating fit/associate while geometrically shrinking the inlier radius from 24px toward a target) | refinement only | FAIL, **and regressed**: both variants converge to a *new*, never-before-seen wrong basin (roll ≈ +64°, rms still ~16) and — worse — the harness reports `solved=true` there: a false positive with no ground truth to catch it |
| 5 | Global 24→8 px **production default**, validated end-to-end (harness genuinely pinned to 8 px, not just `camerasettings.cpp`, so the search/seed stage itself is exercised) + full 42-case regression | search + seed + refine | **PASS**, identical to #2: matched=380, rms=0.237, solved=true, roll −32.8327° |

**Why #3/#4 (refinement-side fixes) cannot work — basin geometry, proven not assumed.**
The correct pose (roll ≈ −32.8°, 380 matches @ 0.24 px) occupies a *separate optimization
basin* that the loose-radius (24 px) search/seed stage never lands near. Any tight-radius
refinement seeded from a loose-radius local optimum gets trapped in a different
nearby-but-still-wrong basin — proven empirically: both annealing variants (which differ
in their walk strategy) independently converge to the *same* never-before-seen wrong
basin (roll ≈ +64°), never the correct one. ICP-style annealing can refine *within* a
basin but structurally cannot *cross* basin boundaries — that requires the search/seed
stage itself to operate at a tighter radius so it lands near the true basin in the first
place. Hence only a change that affects search/seeding (not just refinement) can work —
which is exactly why the blunt global-8px experiment (#2) and the real default change (#5)
both succeed identically while #3/#4 fail identically.

**Why the global fix (#2/#5) cannot be deployed — full-suite regression, severe.**
Validated #5 the *correct* way: the test harness hardcodes
`settings.m_plateSolveMatchRadius/FinalMatchRadius = 24.0` directly on the per-test
settings object (`camerastartests.cpp` ~line 911), **completely overriding**
`camerasettings.cpp`'s defaults — so simply changing the production default is invisible
to the automated suite. (This is the same hardcoding that the original sanity check in #2
had to work around.) Temporarily re-pinning the harness to 8 px, rebuilding, and running
the full 42-case suite gives:

| harness match radius | suite PASS | suite FAIL |
|---|---|---|
| 24 px (clean-revert baseline) | **36** | **6** |
| 8 px (global reduction) | **19** | **23** |

Net **−17 passing tests for +1 fixed**. Eleven distinct image types that pass cleanly at
24 px newly fail at 8 px — `stars-narrow-4/5/6`, `stars-wide-1` (matched count drops to
**0**), `stars-wide-3`, `galaxy-m101-1`, `galaxy-m51-2`, `galaxy-c7-1`, `cluster-m3-1`,
`nebula-c11-1`, `pollux` — i.e. essentially every sparser/wider/less-dense field class in
the corpus. The 24 px default exists precisely to give the search/seed stage enough
latitude to find correct matches when fields are sparser or geometry less precise; ngc-2403
sits at the *opposite* extreme (unusually dense), so the single global constant cannot
serve both regimes. This is exactly the "under-matching regression on sparse/wide fields"
risk anticipated before running the experiment — it proved far more severe (a >2× swing in
the failure count) than a marginal edge case.

**Final disposition: reverted to the original 24 px default; ngc-2403 remains a known,
narrow, accepted limitation** of unusually dense star fields (where ~20 wrong-pose
candidates coincidentally accumulate match counts/RMS similar to the true pose at the
default radius, and the true pose's basin is never reached by the search stage). All
experimental code (`candidateDiscriminationAffinity`, `DiscriminationStats`, both
annealing variants) was fully reverted from `cameraplatesolver.cpp` (verified clean by
grep — zero remnants); `camerasettings.cpp` keeps its 24 px defaults with a comment
summarising this investigation so it isn't blindly repeated; `camerastartests.cpp`'s
temporary 8 px pin was cleanly reverted to 24 px.

**What a real fix would require:** not a single global constant but a
**density-/field-adaptive match radius** — tighter when detection density is unusually
high (à la ngc-2403), looser when sparse — selected per-solve from local field statistics
before the search stage commits to a radius. That is a substantially larger, higher-risk
change (new tuning parameters, its own full regression cycle, risk of destabilising the
already-finely-balanced 36/42 baseline) and was explicitly deferred rather than rushed.
It is the natural complement to the verifier-driven candidate-selection work already noted
above (`poseFalseAlarmLogOdds`) — both are instances of "replace a single hand-tuned global
constant with a statistic computed from the actual field," and both await the larger
calibration corpus needed to do so without overfitting.

## Fresh failure triage (2026-06-08) — narrow-3 & narrow-7 are search-side, ground truth verified

Re-ran the full 42-case suite on a clean build: **36 pass / 6 fail** (matches the prior
baseline). The 6 fails: narrow-3, narrow-7, wide-2 (×2 modes), m51@14, ngc-2403.

Diagnosed the two "unclassified-tractable" candidates by isolating each case
(`narrow3.csv`/`narrow7.csv`), running with `SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE=1`
(plus `QT_LOGGING_RULES=*.debug=true` to surface the **unconditional** rejection-reason
`qDebug` at cameraplatesolver.cpp:20714 — the key accept-gate diagnostic), and verifying
ground truth against astropy (HIP → az/el at the capture time/location):

- **narrow-3** (HIP 73199, Vmag 4.63): true az/el = **32.60 / 70.41**; seed = 32.0 / 70.0,
  i.e. only ~0.7° off (~0.5·fov). The recenter **does** reach it (run5/9/14 land at
  az≈32.55/el≈70.34, within 0.07° of truth) — yet only **4–7 of 45 detections match** there
  (catalog mag-12 projects only ~36 candidates into the FOV). So narrow-3 is **NOT** a
  seed-offset win like narrow-1; it is a sparse-field + roll search failure: at the correct
  centre the matchable set is tiny and the roll isn't pinned. Genuinely hard.

- **narrow-7** (HIP 11505/11254/11118, Vmag 7.78/8.80/8.85, dense Milky-Way field az 342
  el 27): seed only ~0.15° off. Ground truth **verified correct** — the brightest named
  star HIP 11505 is **detected at (150.4, 482.4)** (flux 5375, *saturated*, the 2nd-brightest
  detection), exactly the CSV position (151,482). The solver finds populous poses (run10:
  **196 matches, rms 11.07**, roll −5.08) that *look* strong, but every one is rejected with
  `brightProjected=0/12 seedBright=0/13 namedAnchors=0` — **zero bright catalog stars and the
  named anchor never match**; the 196 are faint coincidences at the ~r/2 (rms≈11) contamination
  floor. The accept gate is doing its job (rejecting contaminated faint-coincidence poses); the
  defect is that the **true bright-aligned roll is never generated/selected**. Same deep-field
  ambiguous-roll / bright-anchored-seed class as the notes' ROOT CAUSE section.

**Conclusion of the triage:** there is no small, safe accept-gate tweak left. The accept gate
correctly rejects the contaminated poses it sees; relaxing it (e.g. accepting a high
detection-match-fraction pose) would risk false positives on exactly the cases the margin
protects (ngc-2403's 217-match/rms-16.8 wrong pose; m51@14's coincidence). All 6 remaining
failures reduce to three large, deferred workstreams, in priority order:
  1. **Bright-anchored, depth/roll-robust seed generation + verifier-driven candidate
     selection** (`poseFalseAlarmLogOdds`, already shadow-logged and proven a perfect
     discriminator). Targets narrow-3, narrow-7, m51@14-class, and the synthetic roll-aliases.
     The true pose scores far higher on the verifier but is never produced as a candidate, so
     the fix lives in seed generation/early-candidate survival, not the accept/select tail.
     Needs the hundreds-case synthetic corpus to retune without overfitting the 36/42 balance.
  2. **Density-/field-adaptive match radius** (ngc-2403): tighter radius where detection
     density is unusually high, looser when sparse, chosen per-solve from field statistics
     before the search commits. The single global 24-px constant cannot serve both regimes
     (8 px globally = 19/42). Same corpus dependency.
  3. **Blind quad-hash indexing** (wide-2, 165° fisheye): seedless astrometry.net-style
     geometric hash; only this class genuinely needs it.

m51@14 is the odd one out: catalog ~19k at mag14, only ~25 matchable → may have no strong
pose at all (sparse-data limit, not a solver bug).

## Implementation spec — bright-detection-anchored verifier rescue (track chosen 2026-06-08)

Concrete, code-grounded scoping for the chosen "verifier-driven seeding" track, written
after tracing the failure path. **Key new insight from the narrow-7 stage dump:** every
geometric seed fails the ≥3-consistent-anchor verification gate (`triangleVerifiedSeeds=0`,
`quadVerifiedSeeds=0`, `guidedAnchorTriangleVerifiedSeeds=0`) — so in a dense field the true
bright-aligned pose is *generated but never survives*. The rescue must therefore **generate
candidates that bypass the seed-verification gate** and rank them by the verifier, which is
different from every prior reverted attempt (those swept roll at the seed direction without
pinning a detection, or only re-ranked the already-gated pool).

**Why it's safe by construction:** run it only when a narrow guided solve is *already going
to be rejected*, and adopt its result via the existing `rollAdoptedAlias` mid-flow swap
(cameraplatesolver.cpp ~20624) so the *unchanged* acceptance gate (~20679) re-validates it.
A passing case never enters the rescue (it's accepted first); a wrong rescue pose is rejected
by the same gate that rejects today → cannot regress passing cases, cannot false-positive.

**Algorithm (inside `SolverContext::solve`, insert at ~20596, after the geometric-consistency
check, before the roll-alias check):**
1. Gate: `useStartDirection && isNarrowField(settings) && !isCancellationRequested()` and the
   current `selectedFinalPass` is *not acceptable* — factor the 20679–20701 acceptance into a
   lambda `directionAcceptanceFor(const FinalMatchPassEvaluation&)` and require it false for the
   current pose. (Avoids running the sweep on cases that already pass; preserves no-regression.)
2. Bright detections: sort `starDetections` by flux desc, take top K≈4, prefer `m_saturated`,
   skip `m_hotPixelSuspect`.
3. Bright catalog set: `selectLocalVisibleStars(catalogContext.visibleStars, …)` (5340) capped
   to mag ≤ `kNarrowGuidedBrightCatalogMaxMagnitude` (12), take brightest N≈10; use `.vector`.
4. For each (detection, catalog-star) × roll ∈ {0,10,…,350}: build a `GuidedAnchorPair`,
   `evaluateAnchoredPose` (3810-style alignment is inside it) → `refineGuidedAnchorSeedWithLm`
   (returns a refit `Evaluation`, not just the pinned seed — necessary, evaluateFinalMatchPass
   alone does not refit). Reuse the exact block at 16346–16396.
5. Convert each refined `Evaluation` → `evaluateFinalMatchPass(its pose)` → `poseFalseAlarmLogOdds`
   (12519, the proven discriminator). Keep the best by log-odds.
6. Adopt only if the best clears a *strong absolute* log-odds bar AND `directionAcceptanceFor`
   is true for it; then `selectedFinalPass = selectedFinalPassForAcceptance = rescuePass`
   (the rollAdoptedAlias pattern) and fall through to normal result population.

**Perf:** K·N·36 ≈ 1440 anchored evaluations; cap K/N and reuse `evaluateRecoveryPosesParallel`
(the existing strided-QThreadPool helper used by the recovery grids). Runs only on the
failure path, so passing cases pay nothing.

**Validation plan (do NOT skip — the suite is finely balanced):** (a) full 42-case real suite
must stay ≥36/42 with zero regressions and zero new false positives; (b) regenerate the
precession-corrected synthetic random corpus (`synthetic_testcases.py --random 100 --seed-from
mount`, ~99/100 baseline) and confirm no regressions there too. Expected wins: narrow-7
(brightest detection is a clean saturated anchor at (150,482)); possibly narrow-3 (sparse, may
still lack enough matchable stars even at the true pose). If it proves inert on narrow-7 after
generating the true-roll candidate, the bottleneck is the verifier *absolute bar* vs. the
contaminated pool — instrument the best rescue candidate's log-odds vs. the contaminated
winner's before tuning the bar.

## Depth-escape false-positive fix + deepen-on-sparse experiment (2026-06-11)

Following the harness re-verification on 2026-06-11 (suite still 36/42 after fixing two
breaks from commit baacdbf75, see [[camera-star-tests-run-procedure]]), m51@14 was found to
be a **false positive**: the depth-escape retry (`CameraPlateSolver::solve`, ~21806) adopted
*any* `escapeResult.m_solved == true`, regardless of fit quality. At mag 13 it produced a
wrong pose (10 matches, rms 14.26px, az 92.145 vs truth ~93.0, roll +4.64 vs truth ~-1.5..-2.8°)
that the gate had no chance to re-check, since depth-escape bypasses
`directionSeedAcceptanceFor` entirely.

**Fix (landed):** added a tight-fit gate to both depth-escape retry attempts —
`escapeResult.m_solved && escapeResult.m_rmsErrorPixels <= 3.0px` — chosen because the
motivating synthetic wins (rand-007/008/027/045) all solve sub-pixel with ~100+ matches,
cleanly separated from m51@14's rms-14.26 false positive. A solved-but-loose escape result is
now logged and rejected, falling through to the (correct) `solved=false` outcome.
**Result: suite stays 36/42, zero regressions; m51@14 is now an honest `solved=false`
(matched=33) instead of a wrong-pose `solved=true`.**

**Deepen-on-sparse (tried, reverted):** symmetric idea for narrow-3 — when a sparse
(20–64 detection) narrow direction-seeded solve fails, retry one/two magnitudes *deeper*
(more catalog stars), adopting only if solved with rms ≤ 3px. Implemented identically to
depth-escape (additive, failure-path-only, tight-fit gated — provably can't regress or
false-positive). **Empirical result: inert.** At mag 13/14 narrow-3's match count barely
moved (6 → 7, rms ~11 → ~10.8, still unsolved) — confirms the existing diagnosis that
narrow-3's bottleneck is the sparse-field roll search itself, not catalog coverage at the
true centre. Reverted to avoid the extra ~6s/case runtime cost on an already-failing case
with no payoff.

**Status of the remaining 6 (unchanged from the 2026-06-08/11 triage):** narrow-3, narrow-7,
wide-2 ×2, ngc-2403 still require the three large deferred workstreams above (verifier-driven
seed generation / bright-support gate recalibration, density-adaptive match radius, blind
quad-hash). m51@14 remains an honest non-solve (genuinely sparse at mag 14, ~25 matchable).
For narrow-7 specifically: the bright-anchor rescue *does* find the correct pose
(Az≈346°/El≈27°/Roll≈-118..-128°, faLogOdds 22-107, geomConsistent/fovAccepted=true) but
`hasWeakNarrowGuidedBrightSupport` rejects it — the same gate that (correctly) rejects this
image's other contaminated candidates. Fixing narrow-7 means recalibrating that gate, which
risks the 36/42 balance and needs full real-suite + synthetic-corpus validation before
attempting (not done in this pass).

## Narrow-7 gate instrumentation (2026-06-11)

Added per-branch debug logging to `hasWeakNarrowGuidedBrightSupport`
(cameraplatesolver.cpp ~12700-12900), gated behind the existing
`SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE` env var (no-op by default — every `return`
site now has a preceding `logBrightSupportDecision(weak, "<reason>")` call that only
`qDebug()`s when the flag is set). **Suite re-verified at 36/42, zero regressions** with
the instrumentation in place (default env, no debug flags).

Ran the instrumented build on an isolated narrow-7 CSV with
`SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE=1` + `QT_FORCE_STDERR_LOGGING=1` +
`QT_LOGGING_RULES=*.debug=true;*.warning=true`. **Finding: the high-faLogOdds rescue
candidate (Az≈346.17°/El≈27.12°/Roll≈-128.1..-128.4°, 196 matches, rms=11.07,
2 named anchors incl. the verified HIP 11505 at d=9.31px, namedRms=11.47) is rejected
by the `poorNoRollSeedRadialSupport` branch specifically** — `prioritySeedRadialErrorPixels`
≈ 4046-4940px against a ≈96px threshold, with `seedRadialMagnitudeMatchFraction` = 0.000
(0/1004 and 0/373 across the two near-duplicate poses logged).

**What this check actually measures** (cameraplatesolver.cpp ~14185-14342): it builds a
*fixed reference projector* from `m_directionSeedReferenceAzimuthDegrees/...Elevation/
...Roll/...Fov`, which are set (line ~19614-19618) from `settings.m_azimuth/m_elevation/
m_roll/m_fov` — i.e. the camera's recorded/expected pointing (Az=342°/El=27°/Roll=0°/
FoV=1.29° for narrow-7), independent of any candidate pose. For each catalog star the
*candidate* matched to a detection, it compares the star's radius-from-centre under this
fixed seed projector to the detection's actual radius-from-centre; a candidate whose
matched stars are nowhere near where the seed-pointing would put them gets a huge
`prioritySeedRadialErrorPixels`. Because radius-from-centre is roll-invariant, this is
effectively an **"is this candidate's Az/El close to the recorded camera pointing?"**
check, independent of the candidate's roll.

**Why it fires for narrow-7's best candidate:** the candidate's Az≈346.17° is ~4.2° from
the recorded Az=342° — over 3x the 1.29° FOV — so under the seed projector the candidate's
matched catalog stars (including HIP 11505/HIP 11318) project to radii of thousands of
pixels (effectively "off-frame" relative to the seed's narrow FOV), while their actual
detection radii are within the few-hundred-px image. This is the *same* mechanism that
correctly rejects this image's other contaminated 100+-match candidates (all clustered
near Az≈341-346°/Roll spanning -128°..+2°, all `poorNoRollSeedRadialSupport`), so the gate
is behaving consistently — it isn't a one-off special case for the "good" candidate.

**Open question (not resolved, no further action taken this pass):** is the recorded
Az=342°/El=27° in `star-tests.csv` for narrow-7 itself off by ~4° (making this check's
reference wrong and the 196-match/2-named-anchor candidate at Az≈346° actually correct),
or is the 196-match candidate a coincidentally-aligned near-miss (2 bright stars landing
within an 11px tolerance by chance while the overall pose is ~4° off)? Resolving this
needs an independent check of HIP 11505's and HIP 11318's true catalog Az/El against the
image's recorded pointing — out of scope for this instrumentation pass. **narrow-7 remains
unsolved (6/42 unchanged)**; the instrumentation is retained (it's a no-op by default) for
use in that follow-up.

### Open question resolved + rescue shortlist instrumentation (2026-06-11, follow-up)

Jon confirmed the recorded Az=342° is correct to within ~1° — so the Az≈346°/Roll≈-128°
candidate (196 matches, faLogOdds 22-107) is **not** the true pose, and
`poorNoRollSeedRadialSupport` is correctly rejecting it. The question becomes: why does
neither `searchGuidedAnchorPose` nor `searchBrightAnchorVerifierRescue` ever produce a
high-scoring candidate near Az≈342° at all?

A second ground-truth point was added to `star-tests.csv` for narrow-7
(`HIP 11318:74:1601`, alongside the existing `HIP 11505:151:482`). Using the harness's
`findProjectedDetectionForExpectedStarNearPosition` diagnostic (camerastartests.cpp
~1225-1289, which projects each named star's catalog Az/El through a projector built from
the *raw recorded* `azimuth/elevation/roll/fov` = 342°/27°/0°/1.29° — solve fails so this
is the fallback path), HIP 11505 catalog-projects to (722.0, 651.8), 0.53px from detection
**#118** (`peak=33 flux=143 snr=24.7 hotPixel=true`), and HIP 11318 projects to
(748.9, 1752.6), 68.6px from **#14** (`peak=76 flux=912 snr=54.6`).

This looked initially like the CSV ground truth might be mislabeled (#118 sitting almost
exactly on the projection), but the photometry rules it out: #118 is a faint
hot-pixel-flagged detection, completely implausible for HIP 11505 (mag 7.78), whereas the
CSV's claimed match — detection **#157** (`peak=117 flux=5375 snr=153 saturated=true`) —
is exactly the kind of bright/saturated detection a mag-7.78 star should produce. The
0.53px "hit" on #118 is coincidental (hot pixel happens to land near where `roll=0`
places HIP 11505); the true roll for narrow-7 is very likely **not** 0°, so the
`roll=0` fallback projection used by this diagnostic doesn't reflect the true pose. The
original premise (HIP 11505 = #157 at (150,482), saturated) stands.

**Why `roll=0` is meaningless for narrow-7:** narrow-7's `plateSolveStartMode=3`
(`PlateSolveStartFovAzEl` — FoV+Az+El only, no roll). **For any test case where
`plateSolveStartMode` is below `PlateSolveStartFovAzElRoll` (4), the CSV's `roll` column
is not valid ground truth — it's an unused placeholder, not a measured value.** This is
the root cause of the "0.53px hot-pixel coincidence": `createDiagnosticProjector`
(camerastartests.cpp ~704-718) falls back to `test.roll` when the solve fails, with no
regard for `plateSolveStartMode`; for narrow-7 that fallback projector's orientation is
simply wrong. Added a code comment at that fallback noting the caveat. **Any future
diagnostic reasoning about narrow-7's "recorded pose" must treat roll as unknown** (Az≈342°
and El≈27° are still confirmed good to ~1°, per Jon).

**Rescue shortlist/pass-2 instrumentation added** to `searchBrightAnchorVerifierRescue`
(after the basin-deduped shortlist sort+resize, and inside the pass-2 loop), gated behind
`SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE`, logging each shortlist candidate's
Az/El/Roll/FoV/matches/rms/anchor indices, each pass-2 candidate's faLogOdds, and the
final `selected` pose. **Suite re-verified at 36/42, zero regressions** (with both this
and the prior `hasWeakNarrowGuidedBrightSupport` instrumentation in place, plus the new
HIP 11318 CSV row).

Ran on the isolated narrow-7 case (14 rescue invocations across the outer solve's
retries). **Every shortlist/pass-2/selected candidate across all 14 invocations clusters
in Az≈346.17-346.42°/El≈26.90-27.54°** — the same wrong basin as before; not one
candidate near the true Az≈342°/El≈27° appears anywhere in the rescue's pass-1 shortlist,
let alone pass-2. `selected faLogOdds` ranges 22.7-107.7 (best: 107.652 at
Az=346.176/El=27.2187/Roll=-118.062), all still rejected downstream by
`hasWeakNarrowGuidedBrightSupport` → `poorNoRollSeedRadialSupport` as before.

This sharpens the open question from a "did the rescue's shortlist crowd out the right
candidate" framing to: **why does `searchBrightAnchorVerifierRescue`'s pass-1 cheap sweep
(4 bright detections × 10 bright catalog stars × 36 rolls) never generate *any* seed near
Az≈342°, when the #157↔HIP 11505 anchor pairing is exactly the kind of bright-detection ×
bright-catalog-star pairing this rescue is supposed to try?** Per
[[plate-solver-failure-state]], an earlier session found `searchGuidedAnchorPose` (a
different search function) ranks the #157↔HIP 11505 pairing #1 (score 7.25) — so the
anchor pairing itself is discoverable; the rescue's pass-1 sweep or its
`guidedAnchorSearchScore`/`anchorAlignedPoseFromPixel` path may be excluding it (e.g. via
its bright-detection or bright-catalog-star candidate lists, or the cheap score ranking
it too low to survive `sameEvaluationBasin` dedup against the Az≈346° basin's higher
cheap scores). Not yet investigated — narrow-7 remains unsolved (6/42 unchanged).

## Dense-match FoV/pose polish experiment (2026-06-11, tried & reverted)

Investigated whether the 196-match/rms=11.07/Az=342.457/El=26.9165/Roll=-5.08025/FoV=1.29
candidate's `brightProjected=2/12` (the proximate cause of `hasWeakNarrowGuidedBrightSupport`
rejecting it) is itself caused by a small systematic FoV error. Added a temporary
`SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_BRIGHTPROJ` diagnostic (inside `evaluateFinalMatchPass`,
after the `brightProjectedStars` loop) that, for narrow-guided candidates with
`finalMatches.size() >= 150`, dumps each of the 12 brightest projected catalog stars: name,
magnitude, projected point, matched flag, and the nearest actual detection's distance/peak/
saturated flag. Result: **8 of the 10 unmatched bright stars (#3-10, mag 8.6-9.7) have a
real nearby detection (peak=129-138, not noise) at 5.15-10.45px** — consistent with the
candidate's overall rms=11.07px, i.e. a small near-uniform pose error (FoV is fixed for
narrow direction-seeded refinement) is plausibly preventing these matches.

Built a "dense-match FoV/pose polish": added an optional `forceFovRefine` parameter to
`refinePoseFromMatches` (frees the FoV LM parameter even for narrow guided solves), and a
new failure-path-only block (after `directionSeedAcceptanceFor` is first computed, before
the bright-anchor rescue) that — only when `!acceptable && weakBrightSupport && matches >=
minMatches+50` — reseeds LM from the candidate's own full match set with FoV free,
re-evaluates via `evaluateFinalMatchPass`, and adopts only if the result independently
re-passes `directionSeedAcceptanceFor` + `hasGeometricallyConsistentMatches` + retains
>=90% of the original matches (same safety pattern as `tightenNarrowFinalPass`).

**Result on narrow-7's 196-match candidate: the polish converged to FoV=1.27208/
Roll=-5.20795 with rms=11.07->2.08px, namedRms=11.47->1.81px, matches=196->193 (98.5%
retained) — a near-perfect fit.** But `brightProjected` only moved 2/12->3/12 and
`seedRadialMagnitudeMatchFraction` only moved 0.036->0.044; `poorNoRollSeedRadialSupport`
still rejected it (`weakBrightSupport` still true), so the polish never adopted for
narrow-7 (or anywhere else: the only narrow case where the precondition fired across the
suite was narrow-7 itself, with multiple per-outer-retry candidates all similarly
unaffected).

**Conclusion: pose/fit quality is not the narrow-7 blocker — even a sub-2px-rms,
98.5%-match-retained refit barely moves `seedRadialMagnitudeMatchFraction`.** This
strongly reinforces the 2026-06-11 finding in [[plate-solver-failure-state]] that
`poorNoRollSeedRadialSupport` (the fixed-reference-projector radial-consistency check
inside `hasWeakNarrowGuidedBrightSupport`) is the actual blocker, essentially independent
of how good the candidate pose itself is. **Reverted** — besides being inert, the extra
LM refit + final-match-pass re-evaluation on every qualifying narrow candidate measurably
slowed narrow-7's already-slow 17-outer-retry solve (observed >2x slower for that one
case in a full-suite run). Any further narrow-7 progress now points squarely at
`poorNoRollSeedRadialSupport`'s formula/reference-projector itself (cameraplatesolver.cpp,
inside `hasWeakNarrowGuidedBrightSupport` ~12700-12900) — recalibrating it is exactly the
"don't tune the accept gate blindly" risk flagged in [[plate-solver-failure-state]], so it
needs careful, narrowly-scoped changes validated against the full suite + synthetic corpus
before attempting. **Suite re-verified at 36/42, zero regressions** after reverting (clean
diff vs. session start). Confirmed via a full 42-case run on the reverted build: exactly
36 PASS / 6 FAIL, the 6 failures being narrow-3, narrow-7, wide-2 (x2), galaxy-m51-1
(m51@14), and ngc-2403 — identical to the established baseline.

## narrow-7 SOLVED: dense-match polish + named-bright-anchor certificate (2026-06-11)

Combining two changes makes narrow-7 PASS for the first time (matched=193, rms=2.08,
poseAz=342.457/El=26.917/Roll=-5.20795/FoV=1.27208 — inside Jon's confirmed Az=342±1°/
El=27° truth band, both named anchors HIP 11505 + HIP 11318 fit at namedRms=1.81px,
timeMs≈33.4s, *faster* than the failing ~78s runs because solving ends the outer retries):

1. **Dense-match FoV/pose polish re-applied** (same code as the reverted experiment —
   failure-path-only block after `directionSeedAcceptanceFor`, fires only when
   `!acceptable && weakBrightSupport && matches >= minMatches+50`, refits via
   `refinePoseFromMatches` with FoV freed (`forceFovRefine`), adopts only on >=90% match
   retention + geometric consistency + re-passing the acceptance gate). On narrow-7 it
   turns the 196-match/rms=11.07 candidate into a 193-match/rms=2.08 near-perfect fit.

2. **Named-bright-anchor certificate** (`hasNamedBrightAnchorCertifiedPose`, next to
   `hasDenseFinalEvidenceOverridingSeedRadial`): >=2 named bright anchors at <=3px rms
   + dense match set (>=max(minMatches+50,80)) at <=3px rms + candidate direction within
   1.5° of the seed reference. Used as (a) a top-level bypass of
   `hasWeakNarrowGuidedBrightSupport` (parallel to the high-confidence-triangle/
   strong-dense bypasses) and (b) a waiver for `poorNoRollSeedRadialSupport` in
   `hasAcceptableGuidedFinalBrightnessConsistency`. Margins vs the known-wrong Az≈346°
   contaminated candidates: they fit the same anchors at >=9.3px (3x the 3px bar) and
   sit >=4.2° from the seed (2.8x the 1.5° bar).

Why the certificate is needed — the three gate layers that each falsely vetoed the
correct polished pose (peeled via DEBUG_SPARSE on narrow7-only):
- `poorNoRollSeedRadialSupport`: radial error 146.7px vs ~96px threshold. The ~96px
  tolerance implicitly assumes the recorded pointing is good to a few hundredths of a
  degree; it is only trusted to ~1°, and the true pose sits 0.205° from the seed
  (= hundreds of px of radial discrepancy from pointing error alone).
- `seedProjectedBright>=4 unmatched, weak magSupport`: seed-projected POSITIONS assume
  the seed roll; narrow-7's start mode (3=FovAzEl) has no roll, the placeholder roll=0 is
  ~5.2° from the true roll, so every seed-projected position is off by ~r*0.09px.
- `brightProjected>=10, matched<5` (brightProjected=3/12): **assignment artifact, not a
  pose problem** — BRIGHTPROJ dump under the polished pose shows 11/12 brightest catalog
  stars have a real detection within 0.37-2.64px, but 8 of them are unmatched because in
  the dense mag-15 catalog (662k stars) their detections were *assigned* to nearby
  fainter catalog stars. `matchedBrightProjectedStars` measures assignment, not
  positional coincidence, and undercounts exactly when the catalog is dense.

Validation (complete):
- narrow7-only: PASS.
- Full 42-case real suite: **37/42, zero regressions** — narrow-7 now PASSES; the 5
  remaining failures are exactly the established narrow-3, wide-2 (x2), m51@14,
  ngc-2403.
- Synthetic random corpus (`star-tests-synthetic-rand.csv`, 100 cases): fixed build and
  a pre-change baseline build score **identically (59/100, same cases)** — the change
  has zero effect there. The polish never adopted in the synthetic run (its
  unconditional adoption log appears 0 times) and the certificate requires rms<=3 so it
  cannot have accepted the rms-13..17 failures observed.

**Separate finding (NOT caused by this change): the synthetic corpus has drifted from
~99/100 (measured 2026-06-05, post-precession-fix) to 59/100**, with many wrong-pose
`solved=true` false positives at rms 13-17 and ~10-50 matches — present identically in
the pre-change baseline. Bisected and root-caused the same day, see the next section.

## Synthetic corpus 99→59 collapse: bisected & fixed (2026-06-11/12)

Two stacked causes, fully separated by bisection:

**Cause 1 (the big one, 59 vs 82): an UNCOMMITTED leftover experiment in
`searchBrightAnchorVerifierRescue`** — a non-debug-gated block that appended up to 4
"in-frame" bright catalog stars (`kMaxRescueInFrameCatalogStars`) to the rescue's anchor
pool (added alongside the debug-gated shortlist instrumentation in the 2026-06-11
rescue-instrumentation session, but unlike the instrumentation it changes behaviour and
was never validated against the synthetic corpus). Bisection method: swapped the
test-target sources (all camera headers + the 6 test TUs) to each committed state in the
build tree (backups in C:\tmp\bisect_backup), built, and ran a 10-known-failure subset
(`synthrand-bisect.csv`): commits 49749e9b3 (06-08), 44cb8f741 (06-10), 86200e672
(06-11/HEAD) all score 8/10; the working tree scored 0/10; removing JUST the in-frame
block from the working tree → 8/10. Failure mechanism observed on synth-rand-013 (truth
Az=77.83/El=64.72, random true roll): the solver finds an 82-match rms=0.16px true pose
via bright-triangle seed, but with the in-frame block the final selection ends up a
22-match rms=16.1 roll-alias (Az exact, Roll +25.3°) accepted as solved=true.
**Fix: removed the block. Validation: real suite 37/42 zero regressions (narrow-7 still
PASS), synthetic 59 → 82/100.**

**Cause 2 (82 vs ~99): the on-disk `star-tests-synthetic-rand.csv`/`images-synthetic-rand`
are STALE — generated 2026-06-05 09:40, BEFORE the generator's precession fix** (commit
afeac3e56, 06-05 20:18). The 18 residual failures (022, 024, 030, 035, 038, 042, 048,
052, 053, 054, 055, 066, 070, 075, 078, 094, 096, 099) are honest non-solves
(recenter-marathon, every run solved=0 incl. depth-escape) and match the documented
"precession-hard" class — 82/100 is *exactly* the documented pre-regeneration score.
The ~99/100 was measured on a corrected-seed corpus. `star-tests-synthetic-rand2.csv`
(150 cases, generated 06-05 19:29, during the precession-fix session) is the
corrected-generation corpus — **validated: 146/150 (97.3%) PASS on the fixed build**
(failures: rand-a-005, rand-a-023, rand-c-009, rand-c-023 — not investigated, presumed
the corpus's own hard tail, analogous to rand-066).

**Bottom line / going forward:** the solver is healthy — real suite 37/42, corrected
synthetic corpus 97.3%. Use `star-tests-synthetic-rand2.csv` as the canonical synthetic
corpus for validation; the 100-case `star-tests-synthetic-rand.csv` on disk is a stale
pre-precession-fix generation whose ~18 recenter-marathon failures are a known test-data
artifact, not solver weakness (regenerate it with the current `synthetic_testcases.py`
if a 100-case corpus is wanted). When validating future changes, expect ≈82/100 on the
stale corpus and ≈146/150 on rand2.

## rand2 triage + m51@14 SOLVED via deepen-escape (2026-06-12)

**rand2's 4 failures triaged:**
- **c-023: GENERATOR BUG, fixed.** The solve was perfect (90 matches, rms=0.168, pose
  exactly on truth) but ground-truth star HIP 92043 (110 Her, V=4.19) sits on *empty
  sky* — `synthetic_testcases.py` picks named ground-truth stars from Hipparcos
  (`I/239/hip_main`) but renders only Gaia DR3 stars, and Gaia lacks bright stars, so a
  bright HIP pick may never be rendered (verified visually in the overlay crop). Fixed
  `pick_named` to require a rendered Gaia counterpart within 2px; patched the existing
  rand2 c-023 row to drop HIP 92043 (the 2 remaining ground-truth stars validate the
  pose). c-023 now PASSES → rand2 baseline is **147/150**.
- **a-023: genuine roll-alias FALSE POSITIVE** (solver-side, open): accepted pose has
  direction exactly right (az 28.0477 vs truth 28.05) but wrong roll, 12 matches,
  rms=16.85, and misses 2 of 3 ground-truth stars (incl. HIP 26358 at mag 8.51, which IS
  rendered and in Gaia — so the pose is truly wrong). Next solver-side accept-gate
  target.
- **a-005 / c-009: honest sparse non-solves** (63/52 detections, best 5-11 matches;
  a-005's best attempt drifts 1.27° in az) — same hard-tail class as rand-066.

**m51@14 SOLVED — real suite now 38/42.** Mechanism (two stacked obstacles):
the @14 row is catalog-starved (509 detections vs 103 in-FoV candidates) AND its seed is
deliberately ~1° off (az 92.0 vs true 93.0). The outer recenter ladder runs only at the
requested depth (starved), while a plain deeper retry inherits the raw off seed (mag 15
from az 92.0 reaches only 46 matches vs 149 from the true centre). Fix: **deepen-escape
retry** (after depth-escape in `CameraPlateSolver::solve`): when a narrow direction
no-roll solve fails with `detections >= 128 && candidates <= detections/3`
(catalog-starved trigger), retry at requestedMag+1/+2 (capped 16.5, deduped via the
attempted-magnitude set), each depth sweeping coarse azimuth recenter offsets
{0, ±0.75·fov, ±1.0·fov}, budget 6 runs total, adopting any *solved* result.
**No rms gate, deliberately**: unlike depth-escape (which escapes INTO the lenient
sparse-catalog acceptance regime and needs its own tight-fit bar), deepening moves into
the denser, *stricter* acceptance regime — an adopted result is exactly as trustworthy
as the same solve requested at that depth directly; and correct galaxy-field solves
carry rms 12-13px from fuzzy detections (m51@15: rms 13.17), which a 3px bar would
wrongly reject. m51@14 now solves with matched=150, rms=13.01,
poseAz=93.0067/El=72.764/Roll=-1.546 (the known-true pose).

**Validation:** real suite **38/42, zero regressions** (remaining: narrow-3, wide-2 ×2,
ngc-2403; narrow-7 still PASS); rand2 **147/150, zero regressions** (a-005, a-023,
c-009). The deepen trigger fired only on m51@14 across both suites (verified by log
grep), as designed — ngc-2403 (1178 detections, 424 candidates) correctly misses the
starvation trigger.

## ngc-2403 SOLVED + a-023 false positive fixed (2026-06-12) — suite 39/42, rand2 148/150

The long-standing "density-adaptive match radius" hypothesis was tested and **disproven**:
sweeping the final match radius 24/12/8/6 px (via a temporary
`SDRANGEL_CAMERA_STAR_TEST_FINAL_MATCH_RADIUS` override in the harness, kept for future
experiments) showed pure chance scaling (matched 217→84→42→30, rms tracking the radius)
with no true pose emerging — the true pose was never in the candidate set. ngc-2403 was a
SEARCH-side gap, not a selection problem. Two stacked root causes, both fixed:

1. **Rescue anchor-pool gap.** `searchBrightAnchorVerifierRescue`'s catalog anchor pool
   (top-10 brightest within the multi-FoV localRadius) was entirely mag 3.1-5.3 stars
   OUTSIDE the 1.27° frame (Muscida, π² UMa, ...); the brightest star actually in the
   image — HIP 37078 at mag 8.2, detection #703 (flux 8371, saturated, ground-truth
   anchor) — could never be paired. Fix: dense-gated (>= 512 detections, narrow) in-frame
   anchor tiers appended to the pool — tier 1 takes the brightest 4 within the frame's
   *inscribed circle* (in-frame at ANY roll, which matters since roll is unknown; without
   this tier, HIP 37078 at 0.32° from centre is crowded out by mag-5.6-6.5 stars in the
   wider radius that sit outside the actual frame), tier 2 the brightest 4 within the
   possibly-in-frame radius. The >= 512-detection gate deliberately excludes the
   sparse/moderate lenient-acceptance regime whose wrong-roll adoptions caused the
   2026-06-11 synthetic-corpus collapse when an ungated version of this idea was tried;
   no synthetic case and no other real case reaches 512 detections on the failure path
   (pollux at 792 solves directly, so its rescue never runs).

2. **LATENT result-sync bug in the rescue adoption block** (present since the rescue
   landed 2026-06-08, never triggered because no adoption had ever actually fired): on
   adoption it updated `selectedFinalPass`/acceptance/scores but NOT
   `result.m_azimuthDegrees`/elevation/roll/fov/center/k1, `m_matchedStars`,
   `m_rmsErrorPixels`, `m_maxErrorPixels`, `m_matchSummary`, or the per-detection labels —
   so the run reported solved=true carrying the *previous* (junk) selected pose. With the
   in-frame anchors, ngc-2403's rescue found and adopted the true pose
   (faLogOdds=365 vs ~90 for every contaminated candidate — the verifier separation is
   dramatic) yet the result still showed the old 191-match/rms-15.9 junk. Fixed by
   mirroring the dense-match-polish block's full result sync.

ngc-2403 now solves: **matched=389, rms=5.22, Az=318.434/El=60.045/Roll=-32.834**,
ground-truth HIP 37078 position check satisfied. **Bonus: rand2's a-023 wrong-roll false
positive disappeared with the sync fix** (its solved=true-with-wrong-pose was the same
latent bug surfacing through a rescue adoption) → rand2 is now **148/150** (only the
honest sparse a-005/c-009 remain).

Validation: real suite **39/42, zero regressions** (remaining: narrow-3, wide-2 ×2;
narrow-7 + m51@14 still PASS, pollux/m51-2/cluster canaries unchanged); rand2 **148/150,
zero regressions**.

## narrow-3 SOLVED via density-scaled rescue shortlist (2026-06-12) — suite 40/42

One-line root cause: the rescue's 5-slot global shortlist was crowded out by
contaminated basins before pass-2's verifier ranking ever saw the true candidate — the
hypothesis recorded in the narrow-7-era notes, now confirmed and fixed.

Diagnosis with the certificate-era tooling: the old "only ~7 of 45 detections match at
the true centre" finding was **stale** — the true pose actually matches 30/37 in-FoV
candidates. The rescue's pools were fine all along: ground-truth star HIP 73199 is
mag 3.17 (bright!), in the catalog anchor pool, and its detection #26 (flux 7003, at the
ground-truth pixel) is the top bright detection. But on this sparse field (45 detections,
37 candidates at mag 12) the pass-1 cheap scores are FLAT — true and wrong-roll basins
all land within 7.4-7.6 — so the global top-5 cut dropped the true basin; the selected
junk candidate scored faLogOdds 4.6 (vs ngc-2403's true-pose 365: the verifier separates
fine when it gets to see the candidate).

Fix: `kMaxRescueShortlist` is now density-scaled —
`clamp(2400 / detections, 5, 24)` — pass-2's per-candidate cost scales with
detections × candidates, so a 45-detection field affords 24 verifier-ranked candidates
for the same budget 5 costs on a 1178-detection one. Dense-field behavior is unchanged
(>=480 detections still gets 5), preserving the 24x-slowdown lesson that motivated the
original cap.

narrow-3 now solves: matched=30, ground-truth HIP 73199 position check satisfied.
Validation: real suite **40/42, zero regressions** (narrow-7, m51@14, ngc-2403 all still
PASS; remaining failures are only the two stars-wide-2 fisheye rows, which need the
blind quad-hash workstream); rand2 **148/150, zero regressions** (a-005/c-009 honest
sparse hard tail).

## Performance pass (2026-06-12) — suite solve time -28.8% (385.5s → 274.6s)

Profiled with the built-in stage/run profiling already embedded in every result line
(`timeMs`, `solve.runN.reason/ms/solved/matches`, `stages=`) — no external profiler
needed. Findings: ~38% of total suite time was `recenter-no-roll-recovery` retries and
~34% `bright-catalog` re-runs, much of it provably wasted (narrow-4 solved at recenter
attempt 4 then burned 13 more; cluster-m7 re-ran a 9.4s bright-catalog pass after already
solving with 1447 matches, and adopted the same pose with half the matches).

Four outer-retry-ladder fixes (all in `CameraPlateSolver::solve`):
1. **Density-aware recenter early-stop**: the strong-solve early-stop bar (>= 80 matches)
   is unreachable on sparse fields whose whole in-FoV catalog is a few dozen stars; a
   solved result matching >= max(minMatches+8, 60% of candidates) now also stops the
   ladder.
2. **Deepen-escape hoisted before the recenter ladder** (as `attemptDeepenEscape`, late
   call kept as fallback): the catalog-starvation signature is visible right after the
   initial run, and the ladder cannot help that class (m51@14 burned ~30s of ladder
   before the deepen that solves it).
3. **Deepen adoption gated on dominant candidate coverage** (matches >= 60% of the
   deeper catalog's in-FoV candidates): the hoist exposed a LATENT false-positive path —
   m101@15's deepen-to-16 from an offset seed produced a gate-passing wrong-roll alias
   (134/328 = 0.41 coverage, roll -25 vs true 87) where m51@14's correct adoption is
   150/165 = 0.91. The gate also protects the original late-call path, which had the
   same latent risk.
4. **Alias-contamination bail-out**: a solved-but-low-coverage deepen result means the
   deeper catalog feeds acceptable-looking aliases for this field — the whole escape
   stops at the first such result instead of spending the remaining offset/depth budget.
   Plus: **post-recenter bright-catalog retry skipped when already solved with >= 120
   matches** (its useful work is rescuing unsolved/weak results; for solved ones it only
   re-derived the same pose from a shallower catalog).

Validation: real suite 40/42 and rand2 148/150, identical failure sets (wide-2 ×2;
a-005/c-009). Per-case highlights (suite-run timings): m51@14 49.3→10.1s, narrow-4
34.6→12.3s, cluster-m7 20.3→10.0s (and now keeps the better 1447-match result),
narrow-9 15.6→5.5s, narrow-1 17.8→10.5s, narrow-3 19.2→12.7s, narrow-5 11.2→5.0s.
Costs accepted: m101@15 +4.2s (pays one rejected deepen probe before its ladder solve),
narrow-7 +5.9s (run variance / wider sparse shortlist).

## Performance pass, round 2 (2026-06-12) — 274.6s → 250.0s (cumulative -35% from 385.5s)

Round 1 left 59% of solve time dark (the stage keys in the result line cover only the
final run, and several hot blocks had no timers at all). Round-2 instrumentation
(permanent `logSolveProfile` timers around `acceptance`, `tightenFinalPass`,
`densePolish`, `brightAnchorRescue`; run the suite with
`SDRANGEL_CAMERA_PLATE_SOLVER_PROFILE=1` and aggregate the
`CameraPlateSolverProfile solve.<stage> elapsedMs=` lines) closed the gap. Suite-wide
stage costs (profiled run):

| stage | calls | total |
|---|---|---|
| searchBestPose | 90 | 138.8s (~60%) |
| acceptance (post-final-pass scoring/gates) | 75 | 16.7s |
| rollRecoveryFinalPass | 21 | 14.5s |
| catalog (context build) | 91 | 11.9s |
| refineCandidates | 79 | 11.7s |
| brightAnchorRescue | 75 | 9.4s |
| tightenFinalPass | 90 | 7.0s |

Changes landed this round:
- **Catalog-context memo** (`buildPlateSolveCatalogContext`): exact-input-key LRU-3 of
  the parsed+aliased+merged context — the outer ladder rebuilds it per run (~0.5s each,
  multi-100k-star parse + alias pass) and revisits identical inputs (repeated recenter
  offsets, same-seed runs). ~10 of 91 builds hit in the suite; behaviour-identical on
  hits (QVector COW keeps cached copies cheap).
- The permanent stage timers above (negligible cost, big diagnostic value).

Validation: real suite 40/42, rand2 148/150, identical failure sets; clean-run solve
total 250.0s.

**Where the remaining time is — and why round 3 needs a decision:** ~60% of solve time
is inside `searchBestPose`, dominated by triangle-seed *evaluations* (~4000/run × 90
runs; the signature construction itself is minor). The options are (a) cross-run
evaluation reuse (structural, medium risk), (b) lowering the evaluation caps
(4000/2048/768 — directly risks losing solves; would need full real+synthetic
revalidation per step), or (c) hot-loop micro-optimisation of the seed evaluators
(lowest behavioural risk, grindy). All three touch the core search that the 40/42
balance rests on — deliberately NOT attempted without an explicit go-ahead.

## Performance pass, round 3 (2026-06-12) — 250.0s → 141.9s quiet-run (cumulative -63% from 385.5s)

Targeted the `searchBestPose` interior (60% of solve time) after a gen-vs-eval split
timer showed candidate GENERATION dominates the bright-anchor-triangle stage ~2:1 over
seed evaluation. Three provably-behaviour-preserving fixes in
`buildBrightGuidedAnchorTriangleSeeds`:

1. **Environment lookup hoisted out of the triple loop**: the
   `SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_TRIPLE` `qEnvironmentVariableIsSet` call was made
   per (a,b,c) combination — up to ~900k getenv calls per run.
2. **Pairwise tables instead of in-loop recompute**: detection pixel distances
   (sqrt) and catalog angular separations (acos·dot) were recomputed per triple; now
   computed once into O(N²) tables (N ≤ 192) and read by the O(N³) loop. The seed
   direction vector was likewise rebuilt per ratio match.
3. **Catalog-triple work hoisted out of the detection-triple loop** (the ordered-triangle
   search): the magnitude sort and triangle signature of each of the ≤2024 catalog
   triples were recomputed for every one of the ≤1540 detection triples (~3M sorts +
   signature builds per run on rich fields). Precomputed once, in the exact (i,j,k)
   iteration order so the candidate encounter order — and the kept-768 cap behaviour —
   is unchanged.

The hot-subset spot check produced bit-identical match counts on all 8 cases, and the
full validation held 40/42 / 148/150 with identical failure sets. Per-case quiet-run
wins: narrow-7 43.1→28.5s, m101@15 24.5→11.4s, narrow-4 9.6→3.6s, m51@14 9.6→4.2s,
ngc-2403 9.1→3.8s, narrow-3 9.4→4.7s. Caveat: suite wall time varies ±10-15% with
machine load (a contended run measured 278.9s with the same binary — per-case analysis,
not totals, is the reliable signal; the optimized-path cases improved even in the
contended run).

## Performance pass, round 4 (2026-06-12) — negative result, safe tier exhausted

**Tried and reverted: catalog-query-centre quantization.** Snapping the catalog region
query centre to a 2° grid would have let every recenter/escape retry share one memoized
context (the region radius ~14° dwarfs the ≤1.3° recenter offsets, and
`buildVisibleCatalog` is direction-agnostic, so this looked behaviour-preserving). It is
not: the spot check changed solver results (narrow-7 matched 193→176; m101@15 145→122
and slower). **Lesson: the loaded region's exact content is implicitly part of solver
behaviour** — candidate/signature selection ranks stars over the whole region, so a
1-2° shift in region centre changes which stars enter the pools even though none of
them are near the image. Any region-changing optimization is therefore in the
behaviour-affecting class, not the provably-safe class. Reverted; revert verified
bit-identical (193/1542/145 restored).

Also added a `solve.rollAliasCheck` stage timer (the `hasCompetitiveRollAlias`
correctness gate evaluates alternate-roll hypotheses inside the acceptance span — now
measurable separately).

**State of play after four rounds:** quiet-run suite total ~142-150s (from 385.5s,
≈ -62%), 40/42 + rand2 148/150 throughout. The remaining time is concentrated in
behaviour-coupled components: `hasCompetitiveRollAlias`, the rescue's pass-1 sweep, the
roll-recovery grid (already parallel), and core-search seed evaluation already
hoisted/optimized in round 3. Further gains require behaviour-affecting changes
(evaluation-cap tuning, retry-ladder restructuring) with full dual-corpus revalidation
per step — diminishing returns; recommended stopping point.

## Blind quad-hash geometric indexing — implementation plan (2026-06-12, not started)

**Goal:** solve the last two real-suite failures — `stars-wide-2.jpg` row 15
(startMode=1 Fov: 165° equidistant fisheye, fov known, direction/roll unknown) and
row 16 (startMode=0 Blind: fov placeholder 9.0, lens model known from settings) —
and structurally strengthen all blind/fov-only solving with a rotation-invariant,
distortion-free quad index. Guided-path adoption and prebuilt deep indexes are
explicitly out of scope (future go/no-go items at the end).

### What exists today (and why it fails on wide-2)

The blind seed pipeline (`searchBestPose`, ~:18230-18300) already runs
bright-pair seeds → `buildBlindTriangleSeeds` → `buildBlindQuadSeeds` (:10798) →
wide-fallback grid (72az × 7el × 13roll × 8fov, roll-sweep cached via
`BlindGridCachedStar`). The quad stage has four structural limits:

1. **Frame mismatch (the core defect).** Detection quads are built in *raw
   distorted pixel space* (`buildDetectionQuadSignatures`, detection `m_center`),
   while catalog quads are projected through *synthetic ~rectilinear local
   projectors* (`buildCatalogQuadSignatures`, `syntheticFov` clamped 20-160°). At
   165° equidistant fisheye the same 4 stars produce different shapes; the
   band-aids (`ratioTolerance=0.06`, `ignoreOrientationHandedness` for fisheye)
   only mask it for compact quads and inflate false matches.
2. **Coverage starvation.** Catalog quads use only the top-24 brightest visible
   stars (C(24,4)=10,626 mostly hemisphere-spanning quads = maximal distortion
   mismatch); detection quads use the top 14-16 detections.
3. **Quantized bucket hashing** (`buildQuadSignatureBuckets`, qint64 keys) has
   bucket-boundary misses vs a continuous ε-search.
4. The fallback grid is too coarse to recover what the quads miss (5° az steps).

### Design — vector-space quad codes over the loaded catalog context

Adapted from astrometry.net, with one deliberate departure: we index the
*per-solve loaded catalog context* (visible hemisphere at mag≤7-8 ≈ a few
thousand `VisibleCatalogStar`s, vectors already computed for capture
time/location, refraction included) instead of prebuilt all-sky files. That is
fast enough to build in-process and obeys the round-4 lesson (region content is
behavioural → derive everything deterministically from the existing context,
cache exact-key only).

1. **Detection rays.** Unproject every detection to a camera-frame unit vector
   via a reference projector at az=0/el=0/roll=0 with the known
   fov/projection/k1 (`unprojectPixelToVector` :3264 already exists). This
   removes ALL lens distortion *before* any geometry — the fix for defect #1.
   With proper rays, handedness is consistent again, so the
   `ignoreOrientationHandedness` concession can be dropped on this path (extra
   discrimination for free).
2. **Continuous quad code** (identical function for catalog vectors and
   detection rays — no synthetic projectors anywhere): pick the quad's
   most-separated pair (A,B); gnomonic-project all 4 stars onto the tangent
   plane at the quad centroid; affine-map A→(0,0), B→(1,1); code =
   (xC, yC, xD, yD), canonicalized (swap C/D so xC≤xD, swap A/B so xC+xD≤1,
   parity bit instead of mirroring). Rotation/translation/scale-invariant.
   When fov is known (mode 1), append a 5th dimension = normalized AB
   great-circle angle — scale awareness cuts false matches ~an order of
   magnitude.
3. **Uniformized catalog quad generation.** Partition the hemisphere into az/el
   cells sized to the scale band; keep the N brightest per cell; build quads
   from each star + 3 of its k nearest neighbours within the band; two bands
   for wide work (~10-30° and ~30-70°); cap quads/star (~8) and total (~50k).
   Bounded O(N·k³), estimated tens of ms.
4. **Index.** Small in-house 4D/5D k-d tree over codes with ε-ball query
   (ε ≈ 0.01-0.02 code units, tuned on the synthetic fisheye corpus). Replaces
   the quantized buckets (defect #3) — no boundary misses.
5. **Hypothesis → pose.** Each code hit gives 4 ray↔sky correspondences →
   N-vector overload of the existing quaternion Wahba solver
   (`wahbaRotationFromVectorPairs` :3637 — the correlation-matrix accumulation
   generalizes trivially) → rotation → az/el/roll (+ fov known or recovered) →
   `Evaluation` seed → existing `consumeBlindSeeds` →
   `evaluateFinalMatchPass` + **unchanged acceptance gates**.
6. **Verification budget.** Rank hypotheses by code distance + quad brightness;
   verify top K (~50) through the existing final-match pass; early-exit on
   `hasGoodWideBlindSeed()` exactly as the current stages do.

**Mode 0 (fov unknown):** unprojection needs fov, so run a coarse fov ladder
*for ray generation only* — {60, 100, 130, 165, 180}° (reuse the
`wideBlindFovs` constants) × the mode-1 engine, stop at first accepted pose.
Matching + Wahba + the existing LM/fov refine (`forceFovRefine`) should
tolerate ~±15% fov error in the rays; phase 0 measures the actual tolerance to
set ladder spacing. Lens *model* is known even in mode 0 (settings carry the
projection type); truly-unknown lens is out of scope.

### Integration & gating

- New engine inside/alongside `buildBlindQuadSeeds` (:10798), invoked from the
  blind-seed stage (:18288). **New path runs first; the legacy quad path stays
  as fallback** when no hypothesis passes the gates — protects wide-1/3/4,
  which currently pass through the legacy machinery.
- Gated to blind/fov-only solves (`!solveUsesDirection`) — the 39 guided
  passing cases never enter the new code, so guided regression risk is zero by
  construction.
- Env kill-switch `SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_QUAD_INDEX` for A/B.
- The built index is cached with the catalog context in the existing exact-key
  LRU-3 memo so the up-to-17 outer retries don't rebuild it.

### Phases

- **Phase 0 — design validation before engine code (~½ day). DONE
  (2026-06-12), design VALIDATED.** One-off Python calculation (astropy for
  star alt/az at the row-15 capture time/location/pose: az=90°/el=70°/roll=100°,
  165° equidistant fisheye, 3552×3552 — confirmed against the CSV's
  Arcturus/Dubhe/Vega pixel positions to ~60-85px / ~2% of frame, consistent
  with the recorded pose's documented ~1° precision). For all C(7,4)=35 quads
  of the 7 named stars (max pairwise separation 37-72°, representative of
  real bright-star quads at this fov):
    - **CURRENT scheme** (raw equidistant-fisheye pixels vs synthetic
      rectilinear-projected catalog, edge-ratio signature): mean mismatch
      0.0555, max 0.1347 — **12/35 (34%) exceed the 0.06 bucket tolerance
      entirely**, i.e. the true catalog quad would never even be hash-bucketed
      together with its true detection quad, independent of detection noise.
      This is the quantitative confirmation of the "frame mismatch" defect.
    - **NEW scheme** (detections unprojected via a fixed az=0/el=0/roll=0
      reference projector to camera-frame rays, both sides gnomonic-projected
      and compared by the same edge-ratio signature): mismatch is **0 to
      machine precision (4e-15)** for all 35 quads, exactly as the rotation-
      invariance argument predicts (camera-frame rays and sky vectors differ
      only by the camera's pose rotation, which both quad-signature
      construction and edge-ratio comparison are invariant to). With added
      1.5px-rms synthetic centroid noise, mismatch stays tiny: mean 0.0023,
      max 0.0069 — **~8-20x tighter than the current 0.06 tolerance**, so a
      much smaller ε (~0.01) is usable for the k-d tree, dramatically cutting
      false-bucket collisions vs the current scheme.
  **Conclusion: proceed.** The new scheme doesn't just fix the 34% miss rate —
  it makes true-quad matching near-exact, which is the property the whole
  index/ε-ball design depends on. Script: `/tmp/phase0_quad.py` (not
  committed — one-off; reproducible from this section + astropy).
  Ray-error-vs-fov-error measurement for the mode-0 ladder spacing was not yet
  done; fold into phase 1/2.
- **Phase 0b — synthetic fisheye corpus.** DONE (2026-06-13). Added
  `vec_from_altaz_arr`/`make_sky_projector`/`project_points` (ported from the
  validated Phase-0 script, square aspect=1) plus `process_fisheye_scene` and
  vectorised `radec_to_azel_arr`/`precess_j2000_to_date_arr` helpers to
  `synthetic_testcases.py`. New `--fisheye N` CLI mode generates random
  az/el/roll/fov scenes (fov 100-170°, equidistant/equisolid mixed, mag≤7,
  el≥30°), querying Gaia DR3 + HIP directly in alt/az (no WCS/TAN — the
  pose IS the projector, matching `createProjector`/`projectVector`).
  - **Fisheye corpus**: 50/50 scenes accepted (70 attempts, ~71% hit rate) →
    `star-tests-synthetic-fisheye.csv` (100 rows: modes 0+1 per scene),
    images in `images-synthetic-fisheye/` (1600x1600 jpg). Spot-checked
    `synth-fisheye-001.jpg`: renders as a plausible Milky-Way-band field
    (~695 stars at G≤7, named HIP stars land inside the rendered cluster as
    expected).
  - **Rectilinear-wide corpus**: reused existing `--random` (TAN-based)
    mode at fov 30/45/60° → `star-tests-synthetic-wide.csv` (36 rows from
    12 scenes). Acceptance rate degrades sharply with fov: 30°→6/17≈35%,
    45°→6/17≈35%, 60°→0/7. **Known limitation**: the single-CD-matrix TAN
    WCS used by `process_scene`/`build_wcs` becomes unusable above ~fov 50°
    (catalog stars project far outside the frame), so it cannot produce
    rectilinear-wide cases beyond ~45°. Not blocking Phase 1/2 (which target
    the fisheye corpus); revisit only if a rectilinear-wide quad-hash case
    is specifically needed later (would require a per-axis CD scale or a
    `make_sky_projector(lens="rectilinear")`-based generator instead of TAN).
- **Phase 1 — vector quad engine, mode 1 (2-3 days).** Items 1-6 above.
  Exit: wide-2 row 15 PASS; synthetic fisheye mode-1 ≥90%.
  - **Items 1-5 (Wahba/N-pair, vector quad code, ray unprojection, catalog
    quad index, hypothesis generation) and item 6 (integration with gating)
    DONE and build-verified** (`featurecamera_star_tests`, 26/26 link steps).
    No regressions: real 42-case suite still 40/42, identical failure set to
    baseline (only wide-2 rows 15/16, mode1/mode0).
  - **Exit criteria NOT met** (2026-06-13 validation run):
    - wide-2 row 15 (mode1) still FAILS (matched=14/541, rms=14.35,
      solved=false) — `search.vector-quad-seeds` never ran for this case.
    - Synthetic fisheye mode-1 corpus (`star-tests-synthetic-fisheye-mode1.csv`,
      50 cases): **15/50 = 30%**, far below the ≥90% target.
  - **Critical Finding #1 — gating makes the new engine dead code for almost
    all target cases.** The outer skip condition at ~19119
    (`brightTriangleSeedAlreadyAcceptable || brightPairSeedAlreadyAcceptable
    || wideBrightPairSeedAlreadyAcceptable`) is already true after the legacy
    bright-pair-seed stage for wide-2 row 15 and for 45/50 synthetic fisheye
    mode-1 cases, so `buildVectorQuadBlindSeeds` is skipped entirely
    (`blindQuadSkipped=1`). The new engine only ran in 5/50 synthetic cases
    (009, 019, 021, 035, 049).
  - **Critical Finding #2 — even when it runs, code-match yield is near zero.**
    In the 5 cases where the engine did run: `vectorQuadCatalogEntries=6496`
    (identical across all 5) and `vectorQuadDetectionQuads=1820` (=C(16,4),
    all valid), but `vectorQuadCodeMatches` was only 0, 6, 0, 19, 6
    respectively (epsilon=0.02). Where matches existed, none were strong
    enough to satisfy `hasGoodWideBlindSeed()`, so legacy
    `buildBlindQuadSeeds` ran anyway as fallback in all 50 cases.
  - **Root cause of #2 — combinatorial structure mismatch between catalog and
    detection quads, confirmed by reading `buildCatalogQuadCodeIndex`
    (:6938-7157).** Catalog quads are *anchor-centric*: for each of a small
    set of per-cell "anchor" stars (brightest `maxStarsPerCell=6` per 10°x10°
    az/el cell), quads are built from that anchor plus 3-of-its-6
    angularly-nearest neighbours (`neighborCount=6`,
    `maxQuadsPerAnchor=8` ⇒ ≤C(6,3)=20 capped to 8), giving 6496 total
    entries. Detection quads, by contrast, are *unstructured*: all
    C(16,4)=1820 combinations of the 16 brightest detections. A true
    4-star correspondence is therefore only representable in the catalog
    index if one of the 4 stars happens to be a kept anchor AND the other 3
    are among that anchor's top-6 nearest catalog neighbours AND that
    specific 3-subset is among the (≤8 of ≤20) kept per-anchor combos —
    multiple independent caps that compound multiplicatively. Most genuine
    detection quads simply have no matching catalog entry at all, regardless
    of epsilon.
  - **Implication / open design decision (not yet actioned):** fixing this
    needs an architectural change to one or both of: (a) where the new engine
    sits in the gating order (e.g. run it *before* the bright-pair/triangle
    stages, or run it unconditionally and let `hasGoodWideBlindSeed()` pick
    the best result across all seed sources), and (b) how catalog vs.
    detection quads are generated so their combinatorial supports overlap —
    either make catalog quads unstructured-anchor-free (all C(k,4) among a
    modest top-k per region, mirroring the detection side) or make detection
    quads anchor-centric (per-detection k-nearest-neighbour 3-subsets,
    mirroring the catalog side). Either direction is a non-trivial rework of
    items 4/5 and re-validation of Phase 1 exit criteria; flagged for
    go/no-go before continuing.
  - **Direction 1c implemented (2026-06-13, approved by Jon):** (a) detection-quad
    generation reworked to be anchor-centric (mirroring
    `buildCatalogQuadCodeIndex`: per detection-anchor, 6 nearest neighbours in
    `[2°,70°]`, ≤8 of the 3-of-6 combos ⇒ 128 detection quads total, down from
    1820); (b) `buildVectorQuadBlindSeeds` now runs *early* for
    `wideWeakMode && plateSolveStartUsesFov(settings)`, before the outer
    `brightTriangleSeedAlreadyAcceptable || ...` skip check, with a
    `vectorQuadSeedsRunEarly` flag guarding the original late call site so it
    doesn't run twice.
  - **Result of 1c: build-verified, zero regressions, but still NOT exit-criteria-met.**
    Real 42-case suite still 40/42, byte-identical failure set/values
    (wide-2 rows 15/16 unchanged: row15 matched=14/541 rms=14.3469,
    `vectorQuadCodeMatches=0`). Synthetic fisheye mode-1 corpus initially
    **unchanged at 15/50=30%**, bit-identical per-case PASS/FAIL and pose
    values to the pre-1c baseline — despite the engine now running broadly
    (`vectorQuadCodeMatches` non-zero, typically 4-15, in nearly all 50 cases
    vs only 5/50 before).
  - **New finding — code matches exist but all hypotheses fail
    `verifyBlindSeedCandidate`'s tight RMS gate.** Debugged `synth-fisheye-013`
    (codeMatches=15, all 15 hypotheses reach `evaluatePose`): every hypothesis
    converges to one of 3-4 nearby poses with `candidate.matchCount`=14-24 and
    `candidate.rmsErrorPixels`≈15.9-17.8 — i.e. the quad-derived pose is
    basically *correct* (lots of catalog matches) but ~2-4px too coarse for
    `isStrongBlindSeedEvaluation`'s `maxRmsError = min(seedRadius*0.60, 18.0)`
    = 14.4px (seedRadius=24 in the test harness). `verifyBlindSeedCandidate`
    gates on this *before* its own internal outlier-rejection/refine step ever
    runs, so a pose that refinement would likely tighten below 14.4px is
    discarded outright.
  - **Fix landed: relaxed direct-accept fallback, mirroring the guided-triangle
    seed pattern (cameraplatesolver.cpp ~12046-12068).** When
    `verifyBlindSeedCandidate` rejects `candidate`, but `candidate.valid` with
    `matchCount >= minBlindSeedMatches` and `rmsErrorPixels <=
    min(max(18, finalMatchRadius*0.75), finalMatchRadius*0.90)` (18px here) and
    `hasAcceptableBrightnessConsistency`, append `candidate` to `seeds` directly
    (not anchored/guidedTriangle — plain blind seed, so no scoring bonus).
    `synth-fisheye-013` now contributes 6 such seeds and PASSes — **but it was
    already PASSing pre-fix** (via `bright-pair-seeds`), so this is a
    no-op for the corpus total: still 15/50 PASS, **identical pass/fail set**
    (verified via full diff of per-case PASS/FAIL labels). Real suite
    re-confirmed 40/42, same 2 failures (wide-2 rows 15/16), after this change.
  - **Debugging gotcha discovered (recorded in
    [[camera-star-tests-run-procedure]]): `QString::arg()` chains with ≥10
    placeholders (i.e. reaching `%10`/`%11`) silently truncate the printed
    output on this MSVC/Qt toolchain** — `qDebug() << QStringLiteral("...%10
    ...%11").arg(...)×11` printed nothing past the text immediately before
    `%7`. Splitting into multiple statements of ≤9 placeholders each fixed it.
    Cost ~5 build/run cycles to isolate; not a `rmsErrorPixels`-is-NaN issue as
    first suspected (it wasn't NaN).
  - **New failure-mode taxonomy for the remaining 35/50 synthetic FAILs**
    (sampled `synth-fisheye-003`): these are NOT "no seed found" cases.
    `synth-fisheye-003` reports `solved=true, matched=23, rms=14.6` — internally
    self-consistent — but the accepted pose (Az=150.06,El=63.60,Roll=-71.92) is
    wildly wrong vs. truth (Az=40.82,El=32.67,Roll=173.25, same FoV=148.94).
    I.e. a **false-positive lock**: a wrong orientation that nonetheless finds
    ~23 plausible catalog matches at acceptable RMS (degenerate/aliased
    alignment, plausibly a roll/mirror or regional-pattern ambiguity in a dense
    fisheye field). This is a *different* problem from Critical Finding #1/#2
    (which were about the engine not contributing a seed at all) and 1c does
    not address it. wide-2 row15 (the other open exit-criterion item) is still
    blocked by Critical Finding #2 proper: `vectorQuadCodeMatches=0` for that
    image even after the 1c rework (its detection quads apparently still don't
    land within `codeEpsilon=0.02` of any of the 6496 catalog entries for that
    field/scale).
  - **Status (2026-06-13, end of session): 1c is real, validated, non-regressing
    progress (it does change behaviour broadly — codeMatches went from 5/50 to
    ~45/50 cases non-zero, and the relaxed-accept fallback now lets those
    contribute seeds), but it has not yet moved either headline metric.
    Both remaining exit-criteria gaps (wide-2 row15's codeMatches=0, and the
    false-positive-lock failure mode in the synthetic corpus) look like
    separate, deeper problems from what 1c targeted. Flagged for go/no-go with
    Jon before further investment in Phase 1.**
  - **Baseline correction (2026-06-14): synthetic corpus is 16/50=32%, not
    15/50=30%.** `grep -aE "^(PASS|FAIL)"` on a `-Encoding utf8` (UTF-8-with-BOM)
    log undercounts by 1 — the BOM prefixes `PASS`/`FAIL` on line 1 (case 001,
    actually PASS) so the regex anchor misses it. Open files with
    `encoding="utf-8-sig"` in Python (or strip the BOM) to get the correct
    count. This affects only the headline percentage, not any pass/fail-set
    diff (both old and new logs were undercounted identically).
  - **False-positive-lock pattern re-characterized: it's "missing labelled
    anchor star", driven by pose-refinement imprecision, not orientation
    lock.** Of the 34 corpus FAILs, 22 are `solved=true` ("FP" — plausible
    matchCount/rms but FAIL) and 12 are honest `solved=false`. Per-case CSV
    `azimuth/elevation/roll` columns are **not** in the same reference frame as
    `poseAz/El/Roll` (PASS cases also show 50-90° deltas), so they cannot be
    used as a pose-correctness oracle. The real, per-case rejection reason
    (printed under each PASS/FAIL line) is always:
    `missing: hip NNNNN` / `position mismatches: hip NNNNN missing labelled
    match near x=... y=...` — i.e. one of the CSV's 2-3 ground-truth anchor
    stars has no solved detection within the 24px position tolerance.
    Drilled into 3 FP cases (003, 004, 048) with
    `SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE=1` per-detection dumps: in 3/4
    sampled instances the "missing" anchor star **is** detected almost exactly
    at its expected pixel (0.16-0.5px off, bright: flux 700-900, comparable to
    successfully-matched neighbours) but is `solved=false` and **absent from
    `m_matchSummary` entirely** — i.e. under the accepted pose, no catalog
    entry projects within match radius of it at all, even though that same
    pose is self-consistent for its own 20-40 matched stars at rms 14-17px
    (close to the "~r/2 floor" described in `tightenNarrowFinalPass`'s comment,
    r=24). Conclusion: **final pose-refinement convergence precision**, not
    seed selection — codeEpsilon/vector-quad-threshold tuning is upstream of
    this and would not help. (4th sample, case 048's 2nd missing star, showed
    no strong nearby detection at all — a separate, lower-priority
    detection-pipeline issue.)
  - **Fix landed (2026-06-14): enable `tightenNarrowFinalPass` for wide-field/
    fisheye, with an added FOV-drift guard.** `tightenNarrowFinalPass`
    (cameraplatesolver.cpp ~20642) re-fits the pose on progressively tighter
    inlier cores (0.5x, 0.33x of `finalMatchRadius`) to escape the full-radius
    "coincidental association" RMS floor, but was gated to
    `isNarrowField(settings)` only — never ran for our ~166° fisheye cases
    (`solve.tightenFinalPass=0` always). Removed that gate. The existing
    accept-if-strictly-better check (≥90% match retention + rms not worse) was
    too permissive on its own: case 006 regressed PASS→FAIL because a small
    rms win (16.66→14.89) came with the FOV drifting 165.43°→179.99° (near the
    180° ceiling). Added a guard requiring
    `|tightened.fov - original.fov| <= max(5°, 5% of original.fov)`
    (skipped if `original.fov` is non-finite/≤0, e.g. no prior pose to compare
    against), rejecting drifted-but-locally-better fits.
    - **Results:** real 42-case suite **unchanged at 40/42** (same 2 failures,
      wide-2 rows 15/16, byte-identical). Synthetic fisheye mode-1 corpus
      **18/50 = 36%**, up from 16/50 = 32% (BOM-corrected baseline). Case 044
      flipped from a complete non-solve (`solved=false`) to a correct solve
      (rms=15.89, matched=24); case 040 improved its pose (rms 16.38→13.54);
      case 006 — which regressed without the FOV guard — is back to PASS with
      its original 165.43° FOV preserved. Net +2/50, zero regressions,
      build-verified (26/26 link).
    - Still 32/50 short of the ≥90% Phase 1 exit criterion: 14 honest
      `solved=false` failures plus ~20 remaining FP-pattern (`missing labelled
      match`) cases. The pose-refinement-precision direction looks more
      promising than further seed-selection tuning, but is a bigger
      investment — flagged for go/no-go with Jon on next steps.
- **Phase 2 — fov ladder, mode 0 (1 day).** Exit: wide-2 row 16 PASS;
  synthetic mode-0 ≥85%.
- **Phase 3 — perf & caching (½-1 day).** Index build ≤100ms/context, cached;
  suite total within noise of ~150s.
- **Phase 4 — full validation (½ day).** Real suite target 42/42 (hard floor:
  ≥40/42, zero regressions, every wide-1/3/4 row re-checked); rand2 148/150
  with guided cases bit-identical (inert by construction — verify anyway);
  synthetic-corpus FP audit comparing solved az/el/roll to truth, not just
  PASS. Update notes + memory.

Total ≈ 5-7 working days.

### Risks

- **Zenith/pole:** all geometry in `SkyVector` space (no az/el trig inside
  codes); the mode-1 row's el=90 placeholder must never be used as a cone
  centre — confirm the blind context loads the full hemisphere
  (`useWideBlindSeedRadius` path).
- **Horizon refraction at 165°:** already handled in catalog az/el (wide-1/3/4
  pass), but quads reaching <10° elevation compress — keep scale bands moderate
  and rely on LM refine.
- **False positives:** the wide weak-mode acceptance gates were tuned for the
  old seed flux. Gates stay unchanged, verify-K stays small, faLogOdds ranking
  is reused, and the phase-4 FP audit checks pose truth, not just PASS.
- **Fisheye-edge centroids:** PSF distortion biases detection centres near the
  edge; prefer inner-image detection quads via the existing region-stratified
  `selectDetectionIndicesForBlindSignatures` (:10812 pattern).
- **Ordering:** legacy blind path remains primary for currently-passing wide
  rows until phase 4 proves the new path superior; any ordering swap is its own
  validated change.

### Future follow-ons (each its own go/no-go)

- **Guided-path adoption:** the quad index as a roll-invariant generator
  replacing the anchor-pair × 36-roll sweep and triangle search — large speed
  win and structurally retires the wrong-roll-alias class, but
  behaviour-affecting on all 40 passing cases.
- **Prebuilt deep index** (healpix-tiled, ra/dec frame, mag 12-15, on-disk) for
  blind *narrow*-field solving — the full astrometry.net equivalent; only
  worthwhile if blind narrow solving becomes a product goal.

## Structural note (not started)

`CameraPlateSolver::SolverContext` is one ~18.5k-line inline class body (lines ~63 to
~18640) — the entire solver in a single translation unit because the class is declared
in the `.cpp`. Splitting it (private header + themed TUs: catalog I/O, projection,
search, acceptance, Siril network, orchestration) is a behaviour-preserving structural
task, deferred.

## Seed-engine unification — ABANDONED after measure-first probes (2026-06-16)

**Outcome: do NOT unify the seed engines onto one geometric-hash core.** The plan below was
explored through P1 (parameterise the quad engine — done, validated) and then two measure-first
probes that refuted the single-core premise. P1 was reverted; the engines stay as they are.

**Why (the probes):**
1. *Can the existing n=4 vector engine serve the narrow-guided regime?* Ran it additively in
   narrow-guided (`SDRANGEL_CAMERA_PLATE_SOLVER_NARROW_QUAD_PROBE`, since reverted): recovered
   **0 of 23** brighttriangle-unique RAND2 cases — `vectorQuadCodeMatches=0`.
2. *Is that because it's quads, or the bright-pool restriction?* (good question — tested it.)
   Swept the pool sizes on a narrow case: codeMatches stayed 0 at catPool 24/40/80 and only
   reached **4** at catPool=150 — where the catalogue index exploded to **20.3 million** quads
   (C(k,4) is quartic) and it *still* didn't solve. So quads *can* match (not a geometry limit),
   but the blind bright-pool mechanism doesn't scale to a deep field: it needs the SAME 4 stars
   in both top-K pools, and "brightest-K by detection flux" != "brightest-K by catalogue
   magnitude" once the catalogue is far deeper than what's detected (narrow telescope deep
   exposure: 302 detections vs a 256k-star Gaia mag-13 catalogue).

**The structural conclusion:** the guided-narrow and blind-wide regimes need *different* seeding
strategies — direction-project-and-match (uses the trusted Az/El; what brightTriangle does) vs
blind quad-hash (all-sky; what vectorQuad does). brightPair and brightTriangle exist precisely
because the blind vector-quad core returns codeMatches=0 in their regimes (fisheye distortion /
deep-narrow pool mismatch). Forcing them onto the vector core means either an infeasible pool, a
big prebuilt-index project, or reimplementing the direction-using approach they already are. The
real, validated seed-layer simplification was deleting `blindquad` (zero marginal). The remaining
four engines are regime-specialised, not redundant — KEEP THEM.

*(The plan below is retained for reference / in case a prebuilt deep index is ever pursued.)*

### Motivation
After the 2026-06-16 ablation (across REAL + fisheye + wide + narrow corpora, see
`test/seed-ablation.ps1`) the seed layer is: `buildVectorQuadBlindSeeds` (quad, blind),
`buildBrightPairSeeds` (pair, wide-blind), `buildBrightGuidedAnchorTriangleSeeds` +
`buildBrightGuidedTriangleSeeds` (triangle, guided), with `buildBlindTriangleSeeds`
retained only as a shared subroutine. `blindquad` was deleted (zero marginal). The
remaining four are NOT redundant — each occupies a distinct cell of two axes:

| engine | primitive N | search mode | why it can't just be quads |
| --- | --- | --- | --- |
| vectorQuadBlind | 4 | blind (all-sky) | — (the modern baseline) |
| brightPair | 2 | blind, wide | quad vector codes break under fisheye distortion (measured `codeMatches=0`); a 2-star angular code + magnitude prior survives |
| brightGuided(Anchor)Triangle | 3 | guided (cone around trusted Az/El) | quads would re-solve all-sky and discard the direction prior |

So the engines are points in **{N: 2,3,4} × {mode: guided, blind}**, differing also in
**pool** (bright-K vs all) and **FoV** (known/scale-aware vs swept/scale-free). The goal:
collapse them onto ONE parameterized geometric-hash engine, removing duplicated code and
threshold surface without losing any regime. This does NOT touch the acceptance layer
(the ~145 reject gates) — that is a separate consolidation.

### Reusable core that already exists (from the vector-quad work)
- `unprojectPixelToVector` (cameraplatesolver.cpp ~3459) — pixel → camera-frame ray.
- `buildVectorQuadCode` (~4227) + `struct QuadVectorCode` (~4211) — rotation-invariant
  code via "most-separated pair = basis, gnomonic-project at centroid". Currently fixed
  at `std::array<SkyVector,4>` but the construction generalizes to any N.
- `CatalogQuadCodeIndex` / `CatalogQuadCodeEntry` (~7138) with `queryEpsilonBall` and a
  `brightPoolLimit` param — the kd-tree code index.
- N-vector Wahba pose solver (completed task #1) — already takes N correspondences.

### Target design: one `buildGeometricHashSeeds(params)` engine
Parameter struct:
- `int n` ∈ {2,3,4} — group size.
- `int catalogBrightPool`, `int detectionBrightPool` — 0 = all, else brightest-K.
- `enum mode { Blind, Guided }` — Guided restricts the catalog index (and detection
  candidates) to a cone around the trusted Az/El; Blind uses the all-sky index.
- `bool fovKnown` — scale-aware code (uses the AB great-circle angle) vs scale-free +
  external FoV sweep.
- `bool useMagnitudePrior` — weight/verify matches by brightness consistency (the term
  that makes low-N discriminative; currently brightPair-only).
- existing `codeEpsilon`, `maxVerified`, `minMatches`.

Then the current engines become presets:
- vectorQuad = {n:4, blind, bright-pool 24/16, fovKnown or swept, mag:off}
- brightPair = {n:2, blind, bright-pool, fovKnown, mag:ON}
- brightGuidedTriangle = {n:3, guided, bright-pool, mag:ON}
…and new useful combos (guided quad, blind triangle) become free.

### Key technical work
1. **Generalize the code function** `buildVectorQuadCode` → `buildVectorCode(span<SkyVector,n>)`.
   - n=4: current quad code (must stay byte-identical for the parity gate).
   - n=3: similarity-invariant triangle code (most-separated pair as basis, 1 remaining
     projected point → 2-D code; or the proven ratio code).
   - n=2: code = AB great-circle angle only (1-D) — geometrically ambiguous, so REQUIRES
     the magnitude prior to be discriminative.
2. **Generalize the index** to store n-dependent code dimensionality.
3. **Magnitude prior** as a first-class term in match scoring + verification — this is the
   single most important new piece: it is what lets n=2/3 disambiguate and what makes the
   engine distortion-robust at low n (the measured reason brightPair beats quads on fisheye).
4. **Guided candidate restriction** — build the index over (or filter to) a cone around the
   trusted direction; subsumes the bright-anchor-triangle behaviour and is far cheaper than
   all-sky in guided mode.
5. **Regime → params policy** (replaces the current hard-coded engine selection in
   `searchBestPose` ~19340-19460): wide/fisheye-blind → try n=2 (robust) then escalate;
   narrow-guided → n=3 guided; FoV-known low-distortion → n=4. Keep the FoV sweep wrapper
   for mode 0.

### Migration — incremental, each phase gated on the 434-case baseline + ablation parity
Baseline to hold: REAL 48 · FISHEYE 56 · WIDE 27 · RAND 83 · RAND2 148 (and per-engine
marginal-contribution parity via `seed-ablation.ps1`).
- **P0+P1 [DONE then REVERTED 2026-06-16]** Parameterised the quad engine into
  `GeometricHashSeedParams` + `buildGeometricHashSeeds`; validated exact parity (REAL 48 / FISHEYE 56 /
  WIDE 27 / RAND 83 / RAND2 148). REVERTED after the P2 probes refuted the single-core premise (see
  Outcome above) — left aspirational scaffolding (a params struct only ever holding n=4) with no
  purpose. Seed layer returned to the clean blindquad-deleted state.
- **P2+ NOT PURSUED** — see Outcome above.
- **P2** Add n=3 + guided restriction + magnitude prior; replace the bright-guided-triangle
  path. Gate: RAND/RAND2 unchanged (esp. the 24 brighttriangle-unique cases).
- **P3** Add n=2 + magnitude prior; replace brightPair. Gate: FISHEYE unchanged (esp. the
  ~10 brightPair-unique cases — the distortion-robustness risk; watch `codeMatches`/verify).
- **P4** Point all seed call sites at the unified engine via the regime policy; delete the
  four old functions and the standalone use of `buildBlindTriangleSeeds`. Full-suite +
  ablation parity.
- **P5** Fold the per-engine threshold constants into the param struct (the seed-layer
  portion of the ~93-constant reduction).

### Risks / guardrails
- **P3 (pairs) is the highest risk:** brightPair's value is distortion-robustness, not
  geometry; the unified n=2 path must keep the magnitude prior doing the disambiguation or
  it will regress fisheye (the `codeMatches=0` lesson). Validate FISHEYE every step.
- **P2 must not perturb narrow** (RAND2 is sensitive; 24 cases ride on guided triangles).
- Keep each old engine behind a flag until its unified replacement reaches parity →
  per-phase rollback.
- Cross-build determinism still applies — prefer match-count / discrete criteria over
  ULP-sensitive continuous scores in the new verification (see the wide-7/8/9 history).

### Payoff vs non-goals
Payoff: 4 engines + 1 subroutine → 1 parameterized engine; one code function, one index,
one verification path; new regime combos for free; smaller seed-layer tuning surface.
Non-goal: this does not simplify the acceptance gates (downstream); that is the separate
"single faLogOdds-based accept" idea.

## WS0 — threshold inventory + sensitivity sweep (2026-06-18, in the worktree)

**WS0a (inventory).** Of the 87 `k*` declarations in `cameraplatesolver.cpp`, **34 are genuine
decision/tuning thresholds** (the overfit surface); the rest are infrastructure (Siril catalog
I/O + healpix sizing ~389-422, parallelism/cache limits 539-573/5750/5819, URLs/paths, `kPi`,
debug log flags, `kVisibleAltitudeFloor`, catalog-query radii). NB the 34 named constants are
**not** the ~145 bare-literal accept/reject sites — those inline literals are a deeper,
un-named overfit surface this sweep does not touch.

**WS0b (sensitivity sweep, REAL corpus).** Harness `test/ksweep.ps1` (untracked local tooling,
like seed-ablation.ps1): perturbs each of the 34 constants ±10% (±1 for small int caps) in-source,
incrementally rebuilds (~12s/rebuild) and re-runs the REAL suite vs the 48/48 baseline; results in
`test/ksweep-results.csv`. Zero-refactor (pristine backup restored after every iteration; tree
verified clean after). Baseline is all-pass, so any flip = a real case going PASS→FAIL, exposing a
margin. Two harness gotchas hit + fixed (recorded in [[camera-star-tests-run-procedure]]): a `.ps1`
em-dash that PS5.1 reads (no-BOM→ANSI) as U+201D `”` = a string delimiter (parser desync); and
`$ErrorActionPreference='Stop'` turning vcvars/cmake/exe **native stderr** into a terminating
NativeCommandError (use `Continue` + explicit exe-mtime/matchCount checks).

**Result: 28/34 constants are robust** (≥10% margin both directions, zero flips) — corroborates the
2026-06-16 metamorphic/negative finding that the solver is not broadly overfit at the named-constant
level. **6 constants are tight in ≥1 direction:**

| constant | direction | #flips | case(s) |
|---|---|---|---|
| kMaxDetectionsForSolve | −10 (96→86) | 1 | galaxy-m51-1 (m51@14) |
| kGeometricSupportCap | −10 (8→7) | 1 | galaxy-m101-1 (m101@15) |
| kRetrySearchRadiusDegrees | −10 (12→10.8) | 1 | stars-narrow-7 |
| kRetrySearchRadiusDegrees | +10 (12→13.2) | 2 | stars-narrow-3, cluster-m3-1 |
| kWideFovBrightFirstPassMaxMagnitude | +10 (5→5.5) | 2 | stars-wide-1, stars-wide-2 |
| kNarrowGuidedBrightCatalogMaxMagnitude | +10 (12→13.2) | 2 | galaxy-m101-1, pollux |

**Reading the map:**
- **Cleanest fitted-to-one-image smell** (exactly one case, one-sided): `kMaxDetectionsForSolve` (−)
  and `kGeometricSupportCap` (−). Each guards a single fragile case.
- **Most fragile / most over-tuned:** `kRetrySearchRadiusDegrees` — flips in **both** directions;
  12.0 sits in a narrow valley wedged between stars-narrow-7 (needs ≥~11) and
  stars-narrow-3 + cluster-m3-1 (need ≤~13). Three distinct real cases ride on this one constant.
- The two **catalog-magnitude caps** are one-sided (raising loses cases): principled asymmetry
  (deeper catalog → more faint contamination/aliasing), so the +flip is expected behaviour, but the
  upward headroom is <10%.
- The knife-edge cases (m51@14, m101@15, narrow-3, narrow-7) are exactly the historically hard-won
  solves — they needed tuning to pass, so they sit near boundaries. Expected, not alarming.
- **Caveat:** REAL-only — cannot see flips that manifest only in the synthetic/wide/fisheye corpora,
  and does not probe the ~145 bare-literal accept sites. "28/34 robust" is therefore a partial map
  that *localizes* fragility, not a clean bill of health for the whole tuning surface.

**WS3 implication:** the acceptance-layer consolidation should prioritise the gates fed by the 6
tight constants — above all `kRetrySearchRadiusDegrees` (the retry/recenter search radius, 3 cases
balanced on it) and the two catalog-magnitude caps — replacing the hand-tuned boundary with a
principled criterion (the `poseFalseAlarmLogOdds`-driven gate). To extend the map before WS3, re-run
`ksweep.ps1` pointed at RAND2/FISHEYE (change `$csv`), and add a pass over the bare-literal accept
sites (the un-named surface).

## WS1a — harness links the shipped solver object (2026-06-18, DONE)

**Change:** `cameraplatesolver.cpp` is now compiled exactly ONCE into a static library
`camera_platesolver` (`plugins/feature/camera/CMakeLists.txt`) with `INTERPROCEDURAL_OPTIMIZATION OFF`.
Both the `featurecamera` plugin and the `featurecamera_star_tests` harness link that same object
(`test/CMakeLists.txt` no longer compiles `../cameraplatesolver.cpp`). The harness now exercises
byte-identical solver machine code to the shipped GUI plugin — the structural fix for the cross-build
(DLL-vs-EXE) ULP divergence in the wide-7/8/9 saga, replacing the "two independent compilations of the
same TU" with "one object linked into both binaries."

**Why it's safe (verified by grep, not assumed):** `cameraplatesolver.cpp` has zero feature-define
`#ifdef` (no `CAMERA_OPENCV_CUDA_*`/FFmpeg) and never references the CUDA-layout-sensitive types
(`CameraPipelineFrame`, `CameraStarDetector`, `cv::cuda`/`GpuMat` — all grep-count 0). Its public
interface — `solve(const CameraSettings&, QSize, QDateTime, QVector<CameraPipelineStarDetection>&)
-> CameraPlateSolveResult` — uses only ABI-stable types: `CameraPipelineStarDetection`
(camerapipelineframe.h lines 69-92, NO CUDA `#ifdef`; only the unrelated `CameraPipelineFrame` struct
at 168+ is CUDA-conditional) and `CameraSettings` (no CUDA `#ifdef`). So one compilation is correct
for both consumers regardless of their own CUDA/FFmpeg define sets; no ODR hazard at the boundary.

**Why LTCG-off is essential:** with `/GL` on, the lib's object is IL and final codegen happens at
*each* consumer's link → DLL and EXE would re-diverge. `/GL` off makes the `.obj` final machine code,
so identical bytes land in both. (`CMAKE_INTERPROCEDURAL_OPTIMIZATION` is global ON via
`cmake/Modules/CompilerOptions.cmake:6`; the per-target OFF property overrides it. Trig is CRT calls
to the same shared ucrtbase in both binaries, so results match once the surrounding code is identical.)

**Validated:** configure clean; static lib + plugin DLL + harness all build (no LTCG/LNK link
warnings); REAL suite **48/48, zero regressions** — the LTCG-off solver codegen did not flip any case.
Files touched: `plugins/feature/camera/CMakeLists.txt`, `plugins/feature/camera/test/CMakeLists.txt`
(both tracked; uncommitted pending Jon's go-ahead).

**GUI CONFIRMED for wide-7 (2026-06-20).** Rebuilt sdrangel.exe + featurecamera.dll in the worktree
(full build, 121 plugins; a near-empty single-plugin build crashed on launch). wide-7 mode 3 now
**solves in the GUI** at Az=52 / El=88 / Roll=94 / FoV=159 (log winner Az=52.2182 El=88.819 Roll=94.37
FoV=159.847 Cx=-41.41 Cy=-34.95 K1=-0.1033, 94 matches, RMS 0.87, 8/8 bright, BrightMagErr=0) — the
correct off-centre-pp pose, identical to the harness. Previously the GUI landed in a wrong basin (best
on-direction RMS ~16.5 > gate) and rejected. WS1a's shared-object fix is therefore **empirically
confirmed: GUI == harness.**

**ALL THREE CONFIRMED in the GUI (2026-06-20).** wide-7 = Az 52/El 88/Roll 94/FoV 159 (221 matched),
wide-8 = 52/88/94/159 (208 matched), wide-9 = 53/88/94/159 (206 matched) — all the correct pose, all
previously GUI-failing. **WS1 (cross-build ULP divergence) is fully validated end-to-end.** (GUI match
counts run a little above the harness's 165-206 because the GUI's live-detection set + the hot-pixel/
label-recovery passes label a few more real stars; the pose is the same.)

**Now unblocked — WS1 follow-on:** the wide-7/8/9 band-aids (seed-anchored grid, Az/El pin, clamped
free-pp polish) were added to force GUI==harness when the two compiled the solver independently. With
WS1a making that identity structural, they are candidate simplifications — remove one at a time against
the full suite + a GUI re-test of wide-7/8/9.

**Not done — WS1b** (decision-boundary margins + deterministic tie-breaks): the second half of WS1, a
separate moderate-risk change that generalises the match-count-grid lesson. Not required to close the
independent-compilation divergence class, which WS1a handles structurally.

## WS3 — acceptance-gate ablation, pass 1 (2026-06-19)

**Measure-first finding (faLogOdds is NOT a viable single gate).** Extracted `verify.faLogOddsMilli`
for every passing REAL case: true-accept faLogOdds spans **1.5 → 1204** (stars-narrow-3 = 1.5 total /
0.05 per match; stars-wide-1 = 10.4; dense m31/c11/cluster-m7 = 476/769/798; max 1204). The notes'
documented wrong-pose rejections score 45–143, so true accepts (down to 1.5) overlap rejects (up to
143) — any single faLogOdds threshold either rejects sparse/faint correct solves or accepts
contaminated poses. The plan's literal WS3 step-1 ("make faLogOdds the primary gate replacing the 145
sites") is therefore **not viable** with the current per-match-sum metric (it scores *whether* bright
stars match, not their geometric configuration, and under-credits sparse/faint correct solves). WS3
reframed to **gate ablation** (find & delete redundant gates), the same method that retired blindquad.

**Instrumentation (kept, inert by default):** `gateAblationDisabled(token)` helper +
`SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_GATE` env hook (mirrors the seed-ablation pattern), wiring the
7 direction-seed acceptance gates in `directionSeedAcceptanceFor` so each can be neutralised to its
permissive value. Confirmed inert: REAL 48/48 with the env unset. Harness: `test/gateablate.ps1`
(env-driven, no rebuilds); results `test/gateablate-results.csv`.

**Pass-1 result (baseline REAL 48 · RAND2 148 · 6 obvious-garbage negatives):**

| gate (disabled → permissive) | REAL | RAND2 | neg FP | verdict |
|---|---|---|---|---|
| overwhelmingFaint | 48 | 148 | – | inert (no deciding vote) |
| sparsePair | 48 | 148 | – | inert |
| highConfSparseAnchors | 48 | 148 | – | inert |
| fov | 48 | 148 | – | inert |
| residual | 48 | 148 | – | inert |
| **weakBrightSupport** | 43 (−5) | 120 (−28) | – | **load-bearing** |
| **brightnessConsistency** | 47 (−ngc-2403) | 148 | **neg-blobs** | **load-bearing** |

- **weakBrightSupport is overloaded** — it is also the *trigger* for the dense-match polish +
  bright-anchor rescue (their precondition is `weakBrightSupport==true`). Forcing it false disables
  those rescues, collapsing 33 cases (narrow-1/3/4, m51-2, c11 + 28 RAND2). It cannot be simplified
  without first untangling the rescue trigger from the reject signal.
- **brightnessConsistency** forced-permissive prematurely accepts a *wrong* pose for ngc-2403
  (bypassing the rescue that finds the right one) and lets garbage (neg-blobs) through — a genuine
  false-positive protector.
- **5 of 7 gates cast no deciding vote** on the trustworthy corpus → strong support for the WS3
  over-determination thesis (much hand-tuned acceptance is redundant). **But not yet safe to delete:**
  the negative suite is only 6 obvious-garbage cases (thin false-positive coverage) and there is no
  fisheye/wide-blind regime in this pass. The 3 accept-bypasses (overwhelmingFaint/sparsePair/
  highConfSparseAnchors) can only tighten acceptance (no FP risk) but were built for faint/sparse
  fields; the 2 reject gates (fov/residual) being deletable rests on the thin negative coverage.

**Pass-2 (2026-06-19): strengthened coverage reclassified `residual`.** Re-ablated the 5 inert gates
against FISHEYE-mode4 (guided wide-fisheye, baseline 42/50) + 2 near-boundary negatives (real narrow
fields pointed 30 deg off truth, `test/star-tests-nearboundary-neg.csv`, both `solved=false` at
baseline) + the 6 garbage negatives. Harness `test/gateablate2.ps1`; results
`test/gateablate2-results.csv`. Result: **`residual` is load-bearing** — disabling it makes the
near-boundary negative `stars-narrow-6` a false positive (accepted at the wrong 30-deg-off pose), and
adds a fisheye false-solve (FISH4 solvedTrue 48->49). The garbage-only negatives of pass 1 missed
this; one plausible-but-wrong negative caught it. The other 4 (overwhelmingFaint, sparsePair,
highConfSparseAnchors, fov) stayed fully inert across FISH4 + all 8 negatives.

Lesson: "inert on corpus" is fragile evidence — a single near-boundary negative flipped `residual`.
This raises the bar for the remaining 4 and means a near-boundary negative *suite* (not 2 cases) is
the right long-term FP oracle.

**Side finding (noted, not chased):** the near-boundary probe showed the solver accepts a *wrong*
pose for rich bright fields at a 30-deg seed error (narrow-8 solved=true at the offset az, caught by
the harness as FAIL; narrow-9 solved=true and slipped through as PASS via a weak named-anchor oracle).
This is the documented coincidental-contamination class at an unrealistic seed error (gates are
calibrated for ~1 deg); narrow-9's PASS also hints some corpus PASS verdicts are weakly validated.

**Joint-disable check (2026-06-19):** disabling all 4 still-inert gates *simultaneously*
(`overwhelmingFaint,sparsePair,highConfSparseAnchors,fov`) across REAL + RAND2 + FISHEYE-mode4 + all 8
negatives was **fully inert** (REAL 48, RAND2 148, FISH4 42, zero new false positives) — so the 3
accept-bypasses are *jointly* redundant (no faint field rides on their OR), and even `fov` casts no
deciding vote on this coverage.

**Deletion landed (2026-06-19):** removed the `overwhelmingFaint`, `sparsePair`, and
`highConfSparseAnchors` accept-bypass branches from `directionSeedAcceptanceFor`, and deleted the
now-orphaned `hasOverwhelmingFaintGuidedSupport` member function (it was used only in that lambda;
`isAcceptableSparseGuidedPairFinalPass` and `hasHighConfidenceSparseGuidedAnchors` are retained — they
are still used in the rescue/recenter/roll-alias paths). Re-validated behaviour-neutral: **REAL 48 ·
RAND2 148 · FISHEYE-mode4 42 · zero false positives** across near-boundary + garbage negatives.

**Kept (load-bearing):** `weakBrightSupport` (also the rescue/polish trigger), `brightnessConsistency`
(rejects wrong poses + garbage), `residual` (the near-boundary FP protector). **Kept (deferred):**
`fov` — a *reject* gate; the `residual` lesson (one near-boundary negative reclassified it) means
reject gates need failure-mode-specific adversarial coverage before removal, and the current negatives
do not vary FoV. Removing `fov` awaits a wrong-FoV negative suite.

**Net WS3 pass-1 result:** the direction-seed acceptance decision dropped from 7 gate terms to 4, with
one helper function deleted and no behaviour change on any corpus. The `gateAblationDisabled` hook +
`SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_GATE` env var are retained as standing infrastructure (inert by
default) for the next ablation pass (roll-alias / blind / elevation acceptance paths, and `fov` once a
wrong-FoV negative suite exists). Harnesses: `test/gateablate.ps1`, `test/gateablate2.ps1`,
`test/star-tests-nearboundary-neg.csv` (untracked local tooling, like the seed-ablation scripts).

## WS3 pass-2 — roll-alias / bright-support-bypass / elevation gates (2026-06-20)

Instrumented 5 more acceptance gates with `gateAblationDisabled` hooks: `rollAlias` (the
`hasCompetitiveRollAlias` ambiguous-roll *reject* only — the adopt-better-alias path is preserved),
the two bright-support bypass waivers `highConfTriangle` / `strongDense`, the `namedAnchorCert`
certificate, and `elevationSeed`. Ablated across REAL + RAND2 + FISHEYE-mode4 + near-boundary +
garbage negatives (harnesses `test/gateablate3.ps1` un-guarded, then `test/gateablate4.ps1` with a
per-corpus timeout guard after the rollAlias hang below).

Result:

| gate (disabled) | effect | verdict |
|---|---|---|
| `rollAlias` | REAL 48->47 (a wrong roll accepted = FP) **and a RAND2 case went into a non-terminating solve** | **load-bearing** |
| `strongDense` | REAL 48->47 (drops galaxy-m31) | **load-bearing** |
| `namedAnchorCert` | REAL 48->47 (drops stars-narrow-7) | **load-bearing** |
| `elevationSeed` | inert (REAL 48, FISH4 42, negatives clean) | **deferred** (reject gate; no mode-2 wrong-pose negative covers it) |
| `highConfTriangle` | inert across REAL 48, RAND2 148, FISH4 42, all negatives (even with `strongDense` active) | **removed** |

**`rollAlias` is doubly load-bearing:** beyond rejecting a wrong-roll FP on a REAL case, disabling it
sent a RAND2 case into a non-terminating solve — the roll-alias reject also *bounds* the expensive
roll-recovery/retry machinery, and the harness's 120 s per-case timeout did not catch it. (Latent
robustness gap: that retry path can run unbounded; only reachable with the gate artificially off.)
This is why `gateablate4.ps1` added a per-corpus timeout guard.

**Deletion landed:** removed the `hasHighConfidenceGuidedTriangleSupport` term from the bright-support
bypass in `hasWeakNarrowGuidedBrightSupport` (the OR with `hasStrongDenseNarrowGuidedFinalPass`).
`strongDense` is retained (load-bearing). The `hasHighConfidenceGuidedTriangleSupport` function stays
(still used by `isAcceptableSparseGuidedPairFinalPass`). The other 4 gate hooks are retained as
standing infrastructure (inert by default). Re-validated behaviour-neutral: **REAL 48 · RAND2 148 ·
FISHEYE-mode4 42 · zero false positives.**

**WS3 to date:** pass-1 removed 3 accept-bypass branches + 1 helper from the direction-seed decision;
pass-2 removed 1 more bypass term. The substantive gates (`weakBrightSupport`+rescue trigger,
`brightnessConsistency`, `residual`, `rollAlias`, `strongDense`, `namedAnchorCert`) all proved
load-bearing. Remaining ablation targets: `fov` and `elevationSeed` (both need failure-mode-specific
negatives), and the blind/fov acceptance path (mode 0/1, wide-2 regime). Harness `test/gateablate3.ps1`
+ `test/gateablate4.ps1` (untracked).

## WS3 pass-3 — bright-support internals firing analysis (2026-06-21)

`hasWeakNarrowGuidedBrightSupport` is a ~13-sub-condition thicket. Rather than blindly ablate each
(high-risk: it is the load-bearing rescue trigger), used its own `logBrightSupportDecision` logging
(`SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE=1` + `QT_LOGGING_RULES=*.debug=true`) to count which
sub-conditions actually fire, across REAL + RAND2 + FISHEYE-mode4. Findings:

- **The gate is narrow-only.** It early-returns for `!isNarrowField` (fov > 5 deg), so **FISHEYE-mode4
  (wide) never enters it** (0 log lines from 50 cases). Wide-fisheye-guided acceptance runs entirely
  through `hasAcceptableGuidedFinalBrightnessConsistency` + the residual gates instead.
- **Dominant firing sub-conditions** (REAL+RAND2): `poorNoRollSeedRadialSupport` (by far the most),
  `brightProjected>=10 & matched<5`, `seedProjectedBright>=4 unmatched`, `brightDetections>=12 &
  matched<3`, `brightProjected>=4 & matched<2`; plus the general-compound `weakBrightMagnitude` /
  `weakSeedRadial` on REAL. These are load-bearing.
- **Provably-dead removed:** `strongBrightDetectionSupportWideFisheye` (the wide waiver on
  `weakBrightProjected`) -- always false, because reaching it requires `isNarrowField` (fov<=5) and a
  direction-seeded (non-blind) solve, while `isWidePlateSolveContext` needs fov>=30 or blind. The two
  predicates are mutually exclusive here. The notes' 2026-06-15 wide-fisheye hardening lives in the
  `hasAcceptableGuidedFinalBrightnessConsistency` copy; this one was dead (likely since the
  `!isNarrowField` early-return). Removed; behaviour-neutral (REAL 48, FISH4 42).
- **Never-fired-but-NOT-removed** (kept, deliberately): the `useSeedProjectedBrightGate`-gated
  branches (that gate is off for every corpus case, but on for other configs), the magnitude-cap
  branches (`brightDetectionMagnitudeError>2.25/2.35`), `brightCatalogShapeMismatch`, the rarer
  `brightProjected>=6 & matched<3` overlap, and the general `seedProjectedBright>=6/>=10` floors. Each
  was added for a specific documented case and "never fires on this corpus" is the same fragile
  signal that `residual` flipped on -- removing them needs config-specific coverage (e.g. a
  `useSeedProjectedBrightGate`-on corpus), not available here.

**WS3 status: gate pruning at its safe floor.** Removed across 3 passes: 3 accept-bypass branches +
1 helper (pass-1), 1 bypass term (pass-2), 1 dead waiver (pass-3). The substantive gates
(`weakBrightSupport` + its dominant sub-conditions, `brightnessConsistency`, `residual`, `rollAlias`,
`strongDense`, `namedAnchorCert`) are all load-bearing. Remaining (deferred, need failure-mode-specific
coverage, not blind tuning): `fov` + `elevationSeed` (cheap narrow/mode-2 safety checks, structurally
inert on this corpus), the `useSeedProjectedBrightGate` branches, and the blind/fov accept path
(mode 0/1). Plus the latent unbounded roll-recovery retry (only reachable with `rollAlias` disabled).

## WS2 — near-zenith pose: rotation-vector LM (2026-06-21, flag-gated, WIP)

**Root cause (confirmed):** `createProjector` builds `right = {cos az, -sin az, 0}` then rolls it about
`center`. At zenith (el->90, center->vertical) a change in **az** and a change in **roll** are the same
rotation about the vertical, so their Jacobian columns are parallel -> the LM normal matrix's Az-Roll
block is singular -> the damped solve drifts along the degenerate valley and which basin it lands in
flips with ULP noise (the wide-7/8/9 saga; currently masked by the Az/El-pin + match-count-grid +
clamped-pp band-aids).

**Implemented (approach: localized rotation-vector LM):** env flag
`SDRANGEL_CAMERA_PLATE_SOLVER_ROTVEC_LM` (default OFF -> the legacy az/el/roll coordinate path is
byte-identical when unset). When on, the LM's orientation deltas (the Az/El/Roll params) are applied as
small rotations about the **camera-frame** axes (yaw about `up`, pitch about `right`, roll about
`center`) instead of az/el/roll coordinate additions. The camera basis is always orthonormal, so the
orientation Jacobian stays well-conditioned even at zenith. Helpers (next to the LM):
`lmBasisFromAzElRoll` / `lmAzElRollFromBasis` (exact inverse of createProjector's convention) /
`lmRotateOrientationCameraFrame`; the change is localized to `addPlateSolveLmParameterDelta` (single
choke point used by both the FD Jacobian and the update). **Key fix:** in rot-vec mode the FD Jacobian
must be taken w.r.t. the *rotation angle applied* (`appliedStep = step`), NOT the resulting az/el/roll
coordinate change -- the latter is non-linear and degenerates to ~0 near zenith (it collapsed the
refinement, the first bug: REAL 48->38). With that fix the update (which applies the solved delta as a
rotation) and the Jacobian use consistent units.

**Validation:**
- flag OFF: REAL 48 (verified byte-identical legacy path -> committed solver is unaffected).
- flag ON: **REAL 48 (neutral)**; RAND2 110/110 passing before an overnight environmental reap (full
  150 run still pending); **FISHEYE-mode4 40 vs 42** -- loses `synth-fisheye-039` + `synth-fisheye-044`,
  both documented borderline/hard fisheye cases at the rms acceptance boundary that the rot-vec LM's
  slightly-different convergence tips over. Not a systematic fisheye break (REAL wide-7/8/9 still pass
  with rot-vec on).

**Status: sound foundation, NOT yet drop-in-neutral.** REAL-neutral and singularity-free, but costs 2
marginal synthetic-fisheye cases, so it cannot become the default until those are resolved/accepted.
Default-OFF keeps the committed behaviour safe at 48/48.

**Remaining to land WS2 (make rot-vec the default + reap the payoff):**
1. Resolve or accept synth-fisheye-039/044 (check whether they fail by a tiny rms margin -- likely
   boundary noise, not a rot-vec defect).
2. Complete the RAND2 (148) validation with rot-vec on (run it in chunks / poll -- the full ~20 min run
   keeps getting environmentally reaped overnight).
3. The actual payoff: with rot-vec on, drop the wide-fisheye Az/El pin (`lockSeedDirection =
   isWidePlateSolveContext`) and GUI-re-test wide-7/8/9 -- the LM should now converge deterministically
   without pinning. Then the seed-anchored-grid / clamped-pp band-aids can likely follow. Measure the
   perf delta (rollAliasCheck/rollRecovery stay -- they serve deep-field roll, not just zenith).

**Process note:** long (~20 min) `run_in_background` corpus runs get reaped by the environment (overnight
sleep / idle) -- run fast corpora (REAL ~3 min, FISH4 ~30 s) foreground (synchronous, no reap) and
poll/chunk the long ones instead of passively waiting on one detached run.

### WS2 payoff — Az/El pin retired under rot-vec (2026-06-21)

Gated the wide-fisheye Az/El pin on `!rotVecLmEnabled()` (`lockSeedDirection =
isWidePlateSolveContext(settings) && !rotVecLmEnabled()`, cameraplatesolver.cpp ~21828): with the
rotation-vector LM the orientation is well-conditioned at zenith, so the pin (a band-aid for the
Az<->Roll ULP basin flip) is dropped on the rot-vec path; the legacy (rot-vec off) path keeps it.
Validated (rot-vec ON, pin removed): **REAL 48/48 with wide-7/8/9 solving at the correct sub-pixel
poses with Az/El FREE** -- wide-7 Az=52.25/El=88.82/Roll=94.40/rms0.95/221, wide-8
52.90/88.80/95.06/rms1.05/207, wide-9 53.50/88.80/94.77/rms0.95/205. FISH4 unchanged at 40 (the pin
never affected the 039/044 marginals). Since WS1a makes the GUI link the same solver object, this
harness result is the GUI's behaviour -- the pin is functionally retired. rot-vec stays default-OFF
until the default-flip (pending: accept/resolve FISH4 039/044, finish RAND2(148) ON). Once flipped,
the seed-anchored-grid / clamped-pp band-aids can likely follow the same way.

### WS2 rot-vec ON — full validation closed (2026-06-21)

Finished the RAND2 run that kept getting reaped, via chunked foreground (3x50 rows, ~6-8 min each,
within the tool window -- the fixed process). Result: **RAND2 rot-vec ON = 148/150, identical fail set
to baseline (a-005, c-009 hard tail)**. So rot-vec ON is **neutral on both trustworthy corpora (REAL
48/48 + RAND2 148/150)**; the only delta is FISHEYE-mode4 40 vs 42 (039 = a corrected OFF weak-oracle
false positive, 044 = documented-marginal). The pin is retired and wide-7/8/9 solve sub-pixel with
Az/El free. Open decision before flipping the default: accept the 2 synthetic-fisheye marginals (REAL
is the trustworthy gate and is neutral) vs. resolve 039/044 first; and a GUI re-test of wide-7/8/9
under rot-vec (near-certain to pass given WS1a's shared object, but worth confirming once).

### WS2 — rot-vec made the DEFAULT, gated to guided solves (2026-06-21)

Flipped `rotVecLmEnabled()` to default-ON (kill-switch `SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_ROTVEC_LM`)
and added `rotVecLmActive(settings) = rotVecLmEnabled() && plateSolveStartUsesDirection(settings)` --
rot-vec applies ONLY to direction-seeded (guided) solves; blind / fov-only solves (mode 0/1) keep the
legacy az/el/roll LM. Threaded a `useRotVec` bool from `runPlateSolveLmRefinement` (which has settings)
into `addPlateSolveLmParameterDelta` + the FD Jacobian; the wide-fisheye Az/El pin removal is gated on
the same `!rotVecLmActive(settings)` (so mode-2 elevation/blind wide keep the legacy pin where rot-vec
does not apply).

WHY guided-only: a *global* rot-vec default regressed the BLIND fisheye corpus (FISHEYE-full 56->51);
the blind/quad-hash LM path tips marginal cases under rot-vec. Gating to guided recovers it to 56.

**Full validation of the guided-gated default (all 5 baseline corpora + FISH4):**

| corpus | rot-vec default | prior baseline | |
|---|---|---|---|
| REAL (trustworthy) | 48 | 48 | neutral; wide-7/8/9 sub-pixel, Az/El pin RETIRED |
| RAND2 (canonical synthetic) | 148 | 148 | neutral |
| WIDE | 27 | 27 | neutral |
| FISHEYE-full (blind, mode0/1) | 56 | 56 | neutral (legacy LM) |
| FISH4 (guided fisheye, mode4) | 40 | 42 | -2 (marginal: 039 corrected-FP, 044) |
| RAND (STALE rand-100) | 80 | 83 | -3 (stale-corpus marginal noise; rand2 is canonical) |

So the **trustworthy + canonical gates (REAL 48, RAND2 148) are neutral**; the deltas are on the
imperfect/stale synthetic corpora (the notes already flag the fisheye oracle as imperfect and rand-100
as stale -> use rand2). Jon approved the flip on that basis. **New committed baseline: REAL 48 /
FISHEYE-full 56 / WIDE 27 / RAND2 148 / RAND ~80 / FISH4-mode4 40.** Kill-switch reverts to legacy for
A/B. Follow-ons now unblocked: the seed-anchored-grid / clamped-pp wide band-aids can likely be retired
the same way (gated on rotVecLmActive); investigate the blind-fisheye rot-vec tip if blind solving is
prioritised.

### WS2 follow-on — wide band-aid cleanup (2026-06-22)

Retired the **clamped free-principal-point polish** clamp on the modern (rot-vec) path. The ±30px
window around the grid value (cameraplatesolver.cpp ~21947) existed purely to make the RMS-minimising
LM converge *identically between the independently-compiled GUI DLL and test EXE* (cx=-41 vs cx=+91
across builds). **WS1a** makes both link the same solver object, so there is no cross-build choice to
pin, and the keep-best rule already rejects a genuine overfit. Gated the clamp on `!rotVecLmActive`
(legacy keeps it as A/B rollback). **Validated neutral: REAL 48 / WIDE 27 / FISH4 40** (identical
pass+fail sets to baseline).

The **match-count grid** (the cx/cy/k1 coarse sweep, ~21878) was tested for retirement the same way
and **kept** — it is *load-bearing*, not a band-aid: gating it off the rot-vec path regressed
**FISH4 40->39** (lost synth-fisheye-031), because it finds the off-centre fisheye principal-point
basin the free-pp LM cannot reach from zero. The notes' earlier "likely retirable" guess was wrong;
the grid is a legitimate coarse-to-fine basin finder and stays on all paths.

**Harness build/run env (this worktree, 2026-06-22):** build-qt6 is configured against
`external/windows/opencv4` (opencv **4.10.0 world**, single `opencv_world4100.dll`) but its cuda
modules are *detected-but-missing* (`opencv_cudaarithm.lib` etc. absent), so camera targets fail to
link out of the box. To build+run the harness here: (1) copy `opencv_cuda*4130.lib` ->
`opencv_cuda*.lib` (unversioned) in the override lib dir
`sdrangel-windows-libraries/opencv4/x64/vc17/lib`; (2) build via **PowerShell** capturing vcvars env
then `$env:LIB="<override-lib>;$env:LIB"` BEFORE `cmake --build` (cmd's `set LIB=...` truncates at
~8191 chars and drops the entry); (3) drop `opencv_world4100.dll` into `build-qt6/bin` (the exe needs
it at load; copy from any sibling sdrangel `build-qt6/bin`). The proper fix is to reconfigure build-qt6
against the override (4.13.0) with `-DOpenCV_DIR=<override>`, but the above gets a working, trustworthy
harness without a full reconfigure.

## WS5 — split the single translation unit (2026-06-23, DONE)

The solver was one ~24k-line TU built around a single ~21k-line **inline** `SolverContext` class
body (nested types + ~219 inline static member functions + ~112 non-static member functions, all at
column-0 indentation), with only `solve()` and the `CameraPlateSolver` public methods out-of-line.
(The plan/notes had assumed "free functions, easy to move" -- they were inline class members, so the
split required converting every definition to out-of-line form.)

Done as a series of behaviour-preserving, per-phase validated moves:
- **Phase 1:** lift the whole class into a private header `cameraplatesolverinternal.h`, included by
  `cameraplatesolver.cpp` (the orchestrator: `solve()` + ctor/dtor/public statics). Enables out-of-line
  member defs in other TUs.
- **Phases 2-7:** move member *definitions* out of the header into themed TUs as out-of-line defs
  with **leading return types** (`CameraPlateSolver::SolverContext::Evaluation
  CameraPlateSolver::SolverContext::f(args) { ... }`); nested return types are explicitly qualified,
  params are left unqualified (they are in class scope after the `Class::name` qualifier). Declarations
  stay in the class header. (The extractor first emitted trailing-return form `auto ... -> Ret` since
  that resolves nested returns in class scope automatically, then a follow-up pass rewrote them all to
  leading return types -- this codebase does not use `auto`/trailing-return in function declarations.)
  TUs: `cameraplatesolvercatalog.cpp` / `siril.cpp` / `refine.cpp` / `acceptance.cpp` / `core.cpp`
  (catalog I/O, projection, visibility, signatures, seeds, matching) / `pipeline.cpp` (non-static
  pipeline members: fetch/build/evaluate/match).

Result: the shared header went **~21k -> 1.95k lines** (now a real declaration header: nested types +
member decls + data members + 3 `template<size_t N>` members + a few tiny `const` members that must
stay inline). Function bodies live in the 6 themed TUs, all in the `camera_platesolver` static lib, so
the GUI plugin and the harness link the identical split object (WS1a preserved). Editing a body now
recompiles only its TU + the small header. Each phase validated **REAL 48 / WIDE 27 / FISH4-mode4 40**
(identical pass+fail sets); commits `693e9bfa8` (P1) .. `8bf241184` (P7).

**Mechanics note:** the move was scripted (a throwaway `ws5extract.py`, not committed) -- the codebase
uses Allman braces at function level (col-0 `{` ... col-0 `}`), so functions are extractable by
paren-matching the signature + col-0 brace bounds. Skips: template-preceded members (must stay inline),
`const` members (trailer after `)`), and the constructor. Optional follow-on: `pipeline.cpp` (~13.7k)
and `core.cpp` (~4.3k) could be split finer, but the header-shrink (the compile-time win) is done.
