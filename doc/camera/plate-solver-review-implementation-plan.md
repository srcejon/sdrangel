# Plate Solver Review — Implementation Plan (2026-07-01)

Implements the findings of the 2026-07-01 five-agent code review of `CameraPlateSolver` +
`CameraStarDetector`, in a dedicated git worktree. Self-contained: read this file plus
`plate-solver-improvement-plan.md` (WS0–WS5 context) and `plate-solver-notes.md` (what is settled)
before starting. Nothing here re-litigates settled ground: seed unification stays abandoned,
`blindquad` stays deleted; the k2 revert is revisited **only** as an explicitly gated probe after
the lens-inversion fixes (Phase 4), because the original k2 trial predates those fixes and may have
been confounded by them.

## Baseline to protect

Re-measure at Phase 0 (numbers drift; the last recorded full-suite baseline was
REAL 48 / FISHEYE 56 / WIDE 27 / RAND 83 / RAND2 148, with FISHEYE mode-1 36/50 and mode-4 42/50).
Rules, unchanged from prior workstreams:

- **REAL is the gate.** No change lands if it costs a REAL case.
- Validate every behaviour-affecting change against **all five corpora + the robustness negatives**
  (`run-robustness.ps1`, `robustness-neg*.csv`) — record **pass/fail sets**, not counts (verdict-set
  diffs catch churn that stable totals hide).
- Behaviour-*neutral* changes (pure caching/lookup) still get one full-suite run and must be
  **verdict-set identical**.
- ULP discipline (the wide-7/8/9 lesson): any change to floating-point *computation order* — even a
  mathematically identical one — is behaviour-affecting, not neutral. Classified per item below.
- One item (or one tightly-coupled group) per commit; commit with
  `git -c submodule.recurse=false commit`; never chain `git rebase` after a commit that may fail.
  Revert-first on any regression; re-probe later with instrumentation.

## Phase 0 — Worktree bootstrap + baseline capture

1. `git worktree add ../srcejon_sdrangel_fix-review <branch>` off the current camera branch.
   Commit this plan to the branch first so it is present in the worktree.
2. Bootstrap (proven procedure; do **not** touch the main checkout):
   - Copy `external/windows` from the main checkout into the worktree (the main copy carries local
     modifications a submodule update would not reproduce). Also copy the untracked SDK dirs:
     `external/windows/asi` (required — `cameraasicontroller.h` fails without `ASICamera2.h`).
   - Copy the **untracked test tooling** from the main checkout's `plugins/feature/camera/test/`:
     `robustness-meta.csv`, `robustness-neg*.csv`, `images-robustness/`, the `*.ps1` harnesses
     (`seed-ablation.ps1`, `gateablate*.ps1`, `run-robustness.ps1`), and
     `star-tests-nearboundary-neg.csv` if present.
   - Configure inside the worktree, outside the sandbox, from vcvars64, with
     `-G Ninja -DOpenCV_DIR="C:/Users/jon/source/repos/sdrangel-windows-libraries/opencv4/x64/vc17/lib"`.
   - Build `featurecamera_star_tests` via
     `cmake --build --preset default-qt6-windows --target featurecamera_star_tests --parallel`
     (PowerShell wrapper, not the Bash `cmd /c` form — the latter swallows output).
   - Run with `QT_PLUGIN_PATH` + `PATH` per AGENTS.md (worktree paths, plus the override tree's
     `opencv4/x64/vc17/bin`); exe by absolute path; CSVs by their real location (image paths are
     CSV-relative). Watch for the UTF-8-BOM off-by-one when tallying `^(PASS|FAIL)`.
3. **Baseline capture:** run all five corpora + FISHEYE mode-1 and mode-4 splits + robustness
   negatives; save per-case verdict sets to `test/baseline-2026-07/` (untracked). Also record the
   quiet-run wall-clock per corpus (perf baseline) and one `SDRANGEL_CAMERA_PLATE_SOLVER_PROFILE=1`
   stage-timing run on a slow case (e.g. m51@14) for the Phase 2/6 before/after.

## Phase 1 — Defects

Fix first; each is small and independently committable.

**Status (2026-07-02): D1, D2, D3, D5, D6, D7, D8 landed and validated (REAL 47/48 + FISHEYE
mode1 35/50 + mode4 40/50 + WIDE 27/36, all verdict sets identical to baseline). D4 ATTEMPTED
AND REVERTED** — quantizing the three banded sort keys (1e-6 / 0.05 / 0.20) is a valid strict-weak
ordering but reshuffles near-tie rankings enough to move the corpus: FISHEYE-mode4 40→44 (+4), but
**WIDE 27→21 (−6)** and FISHEYE-mode1 churned 16 cases at net zero; REAL stayed 47. Some WIDE cases
were implicitly relying on the (UB) band ordering. The UB is latent (no observed crash on the small
candidate-pool arrays in MSVC release), so D4 is deferred to a dedicated effort: it needs (a) quantum
calibration per comparator against the FULL corpus, (b) RAND2 (dense-narrow near-tie regime) in the
gate — establish a same-branch RAND2 baseline first, and (c) likely splitting the sort-key comparator
from the pairwise incumbent check (the plan's original intent) rather than quantizing the shared
function. Fold into WS1b (decision-boundary margins) work.

| # | Item | Where | Class | Validation |
|---|---|---|---|---|
| D1 | Fine roll sweep unreachable: built under `!useStartElevation && !useStartDirection`, consumed only in the `useStartElevation` branch. Decide intent: wire `fovSearchRollOffsets` into the guided-fov grid (mode-1) or delete the fine path. | `cameraplatesolverpipeline.cpp:11956`, `:12235` | **Behaviour-affecting** | A/B on FISHEYE mode-1 + WIDE; full suite. This is also the first Phase-5 coverage probe. |
| D2 | Prefetch cleanup null-deref: `abort()` can synchronously run `finishPending` (nulls `item->reply`), then `deleteLater()` is called on the null. Disconnect the reply from the loop context before `abort()`, and re-check `item->reply` after. | `cameraplatesolverpipeline.cpp:466-474` | Neutral (cancel path) | Code inspection + cancel-during-solve manual test in GUI. |
| D3 | Siril chunk-index fetch failure negative-cached forever (empty `QByteArray` in a never-evicted cache). Don't cache failures, or cache with timestamp + retry window. | `cameraplatesolverpipeline.cpp:327` | Neutral (failure path) | Simulated failure (bad URL env or offline) then retry succeeds. |
| D4 | Strict-weak-ordering violations: epsilon-band comparators fed to `std::sort` (match-assignment sort `\|Δ\|>0.20`; candidate-pool sort via `isBetterWeakModeEvaluation` 1e-6 / `isBetterGuidedDirectionEvaluation` 0.05). Quantize the sort key (exact ties), keep bands only for head-to-head incumbent checks. | `cameraplatesolverpipeline.cpp:7003`, `:11606`, `:8372`, `:8396` | **Behaviour-affecting** (rank order may legitimately change) | Full suite verdict-set diff; also serves WS1b determinism. |
| D5 | `repairFinalMatchCollisions`: no `usedCatalogs` guard — two swaps on one catalog star unseat each other. Add the guard. | `cameraplatesolvercore.cpp:1192-1241` | Behaviour-affecting (rare) | Full suite. |
| D6 | `fetchSirilRangeFromSource`: `timeoutTimer.stop()` before the `isActive()` test — capture the flag before stopping. | `cameraplatesolverpipeline.cpp:163-171` | Neutral | Inspection. |
| D7 | No cancellation checks in the seed-anchored lens grid + polish span (125 serial `evaluatePose` + two 5-pass LM loops). Add `isCancellationRequested()` checks per grid row / LM pass. | `cameraplatesolver.cpp:783-980` | Neutral | Cancel-responsiveness manual test. |
| D8 | `rescoreWeakModeCandidateWithDistortionSweep`: `baseDistortionK1` dead in the calibrate branch; absolute sweep capped at −0.05. Minimal fix here: include `candidate.distortionK1` as a sweep point and remove the dead variable. (The full blind lens-recovery lever is Phase 4 / L4.) | `cameraplatesolverpipeline.cpp:11662-11684` | Behaviour-affecting | FISHEYE + WIDE A/B; full suite. |

## Phase 2 — Behaviour-neutral performance

Do these **before** the accuracy phases: they cut suite wall-clock, which multiplies through every
later validation run. Split into two sub-tiers.

**Status (2026-07-04): P-A and P-C landed bit-identical; P-B already present; P-E/P-F/P-G deferred.**
- **P-A DONE (621ec44b9):** per-solve `FinalPassSeedCache` (guarded by a catalog `visibleStarsGeneration`
  stamp + seed-ref params + detection count). Measured −15..−25% per-case solve time on dense
  narrow-guided cases (cluster-m7 7027→5279ms, galaxy-m31 5606→4236ms); verdict sets identical.
- **P-C DONE (35168e347):** memoized the four pure per-eval gate/score values the pairwise ranking
  comparator recomputed for the incumbent every comparison (`hasStrongDenseNarrowGuidedFinalPass`,
  `finalMatchPassScore`, `narrowGuidedBrightConsistencyScore`, `narrowGuidedSeedConsistencyScore`),
  cached at the end of `evaluateFinalMatchPass` with a `cachedGatesValid` flag + fallback accessors;
  verdict sets identical.
- **P-B:** already implemented in current code (the seed-radial loop uses `matchedDetectionByCatalog`).
- **P-E/P-F/P-G:** deferred — recon showed non-bit-identical wrinkles (P-F normalize is ULP-affecting;
  P-E changes the failure result's catalog fields; P-G's deferred metrics need an equality check).

**2a — bit-identical (caching / lookup replacement); verdict sets must not change:**

- P-A `evaluateFinalMatchPass` candidate-independent recomputation: cache per solve (member scratch)
  the seed projector, seed-projected visible-star points, `hasNearbySeedDetection` flags (use the
  spatial grid, not the O(V×D) scan), sorted `detectionRadii`, sorted bright-detection list.
  Runs per final pass today ×(≤256 pool + 12 roll aliases + ≤24 rescue). Likely the largest single
  win on narrow guided solves. `cameraplatesolverpipeline.cpp:8719-9008`
- P-B O(V×M) match scan → existing `matchedDetectionByCatalog` hash lookup.
  `cameraplatesolverpipeline.cpp:9015-9041` (hash built at `:8748`)
- P-C Comparator memoization: lazily cached derived fields (median error, gate booleans, scores) on
  `Evaluation`/`FinalMatchPassEvaluation` so `isBetterWeakModeFinalMatchPass` and the pool sorts stop
  recomputing both sides per comparison. Pairs with D4. `cameraplatesolverpipeline.cpp:9856-10451`
- P-D Generation-stamped scratch instead of per-call `QSet`/`QHash` in `evaluateAnchoredPose`
  (`:7409`) and `evaluateFinalMatchPass` (`:8468`, `:8669`, `:8707`); reuse the intrusive projected-star
  grid in `appendSupplementalMatches` instead of rebuilding a `QHash` grid per call
  (`cameraplatesolvercore.cpp:3888`).
- P-E Early bail before catalog load: hoist the detections-count checks above the catalog-context
  build (which can hit the network). `cameraplatesolver.cpp:282` vs `:422-437`
- P-F Drop the duplicate ±180° roll offset in `hasCompetitiveRollAlias`
  (`cameraplatesolverpipeline.cpp:10474-10481`); drop the redundant `normalize(vectorFromAltAz(...))`
  (`cameraplatesolvercore.cpp:3140`, `:1390`, `:3335`); insert the sliced Siril sub-range under its own
  `cacheKey` (`cameraplatesolverpipeline.cpp:250-253`).
- P-G Defer `populatePoseScoringMetrics` on grid evaluations until a candidate beats `best` or passes
  the pool floors (matchCount/RMS only). `cameraplatesolverpipeline.cpp:7217`, `:7324` — confirm the
  deferred metrics are recomputed identically before pool insertion, else this moves to 2b.
- P-H Bound the rescue shortlist during pass 1 (insert-sorted, cap-aware) instead of trimming after
  ~2600 seeds. `cameraplatesolverpipeline.cpp:11429-11456` — neutral only if the final trimmed set is
  provably identical; otherwise 2b.

**2b — ULP-affecting micro-optimisations (full-suite A/B each, revert on any REAL delta):**

- P-I `projectVector` trig elimination (algebraic cosφ/sinφ; rectilinear with no transcendentals;
  equisolid via half-angle; equidistant via `atan2`). Hottest function under the grids.
  `cameraplatesolvercore.cpp:1440-1457`
- P-J Pin the Wahba/Davenport convention with a unit test; delete the both-orders × both-R/Rᵀ
  fallbacks (up to 4 error evaluations per hypothesis). `cameraplatesolvercore.cpp:1826-1923`,
  `:2005-2116`
- P-K `medianDistancePixels` via double `nth_element` (deterministic both-middles form).
  `cameraplatesolveracceptance.cpp:23-41`
- P-L Subsample `hasGeometricallyConsistentMatches` above ~100 matches (deterministic stride).
  `cameraplatesolveracceptance.cpp:231-264`
- P-M Cone cull under negative k1: widen the cone by the barrel pull-in bound instead of disabling.
  `cameraplatesolvercore.cpp:2998`
- P-N Quad-index build: hoist `neighborDistances`, use the cell grid for neighbour lookup.
  `cameraplatesolvercore.cpp:3742-3746`. Keep `queryEpsilonBall` ε ≤ cellSize (or set cellSize=ε at
  build). `cameraplatesolverinternal.h:1321-1359`

Exit: quiet-run suite time recorded; target is a measurable cut vs Phase 0 with identical (2a) /
accepted (2b) verdict sets.

## Phase 3 — Star detector accuracy package

Highest expected value for the fisheye gap, and the riskiest for the tuned stack (changes the input
distribution of **every** case). Flag-gate the package behind one env var
(`SDRANGEL_CAMERA_STAR_DETECTOR_V2=1`) during development; land items individually once green.
Full five-corpus run after **each** item; FISHEYE mode-1/mode-4 are the success metrics, REAL the gate.

**Status (2026-07-03): S1 landed flag-gated (default OFF, byte-identical); S3 tried + reverted; V2
NOT default-eligible — no REAL-safe win found.** Measured under the flag:
- **S1 (true pixel-count area)** alone: FISHEYE mode1 35→38 (+3), mode4 40→45 (+5), WIDE 27 (net 0),
  but **REAL 47→43 (−4)** — recovering small blobs adds noise/galaxy-structure false positives that
  confuse dense/real fields (narrow-3, wide-9, m101, m51-2). Committed flag-gated (103f19600).
- **S1+S3 (correct half-normal σ from the positive residual)**: REAL recovered only to 45 (−2), and
  the higher σ → higher 4σ threshold **killed the fisheye gain** (mode1 back to 35, mode4 41) and
  **regressed WIDE 27→21 (−6)**. Root cause: the 4σ multiplier was calibrated against the
  *underestimated* legacy σ, so correcting σ over-thresholds — the two are coupled and must be
  re-tuned together. S3 reverted (net-negative even flag-gated).
- **Conclusion:** every V2 variant regresses REAL, the trustworthy gate. The synthetic-fisheye
  movement is largely oracle churn (consistent with `plate-solver-notes.md`: "remaining synthetic
  mode-1 failures are dominated by test-corpus quality issues… REAL is the trustworthy gate"). So
  the fisheye gap is NOT closeable via these detector changes without a real regression. S1 stays a
  documented opt-in experiment; do not flip the V2 default. A genuine attempt would need to (a) fix σ
  AND re-calibrate the threshold multiplier jointly, (b) add quality-gated small-star recovery so
  only high-confidence faint stars are admitted, and (c) validate on a *trustworthy* fisheye corpus
  (real fisheye frames, not the weak-oracle synthetic set) — a research loop, not a quick fix.

Remaining unattempted items (S2/S4/S5/S6/S8/S9/S10) are held behind the same conclusion: without a
trustworthy fisheye gate and a σ+threshold co-calibration, they risk the same REAL-for-synthetic
trade. Revisit only with a real-fisheye validation corpus.

1. S1 Pixel-count areas: replace `findContours`+`contourArea`+per-contour `drawContours` with one
   `connectedComponentsWithStats` pass (fixes area-0 rejection of 1-px-wide blobs, fixes `fillRatio`
   bias, removes per-blob allocation — the perf item S7 falls out for free).
   `camerastardetector.cpp:809-845`
2. S2 Field-relative hot-pixel classification: after the detection loop compute median FWHM/area;
   keep `hotPixelSuspect` only on outliers vs the field PSF (e.g. fwhm < 0.6·median and
   median ≥ 1.5 px). May allow retiring the downstream hot-pixel-label recovery workaround — check
   after landing. `camerastardetector.cpp:934-936`
3. S3 Signed residual noise: compute the residual as CV_16S/CV_32F; estimate σ from the negative
   half (−p15.87); threshold on the positive side. Removes the `sigma < 1.0` fallback symptom.
   `camerastardetector.cpp:629`, `:106-175`
4. S4 Interpolated tile thresholds (bilinear across 512-px tiles, CLAHE-style) + per-star **local** σ
   for SNR/uncertainty instead of the global mean. `camerastardetector.cpp:190-217`, `:914`
5. S5 Centroid over an expanded box (+2–3 px) weighted by `max(0, residual − 1σ_local)` instead of
   the threshold-truncated contour mask. `camerastardetector.cpp:843-902`
6. S6 Illumination-validity mask: masked/normalized-convolution background, exclusion rects and the
   fisheye dark-corner circle excluded from background and σ estimation (today the mask is ANDed
   only after thresholding). `camerastardetector.cpp:614-659`
7. S8 Hot-path copies: view instead of clone for Grayscale16 (`:50-54`, `:609`); `cv::cvtColor` for
   RGBA64 (`:62-77`); RGB2GRAY without the RGB→BGR hop (`:496-498`, `:611`); per-tile `cv::compare`
   to 8U (`:186`, `:654-658`); downscale→blur→upscale (or box cascade) background — evaluate a
   median-based background at the same time (stops bright stars inflating their own background).
8. S9 CUDA parity for saturated-core recovery: GPU threshold at 250 + download the binary mask (or
   gray ROIs for low-roundness candidates) so bloomed bright anchors aren't GPU-path-only losses.
   `camerastardetector.cpp:704-707`, `:979`
9. S10 (conditional) Radius-dependent aspect/roundness relaxation for fisheye corner coma — only if
   corner stars are still missing after S1–S6, and only after checking the synthetic generator's PSF
   model. `camerastardetector.cpp:817-841`

## Phase 4 — Lens model / geometry accuracy

1. L1 Newton undistortion: replace the 8-iteration fixed-point loop in `unprojectPixelToVector` with
   Newton on the radius cubic (3–4 iterations, explicit convergence exit).
   `cameraplatesolvercore.cpp:1500-1519`
2. L2 Monotonicity guard in `projectVector`: reject when `1 + 3·k1·r² <= 0` (kills fold-region ghost
   stars), not just `scale <= 0`. `cameraplatesolvercore.cpp:1458-1468`
3. L3 QUEST-style λmax (or Rayleigh-quotient convergence check) in
   `largestSymmetric4Eigenvector` — the current shift compresses eigenvalue separation to ~2–3
   digits of quaternion accuracy. `cameraplatesolvercore.cpp:1769-1798`
4. L4 Blind lens recovery: run the coarse (Cx,Cy,K1) grid (`cameraplatesolver.cpp:873-885`) on the
   top 2–3 weak-mode candidates for `isWidePlateSolveContext` non-direction solves (today it is
   gated on direction/elevation seeds, so blind/mode-1 candidates never get principal-point or
   strong-k1 recovery). Parallelizable; bounded cost. Builds on D8.
5. L5 Quad-code canonicalization boundary alternates: when `|xC−xD| ≤ ε`, `|xC+xD−1| ≤ ε`, or the two
   largest pair separations are within the angular noise, query the alternate canonical forms.
   `cameraplatesolvercore.cpp:2167-2181`, `:2262-2282`
6. L6 Projection-dependent FoV limit in `createProjector` (rectilinear < 90° half-FoV; equidistant/
   equisolid to ~179°) so all-sky 180° settings produce a valid projector.
   `cameraplatesolvercore.cpp:1408`
7. L7 (gated probe) k2 re-trial **after** L1+L2 land, since the original net-negative trial ran on
   the flawed inverse. Alternative if k2 fails again: angular (great-circle) residual gates for wide
   solves (`cameraplatesolveracceptance.cpp:446-468`), which are field-angle-homogeneous and don't
   touch the lens model. Pick one; both is over-fitting surface.
8. L8 (deferred, note only) k1's FOV-relative normalization couples the FOV and k1 LM columns;
   re-parameterizing on the image half-diagonal would decondition the joint fit — record as a
   follow-on, do not mix into this phase. `cameraplatesolvercore.cpp:1460`
9. L9 Tie-break nits: `fabs(normalizeSignedDegrees(roll))` and fuzzy FOV equality in the geometric
   tie-break. `cameraplatesolvercore.cpp:4168-4170`

## Phase 5 — Blind-search coverage probes (independent A/Bs)

Each is a single-knob experiment against FISHEYE mode-1 + WIDE + negatives; keep only what moves the
number without costing REAL/RAND2. Run after Phases 3–4 (detector/lens fixes change the baseline
these knobs were tuned against).

- C1 Quad-hash scaling: ε from expected pixel error (`matchRadius/(imageLongEdge/fovDeg)` mapped to
  code space, clamped 0.02–0.05); catalog bright pool 24→32–40; detection pool 16→18–20; verify
  until K *valid* seeds rather than first-50 hypotheses. `cameraplatesolverpipeline.cpp:6296-6311`
- C2 Triangle direction dedup: quality-aware basin admission (allow 2–3 per basin or key on
  direction+FoV bucket); require 2 concordant seeds before the wide-mode `earlyExit`.
  `cameraplatesolverpipeline.cpp:2262-2279`, `:2584-2588`
- C3 Field-angle-scaled fisheye ratio tolerance (or bucketRadius 2 for fisheye); densify the fisheye
  FoV sweep (add 0.70/0.90/1.10/1.40 or a second pass around the best). `:2129`, `:2303`
- C4 Wide bright-pair FoV seeds from ~100° (or derive from `m_minFov`/projection). `:5548`
- C5 Wide-fallback skip gate: require `isStrongWideWeakBlindSeed` (already defined at
  `:12441-12449`) to skip the grid in `wideWeakMode`, or run a decimated grid regardless.
  `:12662-12678`
- C6 Density-scaled `buildMatches` per-detection candidate cap (stars per match-disk area) — this is
  also the designated probe for the **catalog-depth instability** (m51@16 crowd-out signature).
  `cameraplatesolverpipeline.cpp:6921-6926`
- C7 Guided-direction roll-loop early stop without a roll prior once ≥K distinct pool basins exist —
  only if Phase 0 profiling shows this stage matters. `:12164-12165`

## Phase 6 — Parallelism (behaviour-preserving merge rules mandatory)

Deterministic merge = fixed stage/index order, keep-best comparators applied in that order.
Full-suite verdict-set check after each.

- T1 Four sub-engines of `buildBrightGuidedAnchorTriangleSeeds` as pool tasks (outputs concatenated
  in fixed stage order). `cameraplatesolverpipeline.cpp:2680-5102`
- T2 Bucket the precomputed catalog triples (`buildTriangleSignatureBuckets` exists) for the
  ordered-triangle scan; separation-sorted binary search for the anchor-triple window.
  `:4237-4380`, `:4689-4835`
- T3 Atomic work-stealing index in the bright-pair threading (replaces batch+`waitForDone`
  stragglers). `:6096-6127`
- T4 Parallelize: the 125-point lens grid (`cameraplatesolver.cpp:865-961`, merge in index order),
  `hasCompetitiveRollAlias`'s 12 final passes, rescue pass-1 seeds, the guided-direction /
  wide-fallback grids (azimuth stripes). The `copySearchStateFrom` worker-context pattern already
  exists.
- T5 Recenter-ladder batching: run the 16 recenter attempts (and the deepen-escape's 5 azimuth
  offsets) in small parallel batches on detection copies, merged in offset order via the existing
  score rules. `cameraplatesolver.cpp:2869`, `:3037-3123`

## Phase 7 — Network / caching robustness

- N1 Siril range-cache LRU eviction (insertion-order list) instead of clear-all at 32 MB; touch
  region-cache file mtime on read so disk eviction is LRU not write-FIFO.
  `cameraplatesolver.cpp:2511-2521`, `cameraplatesolversiril.cpp:187-220`
- N2 Prefetch source-1 fallback: re-issue failed ranges against Zenodo within the same prefetch
  loop. `cameraplatesolverpipeline.cpp:421`
- N3 Catalog-context memo key: include catalog path mtime / source fingerprint so a mid-session
  download can't serve a stale context. `cameraplatesolverpipeline.cpp:1173-1190`
- N4 Nested-`QEventLoop` re-entrancy: minimum = guard/document `CameraStarDetector` slots against
  re-entry during a solve; full fix (dedicated network thread + blocking wait) is a separate,
  scoped follow-on — do not bundle it into this pass.

## Phase 8 — Structural cleanups & constants

- X1 Extract `applySelectedFinalPassToResult(...)` + `labelDetectionsFromFinalPass(...)` from the
  three duplicated result-population blocks (already caused one stale-field bug).
  `cameraplatesolver.cpp:2004-2020`, `:2176-2221`, `:2274-2319`
- X2 Resolution-relative constants (WS0 continuation): express the fixed pixel tolerances
  (20/30 px triangle/quad edges, 18/14 px blind-seed RMS/median, 18 px pp floors, 28 px similarity
  floor) as fractions of image long edge anchored at their current 1080p-class values. One constant
  per commit, full suite each.
- X3 Acceptance cliffs → ramps/margins (WS1b style, **most carefully**): the 0.80 mag-err constant
  tuned between cases 044/034 (`cameraplatesolver.cpp:1518`), the `minMatches+156/160` cliff
  (`:1291`), the sparse-wide `>12 detections` cliff (`cameraplatesolveracceptance.cpp:357`).
  Consider wiring the shadow-mode `poseFalseAlarmLogOdds` as the discriminator for the first —
  it exists for exactly this. Needs the near-boundary negative suite; skip any item whose negative
  coverage is missing.
- X4 Align `isStrongGuidedSolve` / `isAcceptableElevationSeedEvaluation` on the `ResidualGates`
  shape (deferred if X3 stalls). `cameraplatesolveracceptance.cpp:370`, `:514`
- X5 Hygiene: `#if 1` block (`cameraplatesolverpipeline.cpp:2936`), double-brace leftovers
  (`:4272`), misaligned braces (`:269-271`), elevation clamp vs `kVisibleAltitudeFloor` bin range
  (`:1839`), dot-product comparison instead of per-record `angularSeparationDegrees` in the Siril
  loaders (`:749`, `:966`), extract the shared Siril record-reader (~200 duplicated lines).

## Ordering rationale & exit criteria

Order: **0 → 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8**, with 7 and 8 movable earlier if blocked. Phase 2
before the accuracy work because it multiplies through every later suite run; Phase 3 before 4/5
because the detector changes shift the baseline the search/lens knobs are tuned against; Phase 5
after 4 for the same reason. Phases 1, 7, and X1/X5 are safe filler whenever a long suite run is in
flight.

Done means:
- All Phase 1 defects fixed; suite verdict sets ≥ baseline (REAL never below 48).
- FISHEYE mode-1 > 36/50 and mode-4 > 42/50 (any net gain accepted; each contributing knob
  individually attributed).
- Quiet-run suite wall-clock measurably below the Phase 0 baseline.
- Every landed change has its own commit + a one-line entry appended to `plate-solver-notes.md`
  (including reverted probes — negative results are part of the record).
