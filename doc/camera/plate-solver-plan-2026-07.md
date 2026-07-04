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

### Track 0 — Measurement first (the unlock; do before any further solver tuning)

- **0a Hermetic catalog for the harness.** Snapshot the Siril region/range cache the corpus needs
  into a versioned local archive; add a harness env (`..._OFFLINE=1`) that fails loudly on any
  network miss instead of fetching. Kills the pollux-class nondeterminism, makes REAL a true
  hermetic gate, and makes RAND2 runnable without re-downloading. *Cheap; highest value per line.*
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

### Track 1 — Acceptance/selection modernization (the astrometry.net lesson)

- **1a Reformulate the verifier** from the current per-match log-odds *sum* to a per-detection
  foreground/background **mixture with a distractor fraction** (Sutherland–Saunders as
  astrometry.net implements it), normalized so sparse-correct and dense-correct solves are
  comparable. Keep it in shadow mode; success criterion = clean separation of the WS3 accept/reject
  bands that the current formulation fails.
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

### Track 3 — Performance (continue the proven tier)

P-A/P-C removed the redundant-recompute tier. Next, in order:
- **T1 Re-profile** (post-P-A/P-C) on the slowest cases; the prior ranking (outer retry ladder ~30 s
  worst case, rollAliasCheck, acceptance, rollRecovery) predates these changes.
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
