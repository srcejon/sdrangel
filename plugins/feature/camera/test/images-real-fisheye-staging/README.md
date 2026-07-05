# Real-data plate-solver corpus (gathered + converted 2026-07-04)

The first **real-image** test corpus for the plate solver — Track 0b of
`doc/camera/plate-solver-plan-2026-07.md`, whose thesis is that the binding constraint on solver
improvement is validation quality, and that the synthetic-fisheye oracle is too weak to tune
against. These frames come with **externally-derived astrometric ground truth**, so anchors are
real, not self-generated.

## The two sets

- **`gmn/`** — Global Meteor Network / RMS, Perth Observatory. 6 cameras, real wide-field
  (5×53°, 1×20°), 1280×720, with per-camera **platepar** calibration (pointing + lens distortion).
  Real-WIDE regime. See `gmn/README.md`.
- **`trex/`** — UCalgary TREx RGB all-sky imagers, western Canada. Real **180° fisheye**, 553×480,
  with per-pixel **skymap** az/el calibration. Real-fisheye regime. See `trex/README.md`.

## Generated corpus CSVs (in the parent `test/` dir)

- `star-tests-real-gmn.csv` — 6 rows (4 guided mode-3 + 2 blind mode-0) over 4 GMN cameras.
- `star-tests-real-trex.csv` — 16 rows (9 guided mode-3 + 7 blind mode-1) over 9 TREx fisheye frames.

Row format matches the synthetic corpora. Image paths are relative to `test/` (the harness resolves
them against the CSV's own directory).

## How the ground-truth anchors were derived (this is the important, reusable part)

The `starPositions` anchors are **not** hand-placed or solver-generated. For each frame:

1. **GMN:** fit a 4th-order polynomial mapping tangent-plane (ξ,η, from a bright star's alt/az at
   the frame time) → pixel, using the platepar's own `star_list` (77±real RA/Dec↔pixel pairs) as
   the fit data. The fit reproduces the star_list to **~0.3 px median** (self-validating). A
   per-frame global-shift correction removes an epoch-convention offset. Predicted bright-star
   positions are then **snapped to the actual detected star peak** (required within a few px — which
   both verifies the identification and makes the anchor sub-pixel-exact on the real star).
2. **TREx:** invert the skymap's per-pixel az/el arrays directly — for each bright star, compute its
   alt/az at frame time, find the pixel whose skymap az/el matches (nearest unit-vector), then snap
   to the detected peak. No fit needed; the skymap *is* the calibration.

Every anchor was visually confirmed via the `overlay_*.png` images (green circle on each identified
star). The recovered constellations are geometrically coherent — Orion+CMa (GMN AU000C), Vela+Carina
(AU000D), the full Big Dipper + Cassiopeia (TREx LUCK) — which is the independent sanity check that
identification is correct.

Anchor accuracy target: the harness position tolerance is 24 px; anchors sit on the true detected
star (0 px error), so a *wrong* solve is caught with wide margin, and a correct solve passes.

## Provenance / regeneration

- Bright-star names/positions: the repo's own `plugins/feature/camera/brightstarcatalog.txt`.
- alt/az: astropy, with standard-atmosphere refraction, at each frame's exact UTC.
- Scripts were one-off (staged under the session temp dir); the method above is the durable record.
  The `overlay_*.png`, `metadata.csv`, `skymap_*.sav`, platepar `.cal`, and raw HDF5/FITS are kept so
  anchors can be re-derived or extended.

## First baseline (2026-07-05) — the corpus already earned its keep

Running `star-tests-real-gmn.csv` through the current solver:

- **GMN 4/6 PASS.** AU000C/D/G guided PASS; AU000D blind PASS. AU000A guided FAILs (its lone anchor
  Alpheratz went unmatched — single-anchor rows are fragile, a corpus issue not a solver one) and
  AU000C blind FAILs (Bellatrix unmatched on a dim 28-detection wide frame — a genuinely hard blind
  case).
- **Every frame reported `solved=true`.** On real data the solver always produces *a* solution;
  only the external anchors reveal when the pose is wrong. A self-consistent oracle would have
  passed several of these — which is the whole reason for a real-ground-truth corpus.
- **Concrete durable finding — roll convention.** The solver's reported roll is the **negative** of
  the GMN platepar's `rotation_from_horiz` (AU000D: platepar −3.67° ↔ solver +3.86°; verified across
  stations). `star-tests-real-gmn.csv` therefore stores `expectedRoll = -rotation_from_horiz` (and
  seeds the same). Azimuth/elevation match directly (AU000D solved az=140.1 vs platepar 138.6,
  el=29.3 vs 28.7). This mapping is needed for any future GMN-derived rows.
- **Weak-oracle warning (blind rows).** AU000D blind PASSED at az≈85/el≈60 — far from the true
  az≈140/el≈29 — i.e. a *wrong* pose satisfied the position anchors by chance. Position anchors alone
  are not a strong oracle for blind rows; use many spatially-spread anchors, and prefer guided rows
  (which also get the roll check) as the trustworthy gate until the oracle is hardened.

### TREx (real 180° fisheye) — the detector is the fisheye bottleneck (major finding)

Running `star-tests-real-trex.csv`:

- **Raw frames: 0/16, `detections=1..3, solved=false`.** The solver's `CameraStarDetector` finds
  almost nothing in the dim TREx frames (background DN ~8–16, ~1–2 px stars), while a plain 4σ
  detector finds 200–760. Root cause is the Phase-3 **S1 area-gate bug** (`cv::contourArea≈0` for
  1–2 px blobs → rejected). So on real all-sky data the *detector*, not the plate solver, is what
  fails first. The synthetic corpus never showed this (its stars are rendered larger/brighter).
- **The frames here are stored contrast-stretched** (linear DN[med−1, med+30]→[0,255]; a real
  all-sky pipeline auto-levels). Star positions are unchanged, so anchors stay valid; raw HDF5 kept.
  Stretched, the detector recovers **40–76** detections and the solver attempts real solves.
- **S1/detector-V2 (`SDRANGEL_CAMERA_STAR_DETECTOR_V2=1`) helps on real fisheye.** On a 4-row LUCK
  subset (stretched): **V2 off → 1/4 PASS (52–61 det); V2 on → 3/4 PASS (60–76 det).** This is the
  first *trustworthy* evidence that the S1 pixel-count-area fix — which regressed the synthetic REAL
  suite and looked like net-negative "oracle churn" — genuinely improves the real fisheye regime.
  It vindicates pursuing the detector-V2 package (jointly tuned to protect narrow REAL), per the plan.
- **Real 180° fisheye pose-solving is still hard.** Full set V2-on+stretched = **4/16 PASS**: the
  solver reports `solved=true` on 15/16 but lands a *wrong* pose ~75% of the time. So beyond
  detection there is a genuine fisheye-accuracy gap — now measurable against real ground truth for
  the first time (the synthetic-fisheye oracle could not show this reliably). Caveat: the TREx guided
  seed is approximate (near-zenith az/roll are degenerate; roll left for the solver to search) and
  the equidistant-fisheye model may not exactly match the TREx lens — both are follow-ups before
  treating the 4/16 as a pure solver number.

## Caveats

- **Volume is small** (16 real frames, 22 rows) — a starter set, not a full corpus. Both sources
  can be extended (TREx: bulk HTTP archive; GMN: more platepar samples on request / SkyFit2).
- **TREx is low-res** (553×480, ~0.4°/px) — bright-anchor-only. GMN AU000A/F/K yielded few anchors
  (sparse fields / narrow lens); AU000C/D are the strongest.
- **Licences:** GMN/RMS CC BY 4.0; TREx open scientific data (acknowledge per its policy). The images
  are gitignored like the synthetic corpora — local tuning assets.
- These are the *first pass*: validate the CSVs solve/behave sensibly, then fold into the standing
  gates (the plan's 0b/0c). They do **not** yet replace Jon's own-rig frames for the exact camera
  the product targets.
