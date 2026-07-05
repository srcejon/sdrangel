# TREx RGB real-fisheye staging set (gathered 2026-07-04)

Real 180° all-sky fisheye frames for the plate-solver real-fisheye corpus (Track 0b of
`doc/camera/plate-solver-plan-2026-07.md`).

## Source

University of Calgary **TREx RGB** all-sky imagers (auroral science network, western Canada).
Open data: https://data.phys.ucalgary.ca/sort_by_project/TREx/RGB/ (HTTP/FTP/rsync).
Acknowledge per `acknowledgements.pdf` at the archive root if any result is ever published.

- Cameras: zenith-pointed 180° fisheye, RGB colour, **553×480**, 3-second exposures,
  one HDF5 file per minute (20 frames) under `stream0/YYYY/MM/DD/<site>_rgb-NN/utHH/`.
- Frames here are the middle frame (index n/2) of the named minute, saved as lossless PNG
  with NO stretching (raw 8-bit levels; sky background DN ~8–16, stars peak DN ~15–70).
- `metadata.csv`: image, site, latitude, longitude (from the site's skymap calibration,
  5 dp), exact UTC timestamp (from the HDF5, ~ms precision), exposure, source, peak count.

## Ground truth (the important part)

`skymap_<site>.sav` (IDL save, readable with `scipy.io.readsav`) contains the site's
**per-pixel FULL_ELEVATION / FULL_AZIMUTH arrays** — a complete astrometric calibration of
each camera including its real lens distortion, plus exact site lat/lon. This gives:

- ground-truth pointing (zenith pixel, roll) for corpus row seeds and PASS validation,
- an independent reference for the solver's fisheye projection/distortion model
  (compare solved az/el per matched star against the skymap's per-pixel az/el).

Zenith pixels (max-elevation pixel, x,y): luck (287,234), rabb (263,238), yknf (269,231),
pina (280,237).

## Site coordinates

| site | latitude | longitude | notes |
|---|---|---|---|
| luck (Lucky Lake SK) | 51.15399 | −107.26474 | southernmost = least aurora; best frames |
| pina (Pinawa MB) | 50.25881 | −95.86517 | occulting boom + trees in frame (robustness) |
| rabb (Rabbit Lake SK) | 58.22781 | −103.68063 | |
| yknf (Yellowknife NT) | 62.51985 | −114.31303 | auroral oval — airglow/aurora likely |

## Frame selection

Nights chosen near new moon (2025-12-20, 2026-01-15/18, 2026-02-14), scored by star-like
local-maxima count at low background (see session notes). The 2025-12-20 Lucky Lake night is
the cleanest (four frames at 03/06/09/12 UT — same camera, sky rotated ≈ 45° between frames,
good for roll-diversity). Known contaminants to expect when building corpus rows:
aurora/airglow structure (esp. yknf), horizon trees, an occulting boom (pina), and the low
resolution (~0.4°/pixel — bright-star anchors only; ~200–700 detectable star-like peaks).

## Caveats / next steps

- These are the LOW-RESOLUTION real-fisheye regime (553×480). They complement — not replace —
  frames from Jon's own higher-res rig and, potentially, ESO ALPACA (Paranal all-sky, in the
  ESO science archive) for a high-quality tier. NASA All-Sky Fireball Network (fisheye video
  stills, 768×494) and GMN (wide-field with platepar ground truth, CC BY 4.0) remain gathered-
  later options — see the plan doc.
- The .h5 files are kept alongside: each holds 20 frames of the same minute (more material,
  e.g. for detection-repeatability tests) and the exact timestamps.
- To make corpus rows: pick 3+ bright named-star anchors per frame (seed the solver in mode 4
  at the known zenith pointing, or read anchors straight from the skymap az/el), then add rows
  to a `star-tests-real-fisheye.csv` with startMode/lens settings mirroring the synthetic
  fisheye rows but with `projection` matching the skymap's radial profile.
