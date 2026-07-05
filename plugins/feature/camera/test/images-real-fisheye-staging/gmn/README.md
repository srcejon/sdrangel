# GMN/RMS real wide-field staging set (gathered 2026-07-04)

Real wide-field star frames with **full astrometric ground truth**, for the plate-solver real
corpus (Track 0b of `doc/camera/plate-solver-plan-2026-07.md`). This is the real-WIDE-regime
companion to the TREx 180° fisheye set in `../trex/`.

## Source

`Tests/ExampleStationData/stations.tar.bz2` from the RMS repository
(https://github.com/CroatianMeteorNetwork/RMS) — sample Global Meteor Network station data from
**Perth Observatory, Western Australia**, published for SkyFit2 testing. RMS/GMN data is
CC BY 4.0 / open for exactly this kind of use.

## Contents

Six co-located cameras (site −32.0075°, 116.1349°E, 390 m), one FF frame each from the night of
2025-08-17, different pointings:

| station | az_centre | alt_centre | rot | FoV(h) | notes |
|---|---|---|---|---|---|
| AU000A | 6.04°   | 47.85° | −0.08° | 53.1° | CALSTARS file included |
| AU000C | 66.26°  | 51.15° | +4.46° | 53.0° | |
| AU000D | 138.60° | 28.70° | −3.67° | 53.1° | trees in frame; use mask.bmp (peak count inflated) |
| AU000F | 289.48° | 40.14° | +0.64° | 52.9° | |
| AU000G | 232.55° | 48.46° | −2.33° | 53.0° | CALSTARS file included |
| AU000K | 214.46° | 48.79° | −2.67° | **19.7°** | 16 mm lens — mid/narrow regime |

- `gmn_<station>_<time>utc_ave.png`: the FF file's **avepixel** plane (mean of 256 frames over
  10.24 s — stars are points, meteors average away), 1280×720 8-bit. Epoch = FF block start
  + 5.12 s, in `metadata.csv`.
- `Stations/<st>/platepar_cmn2010.cal`: **the ground truth** (JSON): station lat/lon/elev,
  pointing (az/alt of centre to ~0.01°), rotation, plate scale, and the fitted lens-distortion
  polynomials. This is the independent reference for solver pose AND distortion validation.
- `Stations/<st>/CALSTARS_*.txt` (A and D): RMS's own extracted star list (x, y, intensity) —
  directly comparable against our star detector's output on the same frame.
- `Stations/<st>/mask.bmp`: sensor mask (AU000D needs it — trees).
- Raw `FF_*.fits` kept (maxpixel/avepixel/stdpixel planes; astropy reads them).

## Why this set matters

Unlike the corpus so far, these frames come with *externally fitted* pointing + distortion, so a
solve can be graded on **pose error in degrees**, not just named-anchor PASS/FAIL — feeding the
plan's 0c "record accuracy, not just verdicts" metric. The 5×53° + 1×20° mix lands in the
solver's wide and mid regimes with known roll ≠ 0 (up to 4.5°).

## Extending the set

More per-night GMN data (stacks, calibration reports) is browsable at
https://globalmeteornetwork.org/weblog/ but raw FF frames + platepars are generally NOT published
per night — this RMS sample tarball is the clean public exception. For volume, options are:
ask GMN (data is CC BY 4.0, they share on request), run SkyFit2 on any station's public data,
or use UKMON's live/archive APIs (api.ukmeteors.co.uk — returned 5xx errors when tried).
