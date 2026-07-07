#!/usr/bin/env python3
"""Derive bright-star pixel anchors for TREx frames and emit corpus rows.

For each kept frame:
  - compute alt/az of bright named HYG stars (sidereal-time formulas, J2000,
    no refraction — <0.2 px at the 15 deg elevation floor),
  - predict each star's pixel from the site's skymap FULL_AZIMUTH /
    FULL_ELEVATION per-pixel calibration (scipy.io.readsav),
  - snap to the brightest 3x3-strict local maximum within 6 px
    (reject if none, peak too faint, or elevation < 15 deg),
  - drop the fainter of any two stars snapping within 4 px,
  - pick up to 6 anchors (bright + well spread); keep the frame only if
    >= 3 anchors and max pairwise separation > 300 px,
  - write star-tests-real-trex2.csv (mode-3 rows, template =
    star-tests-real-trex.csv), metadata2.csv, and overlay2_*.png for eyeballing.

Self-check:  python derive_trex2_anchors.py --check
  reruns the pipeline on the existing corpus frames and diffs the snapped
  pixels against the anchors already in star-tests-real-trex.csv.
"""
import os
import re
import sys
import math
import glob
import numpy as np
import h5py
from PIL import Image, ImageDraw
from scipy.io import readsav

HYG = os.path.join(os.environ.get("APPDATA", ""),
                   "f4exb", "SDRangel", "camera", "hyg_v42_reduced.txt")

STAR_NAMES = [
    "Sirius", "Arcturus", "Vega", "Capella", "Rigel", "Procyon", "Betelgeuse",
    "Altair", "Aldebaran", "Pollux", "Deneb", "Regulus", "Castor", "Spica",
    "Bellatrix", "Elnath", "Alnilam", "Alnitak", "Alioth", "Mirfak", "Dubhe",
    "Alkaid", "Menkalinan", "Alhena", "Polaris", "Alphard", "Algieba", "Hamal",
    "Alpheratz", "Kochab", "Mirach", "Saiph", "Rasalhague", "Algol", "Almach",
    "Denebola", "Alphecca", "Mizar", "Sadr", "Eltanin", "Schedar", "Mintaka",
    "Caph", "Merak", "Enif", "Phecda", "Scheat", "Alderamin", "Markab",
]

# per-site row parameters copied from star-tests-real-trex.csv (mode-3 rows)
SITE_ROW = {  # site: (azimuth, elevation, fov)
    "luck": (242.9, 86.03, 171.0),
    "rabb": (112.2, 85.76, 173.0),
    "yknf": (168.0, 86.25, 172.0),
    "pina": (241.8, 88.55, 176.0),
}
SITE_LATLON = {
    "luck": (51.15399, -107.26474),
    "rabb": (58.22781, -103.68063),
    "yknf": (62.51985, -114.31303),
    "pina": (50.25881, -95.86517),
}
CAM = {"luck": "rgb-03", "rabb": "rgb-06", "pina": "rgb-02", "yknf": "rgb-08"}

MIN_ELEV = 15.0
MIN_PEAK = 30          # stretched DN (background sits at ~8); the temporal
MIN_PEAK_OVER_BG = 24  # mean-support check below is the real noise filter
HOT_MIN_DN = 35        # per-pixel MIN across nights >= this => static defect
HOT_SPIKE_DN = 18      # ... and >= this above the min-image local median
HOT_COUNT_DN = 40      # bright at same px on >= 3 distinct nights => defect
HOT_COUNT_NIGHTS = 3
POLE_EXEMPT_RADIUS = 12.0  # do not hot-mask near the celestial pole (Polaris)
PASS1_RADIUS = 6.0     # seed search radius (must catch a whole-camera nudge)
PASS1_MIN_PEAK = 60
PASS2_RADIUS = 3.0     # after per-frame residual correction
PASS2_RADIUS_NO_SEEDS = 3.5
INTERLOPER_RADIUS = 8.0
INTERLOPER_RATIO = 1.8  # a >=1.8x brighter peak nearby => identity suspect
AMBIGUITY_RATIO = 0.5   # second credible peak >= 50% of best => ambiguous
MIN_ANCHORS = 3
MIN_SPREAD = 300.0     # px, max pairwise
MAX_ANCHORS = 6

# frames kept after visual triage of the fetch_trex2.py candidates
KEEPERS = [
    # (luck 20260120 and pina 20251216/20260125-adjacent nights were fetched
    # and triaged out: cloud, aurora, or too few well-spread anchors)
    "trex_luck_20251223_060030utc.png",
    "trex_luck_20251223_090030utc.png",
    "trex_luck_20260210_060030utc.png",
    "trex_luck_20260219_060030utc.png",
    "trex_luck_20260219_090030utc.png",
    "trex_rabb_20251218_060030utc.png",
    "trex_rabb_20251224_060030utc.png",
    "trex_rabb_20260124_060030utc.png",
    "trex_pina_20260112_060030utc.png",
    "trex_pina_20260122_060030utc.png",
    "trex_pina_20260125_060030utc.png",
    "trex_yknf_20251216_080030utc.png",
]


def load_catalog():
    stars = {}
    with open(HYG, encoding="utf-8") as f:
        next(f)
        for line in f:
            p = line.rstrip("\n").split("|")
            if len(p) < 4 or p[0] not in STAR_NAMES or p[0] in stars:
                continue
            h, m, s = p[1].split()
            ra = (float(h) + float(m) / 60 + float(s) / 3600) * 15.0
            d, dm, ds = p[2].split()
            sign = -1.0 if d.strip().startswith("-") else 1.0
            dec = sign * (abs(float(d)) + float(dm) / 60 + float(ds) / 3600)
            stars[p[0]] = (ra, dec, float(p[3]))
    missing = [n for n in STAR_NAMES if n not in stars]
    if missing:
        sys.exit(f"stars missing from HYG catalog: {missing}")
    return stars


def julian_date(y, mo, d, h, mi, s):
    if mo <= 2:
        y -= 1
        mo += 12
    a = y // 100
    b = 2 - a + a // 4
    jd = (math.floor(365.25 * (y + 4716)) + math.floor(30.6001 * (mo + 1))
          + d + b - 1524.5)
    return jd + (h + mi / 60.0 + s / 3600.0) / 24.0


def altaz(ra_deg, dec_deg, lat_deg, lon_deg_east, jd):
    d = jd - 2451545.0
    t = d / 36525.0
    gmst = (280.46061837 + 360.98564736629 * d
            + 0.000387933 * t * t - t * t * t / 38710000.0) % 360.0
    lst = (gmst + lon_deg_east) % 360.0
    ha = math.radians((lst - ra_deg) % 360.0)
    lat = math.radians(lat_deg)
    dec = math.radians(dec_deg)
    sin_alt = (math.sin(lat) * math.sin(dec)
               + math.cos(lat) * math.cos(dec) * math.cos(ha))
    alt = math.asin(max(-1.0, min(1.0, sin_alt)))
    cos_az = ((math.sin(dec) - sin_alt * math.sin(lat))
              / (math.cos(alt) * math.cos(lat)))
    az = math.degrees(math.acos(max(-1.0, min(1.0, cos_az))))
    if math.sin(ha) > 0:
        az = 360.0 - az
    return math.degrees(alt), az


class Skymap:
    def __init__(self, site):
        s = readsav(f"skymap_{site}.sav").skymap[0]
        self.el = np.asarray(s.full_elevation, dtype=np.float64)
        self.az = np.asarray(s.full_azimuth, dtype=np.float64)
        self.valid = np.isfinite(self.el) & np.isfinite(self.az) & (self.el > 0)

    def pixel_for(self, az, el):
        """Grid cell whose calibrated az/el is angularly closest to (az, el)."""
        daz = np.abs(self.az - az)
        daz = np.where(daz > 180.0, 360.0 - daz, daz)
        cost = (self.el - el) ** 2 + (daz * math.cos(math.radians(el))) ** 2
        cost = np.where(self.valid, cost, np.inf)
        iy, ix = np.unravel_index(np.argmin(cost), cost.shape)
        return int(ix), int(iy), math.sqrt(cost[iy, ix])


# static camera defects (hot pixels): a real star never sits on the same
# pixel across different nights/hours, so the per-pixel MIN over clear frames
# from several nights stays dark everywhere except at defects.
CLEAR_FRAMES_FOR_MASK = {
    "luck": ["trex_luck_20251220_030030utc.png",
             "trex_luck_20251220_060030utc.png",
             "trex_luck_20251220_090030utc.png",
             "trex_luck_20251220_120030utc.png",
             "trex_luck_20260115_060030utc.png",
             "trex_luck_20260214_060030utc.png",
             "trex_luck_20251223_060030utc.png",
             "trex_luck_20251223_090030utc.png",
             "trex_luck_20260210_060030utc.png",
             "trex_luck_20260219_060030utc.png",
             "trex_luck_20260219_090030utc.png"],
    "rabb": ["trex_rabb_20251220_060030utc.png",
             "trex_rabb_20260115_060030utc.png",
             "trex_rabb_20251218_060030utc.png",
             "trex_rabb_20251224_060030utc.png",
             "trex_rabb_20260124_060030utc.png"],
    "pina": ["trex_pina_20260118_060030utc.png",
             "trex_pina_20260112_060030utc.png",
             "trex_pina_20260122_060030utc.png",
             "trex_pina_20260125_060030utc.png"],
    "yknf": ["trex_yknf_20260115_080030utc.png",
             "trex_yknf_20251216_080030utc.png"],
}
_hot_masks = {}


def hot_mask(site, sky):
    if site in _hot_masks:
        return _hot_masks[site]
    from scipy.ndimage import median_filter
    imgs = {f: np.array(Image.open(f).convert("L")).astype(np.int16)
            for f in CLEAR_FRAMES_FOR_MASK[site]}
    # (a) persistent point spike: min over clear frames from several nights
    stack = np.stack(list(imgs.values()))
    mn = stack.min(axis=0)
    hot = (mn >= HOT_MIN_DN) & (mn - median_filter(mn, size=5) >= HOT_SPIKE_DN)
    # (b) intermittent defect: a point-like spike at the same pixel on >= 3
    # distinct nights (a real star cannot revisit the same pixel that often)
    nights = {}
    for f, im in imgs.items():
        spike = (im >= HOT_COUNT_DN) & (im - median_filter(im, size=5) >= 25)
        night = f.split("_")[2]
        nights[night] = spike if night not in nights else nights[night] | spike
    hot |= sum(n.astype(np.int16) for n in nights.values()) >= HOT_COUNT_NIGHTS
    # exempt the celestial-pole neighbourhood: Polaris genuinely stays put
    lat, _ = SITE_LATLON[site]
    pole_x, pole_y, _ = sky.pixel_for(0.0, lat)
    yy, xx = np.mgrid[0:hot.shape[0], 0:hot.shape[1]]
    hot &= ((xx - pole_x) ** 2 + (yy - pole_y) ** 2
            > POLE_EXEMPT_RADIUS ** 2)
    dil = hot.copy()  # dilate by 1 px
    dil[1:, :] |= hot[:-1, :]
    dil[:-1, :] |= hot[1:, :]
    dil[:, 1:] |= hot[:, :-1]
    dil[:, :-1] |= hot[:, 1:]
    print(f"  [{site}] hot-pixel mask: {int(hot.sum())} px "
          f"({int(dil.sum())} after dilation)")
    _hot_masks[site] = dil
    return dil


def local_peaks(img, hot, px, py, radius):
    """3x3 local maxima within radius of (px, py), hot pixels skipped.

    Returns [(x, y, DN, dist)] sorted by distance. Small saturated plateaus
    (<= 3 tied neighbours) count as peaks.
    """
    h, w = img.shape
    r = int(math.ceil(radius))
    peaks = []
    for y in range(max(1, py - r), min(h - 1, py + r + 1)):
        for x in range(max(1, px - r), min(w - 1, px + r + 1)):
            dist = math.hypot(x - px, y - py)
            if dist > radius or hot[y, x]:
                continue
            v = int(img[y, x])
            n = img[y - 1:y + 2, x - 1:x + 2].astype(int)
            if v < n.max() or (n == v).sum() > 4:
                continue
            peaks.append((x, y, v, dist))
    peaks.sort(key=lambda p: p[3])
    return peaks


def qualifying(img, peaks):
    out = []
    h, w = img.shape
    for x, y, v, dist in peaks:
        if v < MIN_PEAK:
            continue
        y0, y1 = max(0, y - 10), min(h, y + 11)
        x0, x1 = max(0, x - 10), min(w, x + 11)
        if v - float(np.median(img[y0:y1, x0:x1])) < MIN_PEAK_OVER_BG:
            continue
        out.append((x, y, v, dist))
    return out


def persistence_ok(h5ctx, x, y):
    """A star keeps its flux at (x, y) across the whole minute (the sky
    rotates < 1 px in 60 s); noise maxima, satellites, meteors and cosmic
    rays do not survive the 20-frame temporal mean.

    Requires the 3x3 signal in the temporal-mean image to be both a real
    detection (>= 2.5 raw DN over local background) and a large fraction of
    the single-frame signal."""
    if h5ctx is None:
        return True  # no .h5 available: cannot check
    frame, mean = h5ctx

    def sig(im):
        peak = float(im[max(0, y - 1):y + 2, max(0, x - 1):x + 2].max())
        bg = float(np.median(im[max(0, y - 6):y + 7, max(0, x - 6):x + 7]))
        return peak - bg
    s_frame = sig(frame)
    s_mean = sig(mean)
    return s_frame > 0 and s_mean >= 1.8 and s_mean >= 0.55 * s_frame


def snap_star(img, hot, h5ctx, px, py, radius):
    """Snap one star; None unless the identification is unambiguous."""
    cand = [c for c in qualifying(img, local_peaks(img, hot, px, py, radius))
            if persistence_ok(h5ctx, c[0], c[1])]
    if not cand:
        return None
    cand.sort(key=lambda c: -c[2])
    # merge tied plateau pixels / PSF shoulders into their brightest pixel
    merged = []
    for c in cand:
        if all(max(abs(c[0] - m[0]), abs(c[1] - m[1])) > 2 for m in merged):
            merged.append(c)
    cand = merged
    if len(cand) >= 2 and cand[1][2] >= AMBIGUITY_RATIO * cand[0][2]:
        return None  # ambiguous: two comparable peaks near the prediction
    x, y, v, dist = cand[0]
    # interloper: a clearly brighter peak just outside the snap radius means
    # the identification (and possibly the calibration here) is suspect
    for ix, iy, iv, _ in local_peaks(img, hot, px, py, INTERLOPER_RADIUS):
        if (ix, iy) != (x, y) and iv >= INTERLOPER_RATIO * v:
            return None
    return x, y, v


def parse_frame_name(png):
    m = re.match(r"trex_([a-z]{4})_(\d{8})_(\d{6})utc\.png", png)
    site, date, hms = m.group(1), m.group(2), m.group(3)
    y, mo, d = int(date[:4]), int(date[4:6]), int(date[6:8])
    h, mi, s = int(hms[:2]), int(hms[2:4]), int(hms[4:6])
    return site, (y, mo, d, h, mi, s)


def open_h5(png):
    """Raw grayscale (frame used, temporal mean of the minute) for the PNG."""
    site, (y, mo, d, h, mi, s) = parse_frame_name(png)
    name = f"{y:04d}{mo:02d}{d:02d}_{h:02d}{mi:02d}_{site}_{CAM[site]}_full.h5"
    if not os.path.exists(name):
        return None
    with h5py.File(name, "r") as f:
        gray = f["data/images"][:].astype(np.float32).mean(axis=2)
        ts = [t.decode() if isinstance(t, bytes) else str(t)
              for t in f["data/timestamp"][:]]
    idx = 0
    for i, t in enumerate(ts):
        if f"{h:02d}:{mi:02d}:{s:02d}" in t:
            idx = i
            break
    # short temporal mean (+-2 frames, 15 s): sky rotation smears < 0.4 px
    # even at the frame corners, while noise drops by sqrt(5)
    lo, hi = max(0, idx - 2), min(gray.shape[2], idx + 3)
    return gray[:, :, idx], gray[:, :, lo:hi].mean(axis=2)


def anchors_for(png, catalog, skymaps, verbose=True, select=True):
    site, (y, mo, d, h, mi, s) = parse_frame_name(png)
    lat, lon = SITE_LATLON[site]
    jd = julian_date(y, mo, d, h, mi, s)
    if site not in skymaps:
        skymaps[site] = Skymap(site)
    sky = skymaps[site]
    img = np.array(Image.open(png).convert("L"))
    hot = hot_mask(site, sky)
    h5ctx = open_h5(png)

    # predicted pixel per visible catalog star
    preds = {}
    for name, (ra, dec, mag) in catalog.items():
        el, az = altaz(ra, dec, lat, lon, jd)
        if el < MIN_ELEV:
            continue
        px, py, _ = sky.pixel_for(az, el)
        preds[name] = (px, py, mag, az, el)

    # pass 1: bright, unambiguous seeds -> per-frame residual. The wide seed
    # radius also catches a small whole-camera nudge relative to the skymap.
    resid = []
    for name, (px, py, mag, az, el) in preds.items():
        cand = qualifying(img, local_peaks(img, hot, px, py, PASS1_RADIUS))
        merged = []
        for c in sorted(cand, key=lambda c: -c[2]):
            if all(max(abs(c[0] - m[0]), abs(c[1] - m[1])) > 2
                   for m in merged):
                merged.append(c)
        if len(merged) == 1 and merged[0][2] >= PASS1_MIN_PEAK:
            x, yv, v, _ = merged[0]
            if persistence_ok(h5ctx, x, yv):
                resid.append((x - px, yv - py))
    # residuals are not uniform across the field (a small rotation component
    # exists), so a median correction from too few seeds can mislead
    dx = int(round(np.median([r[0] for r in resid]))) if len(resid) >= 5 else 0
    dy = int(round(np.median([r[1] for r in resid]))) if len(resid) >= 5 else 0
    radius = PASS2_RADIUS if len(resid) >= 5 else PASS2_RADIUS_NO_SEEDS
    if verbose:
        print(f"    pass1: {len(resid)} seeds, residual (+{dx},+{dy}), "
              f"pass2 radius {radius}")

    # pass 2: all stars with the residual-corrected prediction
    cands = []
    for name, (px, py, mag, az, el) in preds.items():
        snap = snap_star(img, hot, h5ctx, px + dx, py + dy, radius)
        if verbose:
            st = (f"snap ({snap[0]},{snap[1]}) DN {snap[2]}" if snap
                  else "REJECT")
            print(f"    {name:12s} mag {mag:5.2f} az {az:6.1f} el {el:5.1f} "
                  f"pred ({px},{py}) {st}")
        if snap:
            cands.append((name, mag, snap[0], snap[1], snap[2]))

    # de-duplicate: two stars snapped to (nearly) the same peak
    cands.sort(key=lambda c: c[1])  # brightest first
    kept = []
    for c in cands:
        if all(math.hypot(c[2] - k[2], c[3] - k[3]) > 4.0 for k in kept):
            kept.append(c)

    # greedy spread-aware selection of up to MAX_ANCHORS
    if select and len(kept) > MAX_ANCHORS:
        pts = [(c[2], c[3]) for c in kept]
        best_pair = max(
            ((i, j) for i in range(len(pts)) for j in range(i + 1, len(pts))),
            key=lambda ij: math.hypot(pts[ij[0]][0] - pts[ij[1]][0],
                                      pts[ij[0]][1] - pts[ij[1]][1]))
        sel = list(best_pair)
        while len(sel) < MAX_ANCHORS:
            rest = [i for i in range(len(kept)) if i not in sel]
            # prefer bright stars that are far from those already chosen
            def score(i):
                dmin = min(math.hypot(pts[i][0] - pts[j][0],
                                      pts[i][1] - pts[j][1]) for j in sel)
                return dmin - 15.0 * kept[i][1]
            sel.append(max(rest, key=score))
        kept = [kept[i] for i in sorted(sel)]

    spread = 0.0
    for i in range(len(kept)):
        for j in range(i + 1, len(kept)):
            spread = max(spread, math.hypot(kept[i][2] - kept[j][2],
                                            kept[i][3] - kept[j][3]))
    ok = len(kept) >= MIN_ANCHORS and spread > MIN_SPREAD
    return site, kept, spread, ok


def draw_overlay(png, anchors):
    im = Image.open(png).convert("RGB")
    dr = ImageDraw.Draw(im)
    for name, mag, x, y, v in anchors:
        dr.line([(x - 8, y), (x - 3, y)], fill=(255, 60, 60))
        dr.line([(x + 3, y), (x + 8, y)], fill=(255, 60, 60))
        dr.line([(x, y - 8), (x, y - 3)], fill=(255, 60, 60))
        dr.line([(x, y + 3), (x, y + 8)], fill=(255, 60, 60))
        dr.text((x + 6, y + 4), name, fill=(120, 220, 120))
    im.save("overlay2_" + png)


def exact_utc_and_peaks(png):
    """Exact frame timestamp from the retained .h5 + star-like peak count."""
    site, (y, mo, d, h, mi, s) = parse_frame_name(png)
    h5 = f"{y:04d}{mo:02d}{d:02d}_{h:02d}{mi:02d}_{site}_{CAM[site]}_full.h5"
    utc = f"{y:04d}-{mo:02d}-{d:02d} {h:02d}:{mi:02d}:{s:02d} UTC"
    if os.path.exists(h5):
        with h5py.File(h5, "r") as f:
            for t in f["data/timestamp"][:]:
                t = t.decode() if isinstance(t, bytes) else str(t)
                if f"{h:02d}:{mi:02d}:{s:02d}" in t:
                    utc = t if t.endswith("UTC") else t + " UTC"
                    utc = utc.replace("utc", "UTC")
                    break
    import fetch_trex2
    img8 = np.array(Image.open(png).convert("L"))
    sm = readsav(f"skymap_{site}.sav").skymap[0]
    mask = np.asarray(sm.full_elevation, dtype=np.float32) > 5.0
    return utc, fetch_trex2.starlike_peaks(img8, mask)


def self_check(catalog, skymaps):
    """Re-derive anchors for existing corpus frames; diff vs the mode-3 rows."""
    rows = {}
    with open("../../star-tests-real-trex.csv", encoding="utf-8") as f:
        header = f.readline()
        for line in f:
            if ",3," not in line:
                continue
            png = os.path.basename(line.split(",", 1)[0])
            pos = line.rstrip().rsplit(",", 1)[-1]
            rows[png] = {p.split(":")[0]: (int(p.split(":")[1]),
                                           int(p.split(":")[2]))
                         for p in pos.split(";")}
    worst = 0.0
    for png, expected in rows.items():
        site, kept, spread, ok = anchors_for(png, catalog, skymaps,
                                             verbose=False, select=False)
        mine = {c[0]: (c[2], c[3]) for c in kept}
        for name, (ex, ey) in expected.items():
            if name in mine:
                dd = math.hypot(mine[name][0] - ex, mine[name][1] - ey)
                worst = max(worst, dd)
                flag = "" if dd <= 2.0 else "  <-- CHECK"
                print(f"  {png} {name:12s} mine {mine[name]} "
                      f"expected ({ex},{ey}) d={dd:.1f}{flag}")
            else:
                print(f"  {png} {name:12s} NOT RECOVERED (expected "
                      f"({ex},{ey}))")
    print(f"self-check worst distance: {worst:.1f} px")


def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    catalog = load_catalog()
    skymaps = {}

    if "--check" in sys.argv:
        self_check(catalog, skymaps)
        return

    csv_rows = []
    meta_rows = []
    for png in KEEPERS:
        print(png)
        site, kept, spread, ok = anchors_for(png, catalog, skymaps)
        print(f"  -> {len(kept)} anchors, max spread {spread:.0f} px, "
              f"{'OK' if ok else 'FRAME REJECTED'}")
        if not ok:
            continue
        draw_overlay(png, kept)
        az, el, fov = SITE_ROW[site]
        lat, lon = SITE_LATLON[site]
        _, (y, mo, d, h, mi, s) = parse_frame_name(png)
        tstr = f"{y:04d}-{mo:02d}-{d:02d} {h:02d}:{mi:02d}:{s:02d}"
        names = ",".join(c[0] for c in kept)
        pos = ";".join(f"{c[0]}:{c[2]}:{c[3]}" for c in kept)
        csv_rows.append(
            f"images-real-fisheye-staging/trex/{png},{tstr},{lat},{lon},200,"
            f"{az},{el},0.0,{fov},equidistant fisheye,0,0,0,\"{names}\","
            f"6.0,3,0,0,0,0,,,{pos}")
        utc, peaks = exact_utc_and_peaks(png)
        date_url = f"{y:04d}/{mo:02d}/{d:02d}"
        meta_rows.append(
            f"{png},{site},{lat},{lon},{utc},3.0s,TREx RGB stream0,"
            f"https://data.phys.ucalgary.ca/sort_by_project/TREx/RGB/stream0/"
            f"{date_url}/,{peaks}")

    header = ("image,time,latitude,longitude,altitude,azimuth,elevation,roll,"
              "fov,projection,cx,cy,k1,stars,plateSolveMaxMagnitude,"
              "plateSolveStartMode,roiX,roiY,roiW,roiH,expectedRoll,"
              "expectedRollTolerance,starPositions")
    with open("../../star-tests-real-trex2.csv", "w", encoding="utf-8",
              newline="\n") as f:
        f.write(header + "\n")
        f.write("\n".join(csv_rows) + "\n")
    with open("metadata2.csv", "w", encoding="utf-8", newline="\n") as f:
        f.write("image,site,latitude,longitude,utc_time,exposure,source,"
                "source_url_base,starlike_peaks\n")
        f.write("\n".join(meta_rows) + "\n")
    print(f"\nwrote {len(csv_rows)} rows to star-tests-real-trex2.csv "
          f"and metadata2.csv")


if __name__ == "__main__":
    main()
