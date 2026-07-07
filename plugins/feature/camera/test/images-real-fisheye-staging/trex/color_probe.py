#!/usr/bin/env python3
"""Color-separability probe: can star color (B-V) from RGB all-sky frames pin
the camera ROLL absolutely -- i.e. discriminate the true pose from a wrong-roll
pose independent of geometry?

Reuses derive_trex2_anchors machinery: skymap_<site>.sav per-pixel az/el grids
(scipy.io.readsav), sidereal alt/az from RA/Dec + site lat/lon + UTC, and the
skymap pixel_for() lookup. Samples color from the RAW LINEAR h5 RGB cube
(data/images, shape (H,W,3,frames)) at the frame nearest :30 -- NOT the
stretched PNG (whose stretch distorts color).

Measured color index per star (background-subtracted aperture flux):
    measuredColor = (Bflux - Rflux) / (Bflux + Rflux)
Blue stars -> positive, red -> negative. Catalog color = B-V (ci column):
blue -> low/negative B-V. So measuredColor should NEGATIVELY correlate with B-V
at the true pose.

Discriminator: Spearman(measuredColor, B-V) at true pose vs at wrong rolls
(rotate each true-pose pixel about the zenith by delta) vs random pixels (null).

Outputs: color_probe_results.csv (per star) and color_probe_scatter.png.
"""
import os
import re
import sys
import math
import numpy as np
import h5py
from scipy.io import readsav
from scipy.stats import spearmanr

# ---------------------------------------------------------------------------
# stacking mode (set from argv in main):
#   default  -> +-2-frame temporal mean centered on the frame nearest :30
#   --deep   -> median stack of ALL frames in the cube (registered)
#   --stack N-> median stack of N frames centered on :30 (registered if N>5)
# The stack combine is a per-channel MEDIAN, which lifts SNR AND rejects
# satellites / cosmic rays / aircraft that a plain mean would smear in.
STACK_MODE = "mean2"       # "mean2" | "deep" | int N
REGISTER = True            # integer-pixel align frames to the first before stack
CSV_OUT = "color_probe_results.csv"
SCATTER_OUT = "color_probe_scatter.png"

HYG = os.path.join(os.environ.get("APPDATA", ""),
                   "f4exb", "SDRangel", "camera", "hyg_v42.csv")

CAM = {"luck": "rgb-03", "rabb": "rgb-06", "pina": "rgb-02", "yknf": "rgb-08"}
SITE_LATLON = {
    "luck": (51.15399, -107.26474),
    "rabb": (58.22781, -103.68063),
    "yknf": (62.51985, -114.31303),
    "pina": (50.25881, -95.86517),
}

# frames to probe, given as h5 files (site derived from name). Chosen to span
# 4 sites and clear nights; the frame nearest :30 is used from each.
FRAMES = [
    "20251223_0900_luck_rgb-03_full.h5",   # best-populated luck frame
    "20251223_0600_luck_rgb-03_full.h5",
    "20260219_0900_luck_rgb-03_full.h5",
    "20260210_0600_luck_rgb-03_full.h5",
    "20251224_0600_rabb_rgb-06_full.h5",
    "20251218_0900_rabb_rgb-06_full.h5",
    "20260112_0600_rabb_rgb-06_full.h5",
    "20260112_0600_pina_rgb-02_full.h5",
    "20260122_0600_pina_rgb-02_full.h5",
    "20251216_0800_yknf_rgb-08_full.h5",
    "20260112_0800_yknf_rgb-08_full.h5",
    "20260219_0800_yknf_rgb-08_full.h5",
]

VMAG_MAX = 3.5      # bright, unambiguous
MIN_ELEV = 30.0     # avoid horizon extinction reddening (color confound)
APER = 2            # half-size -> 5x5 aperture
BG_HALF = 8         # 17x17 local background window
SNR_MIN = 5.0       # validity gate: aperture peak clearly above background
DENOM_MIN = 8.0     # min B+R summed background-subtracted flux for a valid color
ROLLS = [30, 60, 90, 120, 180]
N_RANDOM = 200
RNG = np.random.default_rng(12345)


# ---------------------------------------------------------------------------
# astronomy (copied verbatim from derive_trex2_anchors.py)
# ---------------------------------------------------------------------------
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
        daz = np.abs(self.az - az)
        daz = np.where(daz > 180.0, 360.0 - daz, daz)
        cost = (self.el - el) ** 2 + (daz * math.cos(math.radians(el))) ** 2
        cost = np.where(self.valid, cost, np.inf)
        iy, ix = np.unravel_index(np.argmin(cost), cost.shape)
        return int(ix), int(iy), math.sqrt(cost[iy, ix])

    def zenith_pixel(self):
        """Optical axis = the el~90 (max elevation) valid grid point."""
        el = np.where(self.valid, self.el, -np.inf)
        iy, ix = np.unravel_index(np.argmax(el), el.shape)
        return float(ix), float(iy)


# ---------------------------------------------------------------------------
def load_catalog():
    """Bright stars: name, ra(deg), dec(deg), Vmag, B-V. Vmag<VMAG_MAX."""
    import csv
    out = []
    with open(HYG, encoding="utf-8") as f:
        rd = csv.DictReader(f)
        for row in rd:
            try:
                mag = float(row["mag"])
            except (ValueError, KeyError):
                continue
            if mag >= VMAG_MAX:
                continue
            ci = row.get("ci", "").strip()
            if ci == "":
                continue
            try:
                bv = float(ci)
            except ValueError:
                continue
            ra = float(row["ra"]) * 15.0      # hours -> deg
            dec = float(row["dec"])
            name = row.get("proper", "").strip()
            if not name:
                bf = row.get("bf", "").strip()
                name = bf if bf else f"HIP{row.get('hip', '')}"
            out.append((name, ra, dec, mag, bv))
    return out


def parse_h5_name(name):
    m = re.match(r"(\d{8})_(\d{4})_([a-z]{4})_", name)
    date, hm, site = m.group(1), m.group(2), m.group(3)
    y, mo, d = int(date[:4]), int(date[4:6]), int(date[6:8])
    h, mi = int(hm[:2]), int(hm[2:4])
    return site, (y, mo, d, h, mi)


def _bright_star_seeds(lum0, n=6, margin=20, minpeak=None):
    """Pick up to n bright, isolated local maxima in a single luminance frame
    (used as registration references). Returns [(x,y)]."""
    H, W = lum0.shape
    bg = np.median(lum0)
    if minpeak is None:
        minpeak = bg + 8.0 * (np.std(lum0) + 1e-6)
    flat = lum0.copy()
    flat[:margin, :] = -1; flat[-margin:, :] = -1
    flat[:, :margin] = -1; flat[:, -margin:] = -1
    seeds = []
    work = flat.copy()
    for _ in range(n):
        iy, ix = np.unravel_index(np.argmax(work), work.shape)
        if work[iy, ix] < minpeak:
            break
        seeds.append((int(ix), int(iy)))
        work[max(0, iy - 12):iy + 13, max(0, ix - 12):ix + 13] = -1
    return seeds


def _centroid(lum, x, y, half=4):
    sub = lum[y - half:y + half + 1, x - half:x + half + 1]
    bg = np.median(lum[y - 2 * half:y + 2 * half + 1,
                       x - 2 * half:x + 2 * half + 1])
    w = np.clip(sub - bg, 0, None).astype(np.float64)
    if w.sum() < 1:
        return None
    yy, xx = np.mgrid[y - half:y + half + 1, x - half:x + half + 1]
    return (w * xx).sum() / w.sum(), (w * yy).sum() / w.sum()


def register_and_stack(imgs, frame_idxs):
    """Median-stack the selected frames per-channel, after integer-pixel
    registration of each frame to the first via a consensus of bright-star
    centroid shifts. Returns (stacked (H,W,3), drift_px)."""
    H, W, C, N = imgs.shape
    sel = [i for i in frame_idxs if 0 <= i < N]
    lum = imgs.astype(np.float32).sum(axis=2)     # (H,W,N)
    seeds = _bright_star_seeds(lum[:, :, sel[0]])
    # measure per-frame shift (consensus median of star centroid deltas)
    ref = {s: _centroid(lum[:, :, sel[0]], *s) for s in seeds}
    shifts = {sel[0]: (0, 0)}
    max_drift = 0.0
    for i in sel[1:]:
        dxs, dys = [], []
        for s in seeds:
            c0 = ref[s]
            c1 = _centroid(lum[:, :, i], *s)
            if c0 and c1:
                dxs.append(c1[0] - c0[0]); dys.append(c1[1] - c0[1])
        if dxs:
            mdx, mdy = float(np.median(dxs)), float(np.median(dys))
        else:
            mdx, mdy = 0.0, 0.0
        max_drift = max(max_drift, math.hypot(mdx, mdy))
        shifts[i] = (int(round(-mdx)), int(round(-mdy)))  # shift back to ref
    # apply integer shifts and median-combine per channel
    if REGISTER and max_drift >= 1.0:
        chans = []
        for c in range(C):
            stack = np.empty((len(sel), H, W), np.float32)
            for k, i in enumerate(sel):
                sx, sy = shifts[i]
                stack[k] = np.roll(np.roll(imgs[:, :, c, i].astype(np.float32),
                                           sy, axis=0), sx, axis=1)
            chans.append(np.median(stack, axis=0))
        out = np.stack(chans, axis=2)
        method = "registered-median"
    else:
        out = np.median(imgs[:, :, :, sel].astype(np.float32), axis=3)
        method = "median-noreg"
    return out, max_drift, method


def open_h5_rgb(name):
    """Return linear RGB (H,W,3) combined per STACK_MODE, plus UTC string,
    central-frame index, measured full-minute drift (px) and stack method.

    mean2: +-2-frame temporal mean centered on :30 (baseline).
    deep : median stack of ALL cube frames, registered.
    int N: median stack of N frames centered on :30, registered if N>5.
    """
    if not os.path.exists(name):
        return None, None, None, None, None
    with h5py.File(name, "r") as f:
        imgs = f["data/images"][:]            # (H, W, 3, frames)
        ts = [t.decode() if isinstance(t, bytes) else str(t)
              for t in f["data/timestamp"][:]]
    N = imgs.shape[3]
    secs = []
    for t in ts:
        try:
            secs.append(float(t.split()[1].split(":")[2]))
        except Exception:
            secs.append(999.0)
    idx = int(np.argmin([abs(sv - 30.0) for sv in secs]))

    if STACK_MODE == "mean2":
        lo, hi = max(0, idx - 2), min(N, idx + 3)
        frame = imgs[:, :, :, lo:hi].astype(np.float32).mean(axis=3)
        # still measure drift for the report, but don't register a 5-frame mean
        _, drift, _ = register_and_stack(imgs, list(range(lo, hi)))
        return frame, ts[idx], idx, drift, "mean2(+-2)"
    if STACK_MODE == "deep":
        sel = list(range(N))
    else:
        half = int(STACK_MODE) // 2
        sel = list(range(max(0, idx - half), min(N, idx + half + 1)))
    frame, drift, method = register_and_stack(imgs, sel)
    return frame, ts[idx], idx, drift, f"{method}(n={len(sel)})"


# ---------------------------------------------------------------------------
def sample_color(rgb, x, y):
    """Background-subtracted (Bflux-Rflux)/(Bflux+Rflux) + peak SNR at (x,y).

    Returns (measuredColor, snr, ok) where ok is False if off-frame.
    Color is None when total flux ~0 (undefined)."""
    H, W, _ = rgb.shape
    xi, yi = int(round(x)), int(round(y))
    if xi < BG_HALF or xi >= W - BG_HALF or yi < BG_HALF or yi >= H - BG_HALF:
        return None, 0.0, False
    ax0, ax1 = xi - APER, xi + APER + 1
    ay0, ay1 = yi - APER, yi + APER + 1
    bx0, bx1 = xi - BG_HALF, xi + BG_HALF + 1
    by0, by1 = yi - BG_HALF, yi + BG_HALF + 1
    R = rgb[:, :, 0]
    G = rgb[:, :, 1]
    B = rgb[:, :, 2]
    lum = R + G + B
    # local background per channel = median of the surrounding window
    bgR = np.median(R[by0:by1, bx0:bx1])
    bgB = np.median(B[by0:by1, bx0:bx1])
    bgL = np.median(lum[by0:by1, bx0:bx1])
    stdL = np.std(lum[by0:by1, bx0:bx1]) + 1e-6
    apR = R[ay0:ay1, ax0:ax1]
    apB = B[ay0:ay1, ax0:ax1]
    apL = lum[ay0:ay1, ax0:ax1]
    peakL = float(apL.max())
    snr = (peakL - bgL) / stdL
    fluxR = float((apR - bgR).sum())
    fluxB = float((apB - bgB).sum())
    denom = fluxB + fluxR
    # denom floor: (B-R)/(B+R) is meaningless when the summed star flux is
    # near zero (noise divided by noise -> +-1 blowups like Phecda +1.86).
    # Require the combined B+R signal to be a real detection.
    if denom < DENOM_MIN:
        return None, snr, True
    return (fluxB - fluxR) / denom, snr, True


def rotate_about(x, y, cx, cy, deg):
    a = math.radians(deg)
    dx, dy = x - cx, y - cy
    return (cx + dx * math.cos(a) - dy * math.sin(a),
            cy + dx * math.sin(a) + dy * math.cos(a))


def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    catalog = load_catalog()
    print(f"catalog: {len(catalog)} stars with Vmag<{VMAG_MAX} and a B-V")
    skymaps = {}

    rows = []          # per-star csv rows
    per_frame = []     # (frame, n, trueCorr, {roll:corr}, randCorr, ...)
    pooled = {"true": ([], []), "rand": ([], [])}  # (meas, bv) lists
    pooled_roll = {r: ([], []) for r in ROLLS}

    for png in FRAMES:
        site, (y, mo, d, h, mi) = parse_h5_name(png)
        lat, lon = SITE_LATLON[site]
        rgb, ts, idx, drift, method = open_h5_rgb(png)
        if rgb is None:
            print(f"{png}: h5 MISSING -- skipped")
            continue
        # exact UTC seconds of the central frame for the alt/az epoch
        try:
            s = int(float(ts.split()[1].split(":")[2]))
        except Exception:
            s = 30
        jd = julian_date(y, mo, d, h, mi, s)
        H, W, C = rgb.shape
        if C != 3:
            print(f"{png}: cube not 3-channel (C={C}) -- COLOR UNRECOVERABLE")
            continue
        if site not in skymaps:
            skymaps[site] = Skymap(site)
        sky = skymaps[site]
        cx, cy = sky.zenith_pixel()

        # candidate bright stars above the elevation floor at this time
        kept = []   # (name, mag, bv, px, py, meas_true)
        for name, ra, dec, mag, bv in catalog:
            el, az = altaz(ra, dec, lat, lon, jd)
            if el < MIN_ELEV:
                continue
            px, py, _ = sky.pixel_for(az, el)
            meas, snr, ok = sample_color(rgb, px, py)
            present = ok and snr >= SNR_MIN and meas is not None
            if present:
                kept.append((name, mag, bv, px, py, meas))

        # de-dup: two catalog stars landing on the same aperture
        kept.sort(key=lambda k: k[1])   # brightest first
        dedup = []
        for k in kept:
            if all(math.hypot(k[3] - m[3], k[4] - m[4]) > 3.0 for m in dedup):
                dedup.append(k)
        kept = dedup

        n = len(kept)
        if n < 5:
            print(f"{png}: only {n} stars kept -- too few for correlation")
            # still record rows
        bv_list = [k[2] for k in kept]
        meas_true = [k[5] for k in kept]

        # true-pose correlation
        trueCorr = (spearmanr(meas_true, bv_list).correlation
                    if n >= 4 else float("nan"))

        # wrong-roll correlations: rotate each true pixel about zenith
        rollCorr = {}
        meas_roll = {r: [] for r in ROLLS}
        for r in ROLLS:
            mvals, bvals = [], []
            for (name, mag, bv, px, py, mt) in kept:
                rx, ry = rotate_about(px, py, cx, cy, r)
                meas, snr, ok = sample_color(rgb, rx, ry)
                meas_roll[r].append(meas if (ok and meas is not None)
                                    else float("nan"))
                if ok and meas is not None:
                    mvals.append(meas)
                    bvals.append(bv)
                    pooled_roll[r][0].append(meas)
                    pooled_roll[r][1].append(bv)
            rollCorr[r] = (spearmanr(mvals, bvals).correlation
                           if len(mvals) >= 4 else float("nan"))

        # random-pixel null: sample N random valid in-frame pixels, correlate
        # a random shuffle of them against the SAME bv list, repeated
        randCorrs = []
        valid_yx = np.argwhere(sky.valid)
        for _ in range(30):
            if n < 4:
                break
            mvals = []
            for _ in range(n):
                iy, ix = valid_yx[RNG.integers(len(valid_yx))]
                meas, snr, ok = sample_color(rgb, ix, iy)
                mvals.append(meas if (ok and meas is not None) else 0.0)
            c = spearmanr(mvals, bv_list).correlation
            if not math.isnan(c):
                randCorrs.append(c)
        randCorr = float(np.median(randCorrs)) if randCorrs else float("nan")
        # pooled random null: several random draws per kept star, pairing the
        # measured color at a random valid pixel with THAT star's B-V. Skip
        # empty-sky draws (undefined color) so lengths stay matched. Multiple
        # draws per star keep the pooled null a stable ~0 (a single draw per
        # star is small-N and correlates by chance).
        for bv in bv_list:
            for _ in range(5):
                iy, ix = valid_yx[RNG.integers(len(valid_yx))]
                meas, snr, ok = sample_color(rgb, ix, iy)
                if ok and meas is not None:
                    pooled["rand"][0].append(meas)
                    pooled["rand"][1].append(bv)

        for m, bv in zip(meas_true, bv_list):
            pooled["true"][0].append(m)
            pooled["true"][1].append(bv)

        # robust blue-vs-red separation
        def blue_red_gap(mvals):
            order = np.argsort(bv_list)          # bluest (low B-V) first
            paired = [(bv_list[i], mvals[i]) for i in order
                      if not (isinstance(mvals[i], float) and math.isnan(mvals[i]))]
            if len(paired) < 8:
                return float("nan"), False
            k = min(5, len(paired) // 2)
            blue = [p[1] for p in paired[:k]]
            red = [p[1] for p in paired[-k:]]
            gap = float(np.mean(blue) - np.mean(red))
            return gap, gap > 0
        gap_true, ok_true = blue_red_gap(meas_true)
        gap_r90, ok_r90 = blue_red_gap(meas_roll[90])

        per_frame.append(dict(frame=png, site=site, n=n, trueCorr=trueCorr,
                              rollCorr=rollCorr, randCorr=randCorr,
                              gap_true=gap_true, ok_true=ok_true,
                              gap_r90=gap_r90, ok_r90=ok_r90,
                              drift=drift, method=method))
        print(f"{png}  n={n:2d}  trueCorr={trueCorr:+.3f}  "
              f"roll90={rollCorr[90]:+.3f}  rand={randCorr:+.3f}  "
              f"blueRedGap true={gap_true:+.3f}({'Y' if ok_true else 'n'}) "
              f"r90={gap_r90:+.3f}({'Y' if ok_r90 else 'n'})  "
              f"drift={drift:.2f}px {method}")

        for i, (name, mag, bv, px, py, mt) in enumerate(kept):
            rows.append(dict(frame=png, star=name, vmag=mag, bv=bv,
                             px=px, py=py, meas_true=mt,
                             meas_roll90=meas_roll[90][i], kept=1))

    # -----------------------------------------------------------------------
    # write per-star csv
    with open(CSV_OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write("frame,star,vmag,catalogBV,truePixelX,truePixelY,"
                "measuredColor_true,measuredColor_roll90,kept\n")
        for r in rows:
            mr = r['meas_roll90']
            mr_s = f"{mr:.4f}" if not (isinstance(mr, float) and math.isnan(mr)) else "nan"
            f.write(f"{r['frame']},{r['star']},{r['vmag']:.2f},{r['bv']:.3f},"
                    f"{r['px']},{r['py']},{r['meas_true']:.4f},{mr_s},"
                    f"{r['kept']}\n")

    # -----------------------------------------------------------------------
    # pooled numbers
    def corr(pair):
        a, b = pair
        if len(a) < 4 or len(b) < 4:
            return float("nan")
        n = min(len(a), len(b))
        return spearmanr(a[:n], b[:n]).correlation
    print("\n==== POOLED ====")
    pooled_true = corr(pooled["true"])
    print(f"pooled trueCorr (all stars, all frames): {pooled_true:+.3f} "
          f"(N={len(pooled['true'][0])})")
    for r in ROLLS:
        print(f"pooled roll{r:3d}Corr: {corr(pooled_roll[r]):+.3f} "
              f"(N={len(pooled_roll[r][0])})")
    # pooled null = median of the robust per-frame nulls (each a 30-draw
    # median). The raw corr over the pooled random-pair list is unreliable
    # (repeated-bv tie blocks + occasional real-star hits make it wander), so
    # report the per-frame-aggregate as the trustworthy pooled baseline.
    _rc = [pf["randCorr"] for pf in per_frame
           if not math.isnan(pf["randCorr"])]
    print(f"pooled randCorr (median of per-frame nulls): "
          f"{np.median(_rc):+.3f}  [raw pooled-pair corr "
          f"{corr(pooled['rand']):+.3f}, unreliable]")

    tc = [pf["trueCorr"] for pf in per_frame if not math.isnan(pf["trueCorr"])]
    r90 = [pf["rollCorr"][90] for pf in per_frame
           if not math.isnan(pf["rollCorr"][90])]
    rc = [pf["randCorr"] for pf in per_frame if not math.isnan(pf["randCorr"])]
    print(f"\nmedian per-frame trueCorr: {np.median(tc):+.3f}")
    print(f"median per-frame roll90Corr: {np.median(r90):+.3f}")
    for r in ROLLS:
        vals = [pf["rollCorr"][r] for pf in per_frame
                if not math.isnan(pf["rollCorr"][r])]
        print(f"median per-frame roll{r:3d}Corr: {np.median(vals):+.3f}")
    print(f"median per-frame randCorr: {np.median(rc):+.3f}")

    gt = [pf["gap_true"] for pf in per_frame if not math.isnan(pf["gap_true"])]
    gr = [pf["gap_r90"] for pf in per_frame if not math.isnan(pf["gap_r90"])]
    n_true_hold = sum(1 for pf in per_frame if pf["ok_true"])
    n_r90_hold = sum(1 for pf in per_frame if pf["ok_r90"])
    print(f"\nblue-vs-red gap  true: median {np.median(gt):+.3f}  "
          f"holds {n_true_hold}/{len(per_frame)} frames")
    print(f"blue-vs-red gap  roll90: median {np.median(gr):+.3f}  "
          f"holds {n_r90_hold}/{len(per_frame)} frames")

    # -----------------------------------------------------------------------
    # scatter png
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(11, 5))
        ax[0].scatter(pooled["true"][1], pooled["true"][0], s=14, alpha=0.6)
        ax[0].set_title(f"TRUE pose  (Spearman {pooled_true:+.2f})")
        ax[0].set_xlabel("catalog B-V"); ax[0].set_ylabel("measured (B-R)/(B+R)")
        ax[0].axhline(0, color="k", lw=0.5)
        n90 = min(len(pooled_roll[90][0]), len(pooled_roll[90][1]))
        ax[1].scatter(pooled_roll[90][1][:n90], pooled_roll[90][0][:n90],
                      s=14, alpha=0.6, color="tab:red")
        ax[1].set_title(f"ROLL 90  (Spearman {corr(pooled_roll[90]):+.2f})")
        ax[1].set_xlabel("catalog B-V"); ax[1].set_ylabel("measured (B-R)/(B+R)")
        ax[1].axhline(0, color="k", lw=0.5)
        fig.tight_layout()
        fig.savefig(SCATTER_OUT, dpi=110)
        print(f"\nwrote {SCATTER_OUT}")
    except Exception as e:
        print(f"\n(scatter skipped: {e})")

    drifts = [pf["drift"] for pf in per_frame if pf.get("drift") is not None]
    if drifts:
        print(f"\nregistration: full-minute star drift median {np.median(drifts):.2f}px "
              f"(min {min(drifts):.2f}, max {max(drifts):.2f}); "
              f"method e.g. {per_frame[0]['method']}")
    print(f"wrote {CSV_OUT}")


if __name__ == "__main__":
    if "--deep" in sys.argv:
        STACK_MODE = "deep"
        CSV_OUT = "color_probe_results_deepstack.csv"
        SCATTER_OUT = "color_probe_scatter_deepstack.png"
    for a in sys.argv[1:]:
        if a.startswith("--stack"):
            STACK_MODE = a.split("=", 1)[1] if "=" in a else sys.argv[
                sys.argv.index(a) + 1]
            CSV_OUT = f"color_probe_results_stack{STACK_MODE}.csv"
            SCATTER_OUT = f"color_probe_scatter_stack{STACK_MODE}.png"
    if "--noreg" in sys.argv:
        REGISTER = False
    print(f"[stack mode: {STACK_MODE}, register: {REGISTER}]")
    main()
