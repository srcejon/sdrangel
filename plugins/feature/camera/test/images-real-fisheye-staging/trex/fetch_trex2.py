#!/usr/bin/env python3
"""Fetch TREx RGB stream0 frames for the Tier-2 plate-solver corpus (trex2 set).

For each (site, date, ut-hour) candidate:
  - download <YYYYMMDD>_<HH>00_<site>_<cam>_full.h5 (the HH:00 minute file, 20 frames)
    from https://data.phys.ucalgary.ca/sort_by_project/TREx/RGB/stream0/
  - take the frame whose timestamp is nearest second :30,
  - convert RGB -> grayscale (channel mean),
  - apply the linear stretch DN[median-1, median+30] -> [0, 255]
    (this exact stretch; naive percentile stretch was a known bug),
  - save trex_<site>_<YYYYMMDD>_<HHMMSS>utc.png at native resolution (553x480),
  - score star-like local maxima (for clear/cloudy triage; final keep is by eye).

Run from the trex staging dir:  python fetch_trex2.py
"""
import os
import sys
import urllib.request
import numpy as np
import h5py
from PIL import Image
from scipy.io import readsav

BASE = "https://data.phys.ucalgary.ca/sort_by_project/TREx/RGB/stream0"

CAM = {"luck": "rgb-03", "rabb": "rgb-06", "pina": "rgb-02", "yknf": "rgb-08",
       "gill": "rgb-04"}

# (site, YYYYMMDD, UT hour). New nights only — existing corpus nights are
# 2025-12-20, 2026-01-15, 2026-01-18, 2026-02-14, 2026-02-17.
CANDIDATES = [
    ("luck", "20251218", 6), ("luck", "20251223", 6), ("luck", "20260120", 6),
    ("luck", "20260219", 6), ("luck", "20260317", 6),
    ("rabb", "20251218", 6), ("rabb", "20251223", 6), ("rabb", "20260120", 6),
    ("rabb", "20260219", 6), ("rabb", "20260317", 6),
    ("pina", "20251218", 6), ("pina", "20251223", 6), ("pina", "20260120", 6),
    ("yknf", "20251223", 8), ("yknf", "20260120", 8), ("yknf", "20260219", 8),
    # wave 2: extra UT hours on nights wave 1 showed to be clear (roll
    # diversity within a night, like the existing 2025-12-20 luck set),
    # plus additional new nights for the sites still short of keepers.
    ("luck", "20251223", 9), ("luck", "20260219", 9), ("luck", "20260120", 9),
    ("luck", "20260112", 6), ("luck", "20260210", 6),
    ("rabb", "20251218", 9), ("rabb", "20260112", 6), ("rabb", "20260210", 6),
    ("pina", "20260112", 6), ("pina", "20260210", 6), ("pina", "20260122", 6),
    ("yknf", "20260112", 8), ("yknf", "20260210", 8), ("yknf", "20251216", 8),
    # wave 3: pina/rabb/yknf still short of keepers
    ("pina", "20251216", 6), ("pina", "20260125", 6), ("pina", "20260215", 6),
    ("rabb", "20251216", 6), ("rabb", "20260125", 6),
    ("yknf", "20251218", 8), ("yknf", "20260125", 8),
]


def download(site, date, hour):
    cam = CAM[site]
    fname = f"{date}_{hour:02d}00_{site}_{cam}_full.h5"
    if os.path.exists(fname):
        return fname
    url = (f"{BASE}/{date[:4]}/{date[4:6]}/{date[6:8]}/"
           f"{site}_{cam}/ut{hour:02d}/{fname}")
    try:
        with urllib.request.urlopen(url, timeout=120) as r, \
                open(fname + ".part", "wb") as out:
            out.write(r.read())
        os.replace(fname + ".part", fname)
        return fname
    except Exception as e:
        print(f"  MISSING {url}: {e}")
        return None


def stretch(gray):
    """The corpus stretch: DN[median-1, median+30] -> [0,255], clipped."""
    med = float(np.median(gray))
    lo, hi = med - 1.0, med + 30.0
    out = (gray - lo) * (255.0 / (hi - lo))
    return np.clip(np.rint(out), 0, 255).astype(np.uint8)


def starlike_peaks(img8, elev_mask):
    """Count 3x3 strict local maxima above a fixed stretched-DN threshold."""
    a = img8.astype(np.int16)
    core = a[1:-1, 1:-1]
    ismax = np.ones(core.shape, bool)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            ismax &= core > a[1 + dy:a.shape[0] - 1 + dy,
                              1 + dx:a.shape[1] - 1 + dx]
    ismax &= core >= 40
    ismax &= elev_mask[1:-1, 1:-1]
    return int(ismax.sum())


def elev_mask_for(site):
    s = readsav(f"skymap_{site}.sav").skymap[0]
    return np.asarray(s.full_elevation, dtype=np.float32) > 5.0


def process(site, date, hour, masks):
    fname = download(site, date, hour)
    if fname is None:
        return None
    with h5py.File(fname, "r") as f:
        imgs = f["data/images"][:]          # (480, 553, 3, 20)
        ts = [t.decode() if isinstance(t, bytes) else str(t)
              for t in f["data/timestamp"][:]]
    # frame nearest second :30
    secs = []
    for t in ts:
        hms = t.split()[1]
        h, m, s = hms.split(":")
        secs.append(float(s))
    idx = int(np.argmin([abs(s - 30.0) for s in secs]))
    tstr = ts[idx]
    gray = imgs[:, :, :, idx].astype(np.float32).mean(axis=2)
    img8 = stretch(gray)
    if site not in masks:
        masks[site] = elev_mask_for(site)
    peaks = starlike_peaks(img8, masks[site])
    hms = tstr.split()[1].split(".")[0].replace(":", "")
    png = f"trex_{site}_{date}_{hms}utc.png"
    Image.fromarray(img8).save(png)
    return png, tstr, peaks, float(np.median(gray))


def main():
    masks = {}
    print(f"{'png':46s} {'raw med':>7s} {'peaks':>5s}")
    rows = []
    for site, date, hour in CANDIDATES:
        r = process(site, date, hour, masks)
        if r is None:
            continue
        png, tstr, peaks, med = r
        rows.append((png, tstr, peaks, med))
        print(f"{png:46s} {med:7.1f} {peaks:5d}")
    # also score the existing corpus frames with the same metric, as a
    # clear-sky baseline
    print("\n-- existing corpus frames (same metric) --")
    import glob
    for png in sorted(glob.glob("trex_????_20*utc.png")):
        if any(png == r[0] for r in rows):
            continue
        site = png.split("_")[1]
        if site not in masks:
            masks[site] = elev_mask_for(site)
        img8 = np.array(Image.open(png))
        print(f"{png:46s} {'':7s} {starlike_peaks(img8, masks[site]):5d}")


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    main()
