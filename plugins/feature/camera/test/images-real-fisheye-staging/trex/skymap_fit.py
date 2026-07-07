#!/usr/bin/env python3
"""Skymap-as-fixed-intrinsics — FEASIBILITY probe.

Question: can the plate solver's PARAMETRIC fisheye model (projection type + fov +
principal-point cx/cy + a single k1) reproduce a real TREx all-sky lens (its skymap
per-pixel az/el calibration) to ~pixel accuracy? If yes, we can feed fitted fixed
intrinsics through the existing projector and the true pose will fit tightly. If the
best fit still leaves large residuals, the single-k1 model is the wall and only a
richer model / lookup table would help.

Replicates cameraplatesolvercore.cpp createProjector/projectVector EXACTLY (equidistant/
equisolid/rectilinear; k1 radial on normalized radius; anisotropic vertical scale=aspect;
pp = size/2 + offset). Fits (boresightAz, boresightEl, roll, halfHFov, cx, cy, k1) per
projection type to minimize pixel residual over sampled skymap pixels.

Usage: skymap_fit.py [site ...]   (default: all four skymaps)
"""
import math, sys, glob, os
import numpy as np
from scipy.io import readsav
from scipy.optimize import least_squares


def vector_from_altaz(az_deg, el_deg):
    a = np.radians(az_deg); e = np.radians(el_deg); ce = np.cos(e)
    return np.stack([ce * np.sin(a), ce * np.cos(a), np.sin(e)], axis=-1)


def normalize(v):
    return v / np.linalg.norm(v, axis=-1, keepdims=True)


def rodrigues(v, axis, ang):
    axis = axis / np.linalg.norm(axis)
    c, s = math.cos(ang), math.sin(ang)
    return v * c + np.cross(axis, v) * s + axis * np.dot(v, axis) * (1 - c)


PROJ = {"equidistant": 0, "equisolid": 1, "rectilinear": 2}


def project(vecs, p, W, H, proj):
    """vecs: (N,3) sky unit vectors. p = (bAz,bEl,roll,halfHFov,cx,cy,k1). -> (N,2) pixels."""
    bAz, bEl, roll, halfHFov, cx, cy, k1 = p
    center = normalize(vector_from_altaz(bAz, bEl))
    azR = math.radians(bAz)
    right = np.array([math.cos(azR), -math.sin(azR), 0.0]); right = right / np.linalg.norm(right)
    up = np.cross(right, center); up = up / np.linalg.norm(up)
    if abs(roll) > 1e-12:
        right = rodrigues(right, center, math.radians(roll))
        up = rodrigues(up, center, math.radians(roll))
    depth = vecs @ center
    theta = np.arccos(np.clip(depth, -1.0, 1.0))
    phi = np.arctan2(vecs @ up, vecs @ right)
    if proj == 0:      # equidistant
        r = theta / halfHFov
    elif proj == 1:    # equisolid
        r = np.sin(theta * 0.5) / math.sin(halfHFov * 0.5)
    else:              # rectilinear
        r = np.tan(theta) / math.tan(halfHFov)
    px = np.cos(phi) * r
    py = np.sin(phi) * r
    if abs(k1) > 1e-12:
        scale = 1.0 + k1 * (px * px + py * py)
        px = px * scale; py = py * scale
    aspect = H / W
    nx = px / 1.0
    ny = py / aspect
    ppx = W * 0.5 + cx
    ppy = H * 0.5 + cy
    X = ppx + MIRROR[0] * nx * 0.5 * W   # MIRROR[0] = -1 flips handedness (camera looking up)
    Y = ppy - ny * 0.5 * H
    return np.stack([X, Y], axis=-1), depth


MIRROR = [1]  # set to -1 to test a horizontally-mirrored image (up-looking all-sky camera)


def fit_site(site):
    s = readsav(f"skymap_{site}.sav").skymap[0]
    el = np.asarray(s.full_elevation, dtype=np.float64)
    az = np.asarray(s.full_azimuth, dtype=np.float64)
    H, W = el.shape
    valid = np.isfinite(el) & np.isfinite(az) & (el > 5.0)
    iy, ix = np.nonzero(valid)
    # subsample to ~4000 points, spread across the frame
    if len(ix) > 4000:
        sel = np.linspace(0, len(ix) - 1, 4000).astype(int)
        iy, ix = iy[sel], ix[sel]
    obs = np.stack([ix.astype(float), iy.astype(float)], axis=-1)
    vecs = vector_from_altaz(az[iy, ix], el[iy, ix])
    elvals = el[iy, ix]

    print(f"\n=== {site}: skymap {W}x{H}, {int(valid.sum())} valid px, fitting {len(ix)} samples "
          f"(el {elvals.min():.1f}..{elvals.max():.1f}) ===")

    # Unbounded LM from a near-zenith start. NOTE: the good fit is often reached via a
    # mathematically-equivalent flipped/wrapped basin (bEl~-90, wrapped halfHFov) due to the
    # az/roll degeneracy at zenith + a handedness quirk; the RESIDUAL is what's trustworthy and
    # comparable across projection types, not the raw param values.
    # bounded to the PHYSICAL basin (boresight near zenith el 85..90) so the params are
    # trustworthy; loop projection x mirror-handedness. cx/cy +-150px, k1 +-0.6, fov 130..185 long-edge.
    lo = np.array([-180.0, 85.0, -180.0, math.radians(65.0), -150.0, -150.0, -0.6])
    hi = np.array([ 180.0, 90.0,  180.0, math.radians(92.5),  150.0,  150.0,  0.6])
    best = None
    for mirror in (1, -1):
        for name, proj in PROJ.items():
            def resid(p):
                MIRROR[0] = mirror
                pred, depth = project(vecs, p, W, H, proj)
                r = (pred - obs).ravel()
                return np.where(np.isfinite(r), r, 1e6)
            best_local = None
            for roll0 in (0.0, 90.0, 180.0, -90.0):
                p0 = np.clip(np.array([0.0, 89.0, roll0, math.radians(85.0), 0.0, 0.0, 0.0]), lo, hi)
                try:
                    res = least_squares(resid, p0, bounds=(lo, hi), method="trf", max_nfev=6000)
                    MIRROR[0] = mirror
                    pred, _ = project(vecs, res.x, W, H, proj)
                    d = np.linalg.norm(pred - obs, axis=-1); d = d[np.isfinite(d)]
                    med = np.median(d)
                    if best_local is None or med < best_local[0]:
                        best_local = (med, np.percentile(d, 90), np.percentile(d, 99), d.max(), res.x)
                except Exception:
                    pass
            if best_local is None:
                continue
            med, p90, p99, mx, x = best_local
            hf, cx, cy, k1 = math.degrees(x[3]), x[4], x[5], x[6]
            tag = "MIRRORED" if mirror < 0 else "normal  "
            print(f"  {name:12s} {tag} median={med:6.2f} p90={p90:6.2f} p99={p99:6.2f} max={mx:6.2f}   "
                  f"[halfHFov={hf:.1f} cx={cx:.1f} cy={cy:.1f} k1={k1:.4f} bEl={x[1]:.2f}]")
            if best is None or med < best[1]:
                best = (f"{name}/{tag.strip()}", med, (med, p90, p99, mx), x)
    return site, best


def main():
    sites = sys.argv[1:]
    if not sites:
        sites = sorted(os.path.basename(f)[7:-4] for f in glob.glob("skymap_*.sav"))
    print(f"Skymap-as-fixed-intrinsics feasibility: fitting the solver's parametric model to {sites}")
    results = []
    for site in sites:
        try:
            results.append(fit_site(site))
        except Exception as e:
            print(f"{site}: ERROR {e}")
    print("\n=== SUMMARY (best projection per site) ===")
    for site, best in results:
        if best:
            print(f"  {site:6s} best={best[0]:12s} median residual = {best[1]:.2f} px")
    print("\nVERDICT GUIDE: median residual <~2px => parametric model is adequate, feed fitted "
          "fixed intrinsics through the existing projector (tractable). >~10px => the single-k1 "
          "model is the wall; would need k2/k3 or a lookup-table projector (bigger job).")


if __name__ == "__main__":
    main()
