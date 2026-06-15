#!/usr/bin/env python3
r"""Generate SYNTHETIC plate-solver test cases with exact, free ground truth.

Unlike fits_to_testcases.py (which solves real photos with ASTAP), this renders a
clean synthetic star field from a real catalog (Gaia DR3) at a *chosen* camera pose
(az/el/roll/fov) and known location/time. Because we place every star ourselves, the
named-star pixel positions are exact by construction, and we can systematically sweep
roll, catalog depth and field density -- ideal for stressing the roll-disambiguation /
catalog-depth-instability and for accumulating verifier-calibration data.

Why real catalog stars (not random): the SDRangel solver matches against its own Gaia
DR3 catalog, so the rendered pattern must be a genuine sky region or it can never solve.

Pipeline per scene (label, az, el, roll):
  1. az/el (camera bore-sight) -> RA/Dec at the chosen location/time;
     horizontal roll -> equatorial CROTA2 via the parallactic angle.
  2. build a TAN WCS (same parity as the real Seestar frames: positive-determinant CD,
     validated to solve in native image orientation).
  3. query Gaia DR3 in the field down to a render magnitude limit; project to pixels;
     render Gaussian PSFs with a magnitude->peak (asinh-like) stretch + background/noise.
  4. project the bright Hipparcos stars in the field (HIP labels the solver recognises)
     for the named-star ground truth.
  5. emit star-tests-synthetic.csv rows (one per requested test maxMagnitude), with an
     optional az/el seed jitter to mimic mount-pointing error.

Outputs go to images-synthetic/ and star-tests-synthetic.csv by default.

Usage:
    python synthetic_testcases.py [--mags 12,14,16] [--seed-jitter 0.0]
        [--render-mag 15] [--width 1080 --height 1920 --fov 1.28]
"""
import argparse, math, re
from pathlib import Path
import datetime as _dt

import collections, collections.abc
if not hasattr(collections, "Callable"):
    collections.Callable = collections.abc.Callable  # old keyring on py3.10+

import numpy as np
from astropy.wcs import WCS

# --- astronomy (shared with astap_to_azelroll / fits_to_testcases) ----------------
def julian_date(y, mo, d, h, mi, s):
    jday = ((1461*(y+4800+(mo-14)//12))//4 + (367*(mo-2-12*((mo-14)//12)))//12
            - (3*((y+4900+(mo-14)//12)//100))//4 + d - 32075)
    return jday + (h/24.0-0.5) + mi/1440.0 + s/86400.0
JD2000 = julian_date(2000,1,1,12,0,0)

def lst_deg(jd, lon):
    d=jd-JD2000; f=jd%1.0; ut=(f+0.5)*24.0
    return (100.46+0.985647*d+lon+15.0*ut)%360.0

def norm180(x): return ((x+180.0)%360.0)-180.0
def norm360(x): return x%360.0

def azel_to_radec(az, el, lat, lon, jd):
    """az from north toward east, el above horizon -> (ra_deg, dec_deg) of date."""
    azr,elr,latr = math.radians(az),math.radians(el),math.radians(lat)
    dec = math.asin(math.sin(elr)*math.sin(latr) + math.cos(elr)*math.cos(latr)*math.cos(azr))
    cosha = (math.sin(elr) - math.sin(latr)*math.sin(dec))/(math.cos(latr)*math.cos(dec))
    sinha = -math.sin(azr)*math.cos(elr)/math.cos(dec)
    ha = math.degrees(math.atan2(sinha, max(-1.0,min(1.0,cosha))))
    ra = norm360(lst_deg(jd,lon) - ha)
    return ra, math.degrees(dec), ha

def parallactic_deg(ha_deg, dec_deg, lat):
    hr,dr,lr = math.radians(ha_deg),math.radians(dec_deg),math.radians(lat)
    return math.degrees(math.atan2(math.sin(hr), math.tan(lr)*math.cos(dr)-math.sin(dr)*math.cos(hr)))

def radec_to_azel(ra_deg, dec_deg, lat, lon, jd):
    ha = norm180(lst_deg(jd,lon)-ra_deg)
    dr,lr,hr = math.radians(dec_deg),math.radians(lat),math.radians(ha)
    el = math.asin(math.sin(dr)*math.sin(lr)+math.cos(dr)*math.cos(lr)*math.cos(hr))
    ca = (math.sin(dr)-math.sin(el)*math.sin(lr))/(math.cos(el)*math.cos(lr))
    a = math.degrees(math.acos(max(-1.0,min(1.0,ca))))
    az = a if math.sin(hr)<0.0 else 360.0-a
    return az, math.degrees(el)

def radec_to_azel_arr(ra_deg, dec_deg, lat, lon, jd):
    """Vectorised radec_to_azel (numpy arrays for ra_deg/dec_deg); used for wide/fisheye
    fields where thousands of catalog stars need az/el at once."""
    ha = ((lst_deg(jd,lon)-ra_deg + 180.0) % 360.0) - 180.0
    dr,lr,hr = np.radians(dec_deg),np.radians(lat),np.radians(ha)
    with np.errstate(invalid="ignore", divide="ignore"):
        el = np.arcsin(np.sin(dr)*np.sin(lr)+np.cos(dr)*np.cos(lr)*np.cos(hr))
        ca = (np.sin(dr)-np.sin(el)*np.sin(lr))/(np.cos(el)*np.cos(lr))
        a = np.degrees(np.arccos(np.clip(ca,-1.0,1.0)))
    az = np.where(np.sin(hr)<0.0, a, 360.0-a)
    return az, np.degrees(el)

def precess_j2000_to_date_arr(ra_deg, dec_deg, jd):
    """Vectorised precess_j2000_to_date (numpy arrays for ra_deg/dec_deg)."""
    yrs = (jd - JD2000) / 365.25
    m, n_ra, n_dec = 3.07496, 1.33621, 20.0431   # sec/yr, sec/yr, arcsec/yr
    rr, dr = np.radians(ra_deg), np.radians(dec_deg)
    ra_out = (ra_deg + (m + n_ra*np.sin(rr)*np.tan(dr))*yrs * 15.0/3600.0) % 360.0
    dec_out = dec_deg + (n_dec*np.cos(rr))*yrs / 3600.0
    return ra_out, dec_out

def precess_j2000_to_date(ra_deg, dec_deg, jd):
    """Low-precision (Meeus) general precession of mean J2000 RA/Dec to the epoch of `jd`.
    The Vizier catalogues return J2000 (_RAJ2000/_DEJ2000), but the SDRangel solver works at
    epoch-of-date, so the camera-pointing SEED must be the epoch-of-date direction of the
    field — otherwise every case carries a ~0.1-0.7° (J2000->date precession) seed offset
    that the solver's recenter must absorb (and which, combined with a roll-alias, made the
    densest fields wrongly look like a 'blind-solve' failure class). ~26 years -> ~0.36°."""
    yrs = (jd - JD2000) / 365.25
    m, n_ra, n_dec = 3.07496, 1.33621, 20.0431   # sec/yr, sec/yr, arcsec/yr
    rr, dr = math.radians(ra_deg), math.radians(dec_deg)
    ra_out = norm360(ra_deg + (m + n_ra*math.sin(rr)*math.tan(dr))*yrs * 15.0/3600.0)
    dec_out = dec_deg + (n_dec*math.cos(rr))*yrs / 3600.0
    return ra_out, dec_out

# --- WCS / catalog ----------------------------------------------------------------
def build_wcs(ra_deg, dec_deg, scale_deg, crota2_deg, W, H):
    """TAN WCS centred on the image, positive-determinant CD (matches Seestar parity)."""
    w=WCS(naxis=2)
    w.wcs.crpix=[W/2.0+0.5, H/2.0+0.5]
    w.wcs.crval=[ra_deg, dec_deg]
    w.wcs.ctype=["RA---TAN","DEC--TAN"]
    r=math.radians(crota2_deg); s=scale_deg
    # CDELT1=CDELT2=+s (det>0), rotated by CROTA2 -- empirically the parity the solver
    # accepts in native (un-flipped) image orientation.
    w.wcs.cd=[[ s*math.cos(r), -s*math.sin(r)],
              [ s*math.sin(r),  s*math.cos(r)]]
    return w

def query_catalog(ra,dec,radius_deg, cat, cols, magcol, magmax, limit, retries=4):
    # Resilient to transient Vizier read-timeouts/5xx (common over a long batch): retry with
    # backoff, and return None on persistent failure so the caller can reject/resample the
    # scene instead of crashing the whole run.
    from astroquery.vizier import Vizier
    from astropy.coordinates import SkyCoord
    import astropy.units as u
    import time
    Vizier.TIMEOUT=120
    v=Vizier(columns=cols, catalog=cat, column_filters={magcol:f"<{magmax}"}); v.ROW_LIMIT=limit
    for attempt in range(retries):
        try:
            res=v.query_region(SkyCoord(ra,dec,unit="deg"), radius=radius_deg*u.deg)
            return res[0] if res else None
        except Exception as e:
            print(f"  vizier query failed ({cat}) attempt {attempt+1}/{retries}: {e}", flush=True)
            time.sleep(3*(attempt+1))
    return None

def slugify(s): return re.sub(r"[^A-Za-z0-9]+","-",s).strip("-").lower()

# --- rendering --------------------------------------------------------------------
def mag_to_peak(mag, bright_mag, faint_mag, min_peak=16.0):
    frac=(faint_mag-mag)/max(1e-6,(faint_mag-bright_mag))   # 1 bright, 0 faint
    return float(np.clip(min_peak + frac*(255.0-min_peak), 0.0, 255.0))

# --- wide/fisheye camera projection (mirrors CameraPlateSolver::createProjector /
# projectVector for a square image, aspect=1, no lens distortion / centre offset) -------
def vec_from_altaz_arr(az_deg, el_deg):
    az, el = np.radians(az_deg), np.radians(el_deg)
    ce = np.cos(el)
    return np.stack([ce*np.sin(az), ce*np.cos(az), np.sin(el)], axis=-1)

def make_sky_projector(az_deg, el_deg, roll_deg, fov_deg, lens):
    """Camera basis + projection model for a square image (aspect=1), matching
    cameraplatesolver.cpp's createProjector/projectVector. `lens` is one of
    "rectilinear", "equidistant", "equisolid"."""
    az = math.radians(az_deg)
    center = vec_from_altaz_arr(az_deg, el_deg)
    center = center/np.linalg.norm(center)
    right = np.array([math.cos(az), -math.sin(az), 0.0])
    right = right/np.linalg.norm(right)
    up = np.cross(right, center)
    up = up/np.linalg.norm(up)
    roll = math.radians(roll_deg)
    if abs(roll) > 1e-9:
        def rot(v, axis, angle):
            return (v*math.cos(angle) + np.cross(axis, v)*math.sin(angle)
                    + axis*np.dot(axis, v)*(1.0-math.cos(angle)))
        right = rot(right, center, roll); right /= np.linalg.norm(right)
        up = rot(up, center, roll); up /= np.linalg.norm(up)
    return dict(center=center, right=right, up=up,
                 half_fov=math.radians(fov_deg)*0.5, lens=lens)

def project_points(proj, vecs, W, H):
    """vecs: (N,3) unit vectors. Returns (N,2) pixel coords; NaN rows are behind the camera."""
    depth = vecs @ proj['center']
    planeX = vecs @ proj['right']
    planeY = vecs @ proj['up']
    theta = np.arccos(np.clip(depth, -1.0, 1.0))
    phi = np.arctan2(planeY, planeX)
    lens = proj['lens']
    if lens == "equidistant":
        r = theta/proj['half_fov']
    elif lens == "equisolid":
        r = np.sin(theta*0.5)/np.sin(proj['half_fov']*0.5)
    else:
        r = np.tan(theta)/np.tan(proj['half_fov'])
    px, py = np.cos(phi)*r, np.sin(phi)*r
    x = W*0.5 + px*0.5*W
    y = H*0.5 - py*0.5*H
    out = np.column_stack([x, y])
    out[depth <= 0.0] = np.nan
    return out

def render_field(xy_peak, W, H, sigma=1.15, background=6.0, read_noise=2.0, seed=0):
    rng=np.random.default_rng(seed)
    img=np.full((H,W), background, np.float64)
    half=int(math.ceil(sigma*3.5))
    ax=np.arange(-half,half+1)
    for x,y,peak in xy_peak:
        xi,yi=int(round(x)),int(round(y))
        if xi<-half or yi<-half or xi>=W+half or yi>=H+half: continue
        dx=(xi+ax)-x; dy=(yi+ax)-y
        k=peak*np.exp(-(dx[None,:]**2+dy[:,None]**2)/(2*sigma*sigma))
        x0,x1=max(0,xi-half),min(W,xi+half+1); y0,y1=max(0,yi-half),min(H,yi+half+1)
        kx0,ky0=x0-(xi-half),y0-(yi-half)
        img[y0:y1,x0:x1]+=k[ky0:ky0+(y1-y0), kx0:kx0+(x1-x0)]
    img+=rng.normal(0.0, read_noise, img.shape)
    return np.clip(img,0,255).astype(np.uint8)

def main():
    here=Path(__file__).resolve().parent
    ap=argparse.ArgumentParser()
    ap.add_argument("--out-csv", default=str(here/"star-tests-synthetic.csv"))
    ap.add_argument("--out-images", default=str(here/"images-synthetic"))
    ap.add_argument("--width", type=int, default=1080)
    ap.add_argument("--height", type=int, default=1920)
    ap.add_argument("--fov", type=float, default=1.28, help="long-edge fov degrees")
    ap.add_argument("--render-mag", type=float, default=16.0, help="Gaia depth to render")
    ap.add_argument("--mags", default="12,14,16", help="test maxMagnitudes per scene")
    ap.add_argument("--n-stars", type=int, default=3, help="named HIP stars per case")
    ap.add_argument("--seed-jitter", type=float, default=0.0, help="az/el seed error deg")
    ap.add_argument("--lat", type=float, default=51.5)
    ap.add_argument("--lon", type=float, default=-0.13)
    ap.add_argument("--time", default="2026-09-22 21:30:00", help="UTC")
    # Random-scene generation (for a large tuning corpus). When --random N > 0 the four
    # built-in scenes are replaced by N random pointings, each at a random roll. Pointings
    # are biased to the Milky Way (|galactic b| <= --gal-lat-max) so the field is rich, and
    # rejected if below the horizon (el < --min-el), too sparse (< --min-stars rendered) or
    # carrying no in-frame named HIP star (so every case has usable ground truth).
    ap.add_argument("--random", type=int, default=0, help="generate N random scenes")
    ap.add_argument("--random-seed", type=int, default=20260604, help="RNG seed (reproducible)")
    ap.add_argument("--label", default="synth-rand",
                    help="scene label prefix (use distinct labels + seeds + out-csv per "
                         "parallel instance to a shared --out-images so they don't collide)")
    ap.add_argument("--min-el", type=float, default=20.0, help="reject pointings below this elevation")
    ap.add_argument("--gal-lat-max", type=float, default=22.0, help="max |galactic latitude| deg")
    ap.add_argument("--min-stars", type=int, default=120, help="reject scenes with fewer rendered stars")
    # Deterministic seed-jitter SWEEP (stress dataset). --expand <baseline.csv> re-emits each
    # baseline row plus, for every (magnitude x direction x sign), a copy with the camera SEED
    # perturbed by that offset — same image, same ground truth, only the starting guess moves.
    # This maps the solver's solve-success boundary vs seed-pointing error (the dominant real
    # weakness) without any re-rendering or network. Rows stay in order (baseline then its
    # offsets) so output can be zipped back to offsets for analysis.
    ap.add_argument("--expand", default="", help="baseline CSV to expand into a seed-jitter sweep")
    ap.add_argument("--jitter-sweep", default="0.5,1.0,1.5", help="offset magnitudes (deg)")
    ap.add_argument("--jitter-dirs", default="az,el", help="offset directions: az,el,diag")
    # Synthetic fisheye/wide corpus (Phase 0b of the quad-hash plan, see
    # doc/camera/plate-solver-notes.md): random equidistant/equisolid fisheye scenes with
    # az/el/roll/fov as the camera pose (no WCS/RA-Dec rendering geometry -- the field is
    # projected directly with make_sky_projector/project_points, matching
    # CameraPlateSolver::createProjector/projectVector). Emits both plateSolveStartMode 0
    # (Blind) and 1 (Fov) rows per scene with the SAME true pose (only m_fov is used by
    # mode 1; mode 0 ignores the seed pose entirely).
    ap.add_argument("--fisheye", type=int, default=0, help="generate N random fisheye scenes")
    ap.add_argument("--fisheye-seed", type=int, default=20260612, help="RNG seed (reproducible)")
    ap.add_argument("--fisheye-fov-min", type=float, default=100.0)
    ap.add_argument("--fisheye-fov-max", type=float, default=170.0)
    ap.add_argument("--fisheye-size", type=int, default=1600, help="square image side (px)")
    ap.add_argument("--fisheye-mag", type=float, default=7.0,
                    help="render depth and plateSolveMaxMagnitude for fisheye scenes")
    ap.add_argument("--fisheye-el-min", type=float, default=30.0, help="min bore-sight elevation")
    ap.add_argument("--fisheye-projection", default="mixed",
                    help="equidistant, equisolid or mixed (random per scene)")
    ap.add_argument("--fisheye-min-stars", type=int, default=30,
                    help="reject fisheye scenes with fewer rendered stars")
    ap.add_argument("--refresh-fisheye-anchors", default=None,
                    help="re-derive the named-star validation anchors of an existing fisheye "
                         "CSV (spread+bright selection) WITHOUT re-rendering images; writes the "
                         "updated CSV to --out-csv. Keeps poses and images byte-identical.")
    args=ap.parse_args()

    if args.expand:
        import csv as _csv
        rows=list(_csv.reader(Path(args.expand).read_text().splitlines()))
        mags=[float(m) for m in args.jitter_sweep.split(",") if m.strip()]
        dirs=[d.strip() for d in args.jitter_dirs.split(",") if d.strip()]
        quoted={0,9,13,22}   # image, projection, stars, starPositions (generator format)
        emit=lambda fs: ",".join((f'"{v}"' if i in quoted else v) for i,v in enumerate(fs))
        out=[emit(rows[0])]; n_base=n_off=0
        for r in rows[1:]:
            if len(r)<7: continue
            out.append(emit(r)); n_base+=1                 # baseline (offset 0)
            az0,el0=float(r[5]),float(r[6])
            for m in mags:
                for d in dirs:
                    for s in (1.0,-1.0):
                        daz=dele=0.0
                        if d=="az": daz=s*m
                        elif d=="el": dele=s*m
                        elif d=="diag": daz=s*m/math.sqrt(2); dele=s*m/math.sqrt(2)
                        rr=list(r)
                        rr[5]=f"{(az0+daz)%360.0:.4f}"
                        rr[6]=f"{max(-90.0,min(90.0,el0+dele)):.4f}"
                        out.append(emit(rr)); n_off+=1
        Path(args.out_csv).write_text("\n".join(out)+"\n")
        print(f"expanded {n_base} baseline -> {n_base+n_off} rows "
              f"({len(mags)} mags x {len(dirs)} dirs x 2 signs + baseline) -> {args.out_csv}")
        return

    import cv2

    outimg=Path(args.out_images); outimg.mkdir(exist_ok=True)
    mags=[float(m) for m in args.mags.split(",") if m.strip()]
    t=_dt.datetime.strptime(args.time,"%Y-%m-%d %H:%M:%S")
    jd=julian_date(t.year,t.month,t.day,t.hour,t.minute,t.second)
    long_px=max(args.width,args.height)
    scale=args.fov/long_px                       # deg/pixel
    bright_mag, faint_mag = 4.0, args.render_mag
    rng=np.random.default_rng(1234)

    from astropy.coordinates import SkyCoord

    # Render + ground-truth one scene; returns its CSV rows (one per test magnitude), or
    # None if the scene is rejected (no Gaia stars, too sparse, or — when require_named —
    # no in-frame named HIP star). The image is only written once the scene is accepted.
    def process_scene(label, ra, dec, roll, require_named, min_stars):
        ha=norm180(lst_deg(jd,args.lon)-ra)
        # The field is rendered from J2000 catalogue positions, but the camera-pointing seed
        # the solver consumes must be at epoch-of-date (the solver works at date). Compute the
        # seed az/el from the precessed field centre so there is no built-in seed offset; the
        # WCS / CROTA2 / named-star pixels stay in the rendered (J2000) frame.
        ra_d,dec_d=precess_j2000_to_date(ra,dec,jd)
        az,el=radec_to_azel(ra_d,dec_d,args.lat,args.lon,jd)
        crota2=norm180(roll - parallactic_deg(ha,dec,args.lat))
        wcs=build_wcs(ra,dec,scale,crota2,args.width,args.height)
        flag="" if el>=15 else "  (LOW ELEVATION)"
        print(f"\n=== {label}  RA/Dec={ra/15:.3f}h/{dec:+.2f}  roll={roll:.1f}  -> az/el={az:.1f}/{el:.1f}{flag}  CROTA2={crota2:.2f}")

        gcat=query_catalog(ra,dec, radius_deg=args.fov*0.62, cat="I/355/gaiadr3",
                           cols=["_RAJ2000","_DEJ2000","Gmag"], magcol="Gmag",
                           magmax=args.render_mag, limit=50000)
        if gcat is None or len(gcat)==0:
            print("  no Gaia stars; skipping"); return None
        gsky=SkyCoord(np.asarray(gcat["_RAJ2000"],float), np.asarray(gcat["_DEJ2000"],float), unit="deg")
        gx,gy=wcs.world_to_pixel(gsky)
        gmag=np.asarray(gcat["Gmag"],float)
        xy_peak=[(float(x),float(y),mag_to_peak(m,bright_mag,faint_mag))
                 for x,y,m in zip(gx,gy,gmag)
                 if -5<=x<args.width+5 and -5<=y<args.height+5 and np.isfinite(m)]
        if len(xy_peak) < min_stars:
            print(f"  only {len(xy_peak)} in-frame stars (< {min_stars}); skipping"); return None

        # named HIP stars (solver-recognised labels), exact positions. Skip the very
        # brightest (saturated, hard to label); relax the window if the field is sparse.
        # A picked star must have a rendered Gaia counterpart: the frame is rendered from
        # Gaia DR3, which is missing/unreliable for bright stars (G <~ 6), so a HIP pick
        # without a nearby rendered star would put ground truth on empty sky that no
        # solver can ever label (seen with HIP 92043 / 110 Her, V=4.19, in rand2 c-023).
        def pick_named(vmin, vmax):
            cat=query_catalog(ra,dec, radius_deg=args.fov*0.55, cat="I/239/hip_main",
                              cols=["HIP","Vmag","_RAJ2000","_DEJ2000"], magcol="Vmag",
                              magmax=vmax, limit=120)
            picked=[]
            if cat is not None:
                for r in sorted(cat, key=lambda r:float(r["Vmag"])):
                    if float(r["Vmag"])<vmin: continue
                    hx,hy=wcs.world_to_pixel(SkyCoord(float(r["_RAJ2000"]),float(r["_DEJ2000"]),unit="deg"))
                    X,Y=float(hx),float(hy)
                    if X<25 or Y<25 or X>=args.width-25 or Y>=args.height-25: continue
                    if not any(abs(x-X)<=2.0 and abs(y-Y)<=2.0 for x,y,_ in xy_peak):
                        print(f"  skipping HIP {int(r['HIP'])} (V={float(r['Vmag']):.2f}): no rendered Gaia counterpart at ({X:.0f},{Y:.0f})")
                        continue
                    picked.append((int(r["HIP"]),X,Y))
                    if len(picked)>=args.n_stars: break
            return picked
        chosen=pick_named(3.5,11.0) or pick_named(0.0,12.5)
        if not chosen:
            if require_named:
                print("  no in-frame named HIP star; skipping"); return None
            print("  WARNING: no in-frame HIP named stars; case has empty starPositions")

        img=render_field(xy_peak,args.width,args.height,seed=int(rng.integers(1e9)))
        jpg=outimg/f"{label}.jpg"
        cv2.imwrite(str(jpg),img,[int(cv2.IMWRITE_JPEG_QUALITY),95])
        print(f"  rendered {len(xy_peak)} Gaia stars (<=mag {args.render_mag}) -> {jpg.name}")

        # az/el seed (optionally jittered to mimic mount-pointing error)
        jx=args.seed_jitter
        az_s=az + (float(rng.normal(0,jx)) if jx>0 else 0.0)
        el_s=el + (float(rng.normal(0,jx)) if jx>0 else 0.0)
        names=",".join(f"HIP {h}" for h,_,_ in chosen)
        spos=",".join(f"HIP {h}:{int(round(X))}:{int(round(Y))}" for h,X,Y in chosen)
        print(f"  named: {spos if spos else '(none)'}")
        srows=[]
        img_dir_name=Path(args.out_images).name
        for mag in mags:
            srows.append(f'"{img_dir_name}/{label}.jpg",{args.time},{args.lat:.6f},'
                         f'{args.lon:.6f},0.0,{az_s:.2f},{el_s:.2f},0.0,{args.fov:.3f},'
                         f'"rectilinear",0.0,0.0,0.0,"{names}",{mag:g},3,0,0,0,0,,,"{spos}"')
        return srows

    # Render + ground-truth one fisheye/wide scene (az/el/roll/fov IS the camera pose,
    # projected directly via make_sky_projector/project_points -- no WCS/TAN involved).
    # Returns CSV rows for plateSolveStartMode 0 (Blind) and 1 (Fov), or None if rejected.
    def process_fisheye_scene(label, az, el, roll, fov_deg, lens, size, max_mag, min_stars,
                              render=True):
        ra_d, dec_d, _ha = azel_to_radec(az, el, args.lat, args.lon, jd)
        print(f"\n=== {label}  az/el={az:.1f}/{el:.1f}  roll={roll:.1f}  fov={fov_deg:.1f} ({lens})")

        radius = min(fov_deg*0.5*math.sqrt(2.0)+2.0, 90.0)
        gcat=query_catalog(ra_d,dec_d, radius_deg=radius, cat="I/355/gaiadr3",
                           cols=["_RAJ2000","_DEJ2000","Gmag"], magcol="Gmag",
                           magmax=max_mag, limit=20000)
        if gcat is None or len(gcat)==0:
            print("  no Gaia stars; skipping"); return None

        ra_j=np.asarray(gcat["_RAJ2000"],float); dec_j=np.asarray(gcat["_DEJ2000"],float)
        gmag=np.asarray(gcat["Gmag"],float)
        ra_dt,dec_dt=precess_j2000_to_date_arr(ra_j,dec_j,jd)
        saz,sel=radec_to_azel_arr(ra_dt,dec_dt,args.lat,args.lon,jd)
        vecs=vec_from_altaz_arr(saz,sel)

        proj=make_sky_projector(az,el,roll,fov_deg,lens)
        pix=project_points(proj,vecs,size,size)
        gx,gy=pix[:,0],pix[:,1]

        valid=np.isfinite(gx)&np.isfinite(gy)&np.isfinite(gmag)&(gx>=-5)&(gx<size+5)&(gy>=-5)&(gy<size+5)
        xy_peak=[(float(x),float(y),mag_to_peak(m,0.0,max_mag))
                 for x,y,m in zip(gx[valid],gy[valid],gmag[valid])]
        if len(xy_peak) < min_stars:
            print(f"  only {len(xy_peak)} in-frame stars (< {min_stars}); skipping"); return None

        # named HIP stars: brightest, with a rendered Gaia counterpart, away from the frame
        # edge (the equidistant/equisolid model is undefined past the FOV circle).
        radius_named=min(fov_deg*0.5*math.sqrt(2.0), 90.0)
        def pick_named(vmax, limit):
            cat=query_catalog(ra_d,dec_d, radius_deg=radius_named, cat="I/239/hip_main",
                              cols=["HIP","Vmag","_RAJ2000","_DEJ2000"], magcol="Vmag",
                              magmax=vmax, limit=limit)
            # Collect ALL qualifying named-star candidates (bright, in-frame, away from
            # the image edge, with a rendered Gaia counterpart), brightest-first. The
            # qualifying test is unchanged from the original so scene acceptance - and
            # therefore the rendered images/poses driven by the RNG - stay identical.
            candidates=[]   # (vmag, HIP, X, Y)
            if cat is not None and len(cat)>0:
                ra_h=np.asarray(cat["_RAJ2000"],float); dec_h=np.asarray(cat["_DEJ2000"],float)
                ra_hd,dec_hd=precess_j2000_to_date_arr(ra_h,dec_h,jd)
                haz,hel=radec_to_azel_arr(ra_hd,dec_hd,args.lat,args.lon,jd)
                hvecs=vec_from_altaz_arr(haz,hel)
                hpix=project_points(proj,hvecs,size,size)
                vmags=np.asarray(cat["Vmag"],float)
                for i in np.argsort(vmags):
                    X,Y=float(hpix[i,0]),float(hpix[i,1])
                    if not (np.isfinite(X) and np.isfinite(Y)): continue
                    if X<25 or Y<25 or X>=size-25 or Y>=size-25: continue
                    if not any(abs(x-X)<=2.0 and abs(y-Y)<=2.0 for x,y,_ in xy_peak):
                        continue
                    candidates.append((float(vmags[i]),int(cat["HIP"][i]),X,Y))
            if not candidates:
                return []
            # Select n_stars anchors that are BRIGHT *and* spatially SPREAD, via
            # farthest-point sampling within a bright candidate pool, preferring the
            # reliable interior. Tightly-clustered anchors make a weak validation
            # oracle: a wrong pose can satisfy 3 stars grouped within ~150px (this is
            # exactly what let synth-fisheye-027/039/042 "pass" at wrong poses). Spread
            # anchors pin the pose. Selection only re-ranks the qualifying candidates,
            # so it never changes acceptance.
            cx=cy=size/2.0
            interior_r=0.45*size
            pool=candidates[:max(args.n_stars*5, 15)]
            interior=[c for c in pool if math.hypot(c[2]-cx, c[3]-cy) <= interior_r]
            source=interior if len(interior) >= args.n_stars else pool
            chosen_c=[source[0]]   # brightest in the preferred set
            while len(chosen_c) < args.n_stars and len(chosen_c) < len(source):
                best=None; best_d=-1.0
                for c in source:
                    if c in chosen_c: continue
                    d=min((c[2]-p[2])**2+(c[3]-p[3])**2 for p in chosen_c)
                    if d > best_d: best_d=d; best=c
                chosen_c.append(best)
            return [(h,X,Y) for _v,h,X,Y in chosen_c]
        chosen=pick_named(max_mag,3000) or pick_named(12.0,8000)
        if not chosen:
            print("  no in-frame named HIP star; skipping"); return None

        if render:
            img=render_field(xy_peak,size,size,seed=int(rng.integers(1e9)))
            jpg=outimg/f"{label}.jpg"
            cv2.imwrite(str(jpg),img,[int(cv2.IMWRITE_JPEG_QUALITY),95])
            print(f"  rendered {len(xy_peak)} Gaia stars (<=mag {max_mag}) -> {jpg.name}")
        else:
            print(f"  anchor-refresh: {len(xy_peak)} in-frame Gaia stars (image kept)")

        names=",".join(f"HIP {h}" for h,_,_ in chosen)
        spos=",".join(f"HIP {h}:{int(round(X))}:{int(round(Y))}" for h,X,Y in chosen)
        print(f"  named: {spos}")

        proj_name="equidistant fisheye" if lens=="equidistant" else "equisolid fisheye"
        img_dir_name=Path(args.out_images).name
        srows=[]
        for mode in (0,1):
            srows.append(f'"{img_dir_name}/{label}.jpg",{args.time},{args.lat:.6f},'
                         f'{args.lon:.6f},0.0,{az:.2f},{el:.2f},{roll:.2f},{fov_deg:.3f},'
                         f'"{proj_name}",0.0,0.0,0.0,"{names}",{max_mag:g},{mode},0,0,0,0,,,"{spos}"')
        return srows

    if args.refresh_fisheye_anchors:
        # Re-derive only the named-star validation anchors (spread+bright) of an existing
        # fisheye CSV, re-using its poses and leaving the rendered images untouched. Each
        # raw line is preserved byte-for-byte except its stars (col 13) and starPositions
        # (col 22) fields, which are replaced with the freshly-selected anchors.
        #
        # Resumable + per-scene resilient: the heavy Vizier queries (one wide near-full-sky
        # Gaia cone per scene) are flaky over a 50-scene batch and can exceed a single run's
        # wall-clock budget, so each scene's result is checkpointed to a JSON cache as soon
        # as it is computed. Re-run the same command to resume; a scene whose query fails is
        # left for a later run (its original anchors are kept until then).
        import csv as _csv, json
        src=Path(args.refresh_fisheye_anchors)
        raw_lines=src.read_text().splitlines()
        parsed=list(_csv.reader(raw_lines))
        cache_path=Path(args.out_csv+".anchorcache.json")
        cache=json.loads(cache_path.read_text()) if cache_path.exists() else {}

        labels=[]; seen=set(); pose={}; orig={}
        for idx in range(1,len(raw_lines)):
            if not raw_lines[idx].strip():
                continue
            row=parsed[idx]; lab=Path(row[0]).stem
            orig.setdefault(lab,(row[13],row[22]))
            if lab not in seen:
                seen.add(lab); labels.append(lab)
                pose[lab]=(float(row[5]),float(row[6]),float(row[7]),float(row[8]),
                           "equidistant" if "equidistant" in row[9] else "equisolid",
                           float(row[14]))
        for lab in labels:
            if lab in cache:
                continue
            az,el,roll,fov,lens,mmag=pose[lab]
            try:
                srows=process_fisheye_scene(lab,az,el,roll,fov,lens,
                                            args.fisheye_size,mmag,args.fisheye_min_stars,
                                            render=False)
            except Exception as e:
                print(f"  ERROR {lab}: {e}"); srows=None
            if srows:
                nrow=next(_csv.reader([srows[0]]))
                cache[lab]=[nrow[13],nrow[22]]
                cache_path.write_text(json.dumps(cache))   # checkpoint after each scene
            else:
                print(f"  WARNING: {lab} produced no anchors this run; will retry on re-run")

        out=[raw_lines[0]]
        for idx in range(1,len(raw_lines)):
            if not raw_lines[idx].strip():
                continue
            row=parsed[idx]; lab=Path(row[0]).stem
            os_,op_=orig[lab]
            if lab in cache:
                nn,ns=cache[lab]
                out.append(raw_lines[idx].replace(f'"{os_}"',f'"{nn}"').replace(f'"{op_}"',f'"{ns}"'))
            else:
                out.append(raw_lines[idx])
        Path(args.out_csv).write_text("\n".join(out)+"\n")
        print(f"\nRefreshed {len(cache)}/{len(labels)} scenes -> {args.out_csv}")
        return

    rows=[]
    if args.random > 0:
        # Random pointings biased to the Milky Way (|galactic b| <= gal-lat-max) so the
        # field is rich; above the horizon; rich enough; with a named HIP star. The
        # galactic-latitude and elevation rejections are free (no network), so only viable
        # pointings cost a catalog query.
        srng=np.random.default_rng(args.random_seed)
        accepted=0; attempts=0; maxatt=args.random*40
        while accepted < args.random and attempts < maxatt:
            attempts+=1
            ra=float(srng.uniform(0.0,360.0))
            dec=math.degrees(math.asin(float(srng.uniform(-1.0,1.0))))   # uniform on sphere
            if abs(float(SkyCoord(ra,dec,unit="deg").galactic.b.deg)) > args.gal_lat_max:
                continue
            az,el=radec_to_azel(ra,dec,args.lat,args.lon,jd)
            if el < args.min_el:
                continue
            roll=float(srng.uniform(0.0,360.0))
            srows=process_scene(f"{args.label}-{accepted+1:03d}", ra, dec, roll,
                                require_named=True, min_stars=args.min_stars)
            if srows:
                rows.extend(srows); accepted+=1
        print(f"\nAccepted {accepted}/{args.random} random scenes in {attempts} attempts")
    else:
        # Built-in rich scenes at a few rolls (RA/Dec given; az/el derived at the
        # configured location/time). Edit/extend freely.
        scenes=[
            dict(label="synth-cyg-r0",  ra=305.5, dec=40.2, roll=0.0),    # Cygnus (dense)
            dict(label="synth-cyg-r60", ra=305.5, dec=40.2, roll=60.0),   # same field, rolled
            dict(label="synth-cas-r0",  ra=14.0,  dec=60.7, roll=0.0),    # Cassiopeia (dense)
            dict(label="synth-cas-r90", ra=14.0,  dec=60.7, roll=90.0),   # same field, rolled
        ]
        for sc in scenes:
            srows=process_scene(sc["label"], sc["ra"], sc["dec"], sc["roll"],
                                require_named=False, min_stars=1)
            if srows:
                rows.extend(srows)

    if args.fisheye > 0:
        frng=np.random.default_rng(args.fisheye_seed)
        accepted=0; attempts=0; maxatt=args.fisheye*40
        while accepted < args.fisheye and attempts < maxatt:
            attempts+=1
            az=float(frng.uniform(0.0,360.0))
            el=float(frng.uniform(args.fisheye_el_min,89.0))
            roll=float(frng.uniform(0.0,360.0))
            fov=float(frng.uniform(args.fisheye_fov_min,args.fisheye_fov_max))
            if args.fisheye_projection=="mixed":
                lens="equidistant" if frng.integers(0,2)==0 else "equisolid"
            else:
                lens=args.fisheye_projection
            srows=process_fisheye_scene(f"synth-fisheye-{accepted+1:03d}", az, el, roll, fov,
                                        lens, args.fisheye_size, args.fisheye_mag,
                                        args.fisheye_min_stars)
            if srows:
                rows.extend(srows); accepted+=1
        print(f"\nAccepted {accepted}/{args.fisheye} fisheye scenes in {attempts} attempts")

    header=("image,time,latitude,longitude,altitude,azimuth,elevation,roll,fov,projection,"
            "cx,cy,k1,stars,plateSolveMaxMagnitude,plateSolveStartMode,roiX,roiY,roiW,roiH,"
            "expectedRoll,expectedRollTolerance,starPositions")
    Path(args.out_csv).write_text(header+"\n"+"\n".join(rows)+"\n")
    print(f"\nWrote {len(rows)} rows to {args.out_csv}")

if __name__=="__main__":
    main()
