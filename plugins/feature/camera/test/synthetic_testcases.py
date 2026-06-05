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
    ap.add_argument("--min-el", type=float, default=20.0, help="reject pointings below this elevation")
    ap.add_argument("--gal-lat-max", type=float, default=22.0, help="max |galactic latitude| deg")
    ap.add_argument("--min-stars", type=int, default=120, help="reject scenes with fewer rendered stars")
    args=ap.parse_args()
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
        az,el=radec_to_azel(ra,dec,args.lat,args.lon,jd)
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
            srows=process_scene(f"synth-rand-{accepted+1:03d}", ra, dec, roll,
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

    header=("image,time,latitude,longitude,altitude,azimuth,elevation,roll,fov,projection,"
            "cx,cy,k1,stars,plateSolveMaxMagnitude,plateSolveStartMode,roiX,roiY,roiW,roiH,"
            "expectedRoll,expectedRollTolerance,starPositions")
    Path(args.out_csv).write_text(header+"\n"+"\n".join(rows)+"\n")
    print(f"\nWrote {len(rows)} rows to {args.out_csv}")

if __name__=="__main__":
    main()
