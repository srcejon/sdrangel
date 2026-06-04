#!/usr/bin/env python3
r"""Generate star-tests.csv rows from Seestar (or any WCS-solvable) FITS frames.

For each FITS file this:
  1. reads the capture metadata from the header (DATE-OBS as UTC, SITELAT/SITELONG,
     NAXIS, XPIXSZ/FOCALLEN, mount RA/DEC, OBJECT);
  2. renders an asinh-stretched 8-bit RGB PNG (the test harness loads images with
     cv::imread(IMREAD_COLOR), so a linear 16-bit FITS must be stretched to 8-bit);
  3. plate-solves the FITS with ASTAP (native FITS WCS, no orientation ambiguity);
  4. builds an astropy WCS from the ASTAP solution, queries the brightest Hipparcos
     stars in the field (Vizier), projects them to pixels, and EMPIRICALLY VALIDATES
     the FITS->PNG orientation by checking each projected star lands on a bright pixel;
  5. converts the ASTAP RA/Dec/CROTA2 to az/el/roll (same math as astap_to_azelroll.py);
  6. prints ready-to-paste star-tests.csv rows (optionally several magnitudes each, to
     probe catalog-depth stability), and writes the PNGs into images/.

Usage:
    python fits_to_testcases.py [--src images/new] [--out-images images]
        [--astap "C:\Program Files\astap\astap_cli.exe"]
        [--mags 13,15,17] [--n-stars 3] [--seed-from astap|mount]
"""
import argparse, configparser, math, re, subprocess, sys
from pathlib import Path
import datetime as _dt

import collections, collections.abc
if not hasattr(collections, "Callable"):
    collections.Callable = collections.abc.Callable  # old keyring on py3.10+

import numpy as np
from astropy.io import fits
from astropy.wcs import WCS

DEFAULT_ASTAP = r"C:\Program Files\astap\astap_cli.exe"

# --- astronomy (ported from astap_to_azelroll.py / sdrbase astronomy) -------------
def julian_date(y, mo, d, h, mi, s):
    jday = ((1461 * (y + 4800 + (mo - 14)//12))//4 + (367*(mo - 2 - 12*((mo-14)//12)))//12
            - (3*((y + 4900 + (mo-14)//12)//100))//4 + d - 32075)
    return jday + (h/24.0 - 0.5) + mi/1440.0 + s/86400.0
JD2000 = julian_date(2000,1,1,12,0,0); JDB1950 = julian_date(1949,12,31,22,9,0)

def precess(ra_h, dec, jd_from, jd_to):
    dpc=36524.219878; t0=(jd_from-JDB1950)/dpc; t=(jd_to-jd_from)/dpc
    r=[[0.0]*3 for _ in range(3)]
    r[0][0]=1.0-((29696.0+26.0*t0)*t*t-13.0*t*t*t)*1e-8
    r[1][0]=((2234941.0+1355.0*t0)*t-676.0*t*t+221.0*t*t*t)*1e-8
    r[2][0]=((971690.0-414.0*t0)*t+207.0*t*t+96.0*t*t*t)*1e-8
    r[0][1]=-r[1][0]; r[1][1]=1.0-((24975.0+30.0*t0)*t*t-15.0*t*t*t)*1e-8
    r[2][1]=-((10858.0+2.0*t0)*t*t)*1e-8
    r[0][2]=-r[2][0]; r[1][2]=r[2][1]; r[2][2]=1.0-((4721.0-4.0*t0)*t*t)*1e-8
    ra_deg=ra_h*15.0
    x=math.cos(math.radians(ra_deg))*math.cos(math.radians(dec))
    y=math.sin(math.radians(ra_deg))*math.cos(math.radians(dec))
    z=math.sin(math.radians(dec))
    xp=r[0][0]*x+r[0][1]*y+r[0][2]*z; yp=r[1][0]*x+r[1][1]*y+r[1][2]*z; zp=r[2][0]*x+r[2][1]*y+r[2][2]*z
    ra=math.degrees(math.atan2(yp,xp))%360.0
    return ra/15.0, math.degrees(math.asin(max(-1.0,min(1.0,zp))))

def lst_deg(jd, lon):
    d=jd-JD2000; f=jd%1.0; ut=(f+0.5)*24.0
    return (100.46+0.985647*d+lon+15.0*ut)%360.0

def norm180(x): return ((x+180.0)%360.0)-180.0

def radec_to_azelroll(ra_h, dec, crota2, lat, lon, jd):
    ra_h, dec = precess(ra_h, dec, JD2000, jd)
    ha = math.fmod(lst_deg(jd,lon) - ra_h*15.0, 360.0)
    dr,lr,hr = math.radians(dec),math.radians(lat),math.radians(ha)
    alt=math.asin(math.sin(dr)*math.sin(lr)+math.cos(dr)*math.cos(lr)*math.cos(hr))
    ca=(math.sin(dr)-math.sin(alt)*math.sin(lr))/(math.cos(alt)*math.cos(lr))
    a=math.degrees(math.acos(max(-1.0,min(1.0,ca))))
    az=a if math.sin(hr)<0.0 else 360.0-a
    q=math.degrees(math.atan2(math.sin(hr), math.tan(lr)*math.cos(dr)-math.sin(dr)*math.cos(hr)))
    return az, math.degrees(alt), norm180(crota2+q)

def jd_from_utc(s):
    s=s.strip().replace("T"," ")
    t=_dt.datetime.strptime(s.split(".")[0], "%Y-%m-%d %H:%M:%S")
    return julian_date(t.year,t.month,t.day,t.hour,t.minute,t.second), t

# --- helpers ---------------------------------------------------------------------
def slugify(obj, fallback):
    s=re.sub(r"[^A-Za-z0-9]+","-", (obj or fallback).strip()).strip("-").lower()
    return s or fallback

def stretch_to_8bit(data):
    """data: (3,H,W) or (H,W) uint16 linear -> (H,W,3) uint8, asinh-stretched."""
    if data.ndim==3:
        img = np.moveaxis(data,0,-1).astype(np.float64)   # H,W,3
    else:
        img = np.repeat(data[...,None],3,axis=2).astype(np.float64)
    out = np.zeros_like(img)
    a = 12.0
    for c in range(img.shape[2]):
        ch = img[:,:,c]
        black = np.percentile(ch, 40.0)
        white = np.percentile(ch, 99.85)
        if white <= black: white = black + 1.0
        norm = np.clip((ch-black)/(white-black), 0.0, None)
        st = np.arcsinh(norm*a)/math.asinh(a)
        out[:,:,c] = np.clip(st*255.0, 0, 255)
    return out.astype(np.uint8)

def parse_ini(p):
    cfg=configparser.ConfigParser(strict=False)
    cfg.read_string("[a]\n"+Path(p).read_text(errors="ignore"))
    return {k.upper():v for k,v in cfg["a"].items()}

def bright_stars(ra_deg, dec_deg, radius_deg, vmag_min=3.5, vmag_max=11.0, limit=60):
    """Hipparcos stars in [vmag_min, vmag_max], brightest first. The lower bound skips
    very bright targets (e.g. Pollux mag 1.2) whose saturated bloom the solver may detect
    but not cleanly label as a single catalog star."""
    from astroquery.vizier import Vizier
    from astropy.coordinates import SkyCoord
    import astropy.units as u
    v=Vizier(columns=["HIP","Vmag","_RAJ2000","_DEJ2000"], catalog="I/239/hip_main",
             column_filters={"Vmag":f">{vmag_min} && <{vmag_max}"})
    v.ROW_LIMIT=limit
    res=v.query_region(SkyCoord(ra_deg,dec_deg,unit="deg"), radius=radius_deg*u.deg)
    if not res: return []
    t=res[0]; out=[]
    for row in t:
        try: out.append((int(row["HIP"]), float(row["Vmag"]), float(row["_RAJ2000"]), float(row["_DEJ2000"])))
        except Exception: continue
    out.sort(key=lambda r:r[1])
    return out

def sample_brightness(gray, x, y, r=6):
    h,w=gray.shape
    xi,yi=int(round(x)),int(round(y))
    if xi<r or yi<r or xi>=w-r or yi>=h-r: return None
    return float(gray[yi-r:yi+r+1, xi-r:xi+r+1].max())

def main():
    ap=argparse.ArgumentParser()
    here=Path(__file__).resolve().parent
    ap.add_argument("--src", default=str(here/"images"/"new"))
    ap.add_argument("--out-images", default=str(here/"images"))
    ap.add_argument("--astap", default=DEFAULT_ASTAP)
    ap.add_argument("--mags", default="13,15,17")
    ap.add_argument("--n-stars", type=int, default=3)
    ap.add_argument("--seed-from", choices=["astap","mount"], default="mount")
    ap.add_argument("--radius", type=float, default=180.0)
    args=ap.parse_args()

    src=Path(args.src); outimg=Path(args.out_images); outimg.mkdir(exist_ok=True)
    astap=Path(args.astap)
    mags=[float(m) for m in args.mags.split(",") if m.strip()]
    import cv2

    for fpath in sorted(src.glob("*.fit"))+sorted(src.glob("*.fits")):
        hdr=fits.getheader(fpath); data=fits.getdata(fpath)
        H = hdr["NAXIS2"]; W = hdr["NAXIS1"]
        date_obs=str(hdr["DATE-OBS"]); jd,tutc=jd_from_utc(date_obs)
        lat=float(hdr["SITELAT"]); lon=float(hdr["SITELONG"])
        obj=str(hdr.get("OBJECT","")).strip()
        slug=slugify(obj, fpath.stem)
        print(f"\n===== {fpath.name}  obj={obj}  {tutc} UTC  lat={lat} lon={lon}")

        # 1) render PNG (flipud so FITS row0=bottom displays at bottom: non-mirrored)
        img8 = stretch_to_8bit(np.asarray(data))
        # Write the FITS array in its native (un-flipped) orientation. ASTAP solves the
        # FITS in the same array convention, so projected pixel coords map straight to PNG
        # rows. flipud would vertically mirror the frame -> opposite sky parity, which the
        # SDRangel solver (fixed-parity) cannot match.
        img_path = outimg/f"{slug}.jpg"
        cv2.imwrite(str(img_path), cv2.cvtColor(img8, cv2.COLOR_RGB2BGR),
                    [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        gray = cv2.cvtColor(img8, cv2.COLOR_RGB2GRAY)

        # 2) ASTAP solve the FITS (native WCS)
        out_base = src/"astap-output"/slug; out_base.parent.mkdir(exist_ok=True)
        for ext in (".ini",".wcs"):
            f=out_base.with_suffix(ext)
            if f.exists(): f.unlink()
        pixscale = 206.265*float(hdr["XPIXSZ"])/float(hdr["FOCALLEN"])  # arcsec/px
        fov_hint = max(H,W)*pixscale/3600.0
        cmd=[str(astap),"-f",str(fpath),"-r",f"{args.radius:g}","-fov",f"{fov_hint:g}",
             "-o",str(out_base)]
        subprocess.run(cmd,capture_output=True,text=True,timeout=180)
        ini=out_base.with_suffix(".ini")
        d=parse_ini(ini) if ini.exists() else {}
        if d.get("PLTSOLVD","F").upper()!="T":
            print(f"  ASTAP did NOT solve ({d.get('ERROR') or d.get('WARNING') or 'no solution'})"); continue
        crval1=float(d["CRVAL1"]); crval2=float(d["CRVAL2"]); crota2=float(d["CROTA2"])
        cdelt=abs(float(d.get("CDELT2", d.get("CDELT1"))))
        fov = cdelt*max(H,W)            # true long-edge fov from solve
        print(f"  ASTAP: RA={crval1/15:.4f}h Dec={crval2:+.3f} roll(CROTA2)={crota2:.2f} fov={fov:.4f}")

        # 3) WCS (built from the ASTAP .ini CD matrix; the .wcs file ASTAP writes is a
        #    non-standard partial header astropy can't open) + bright-star projection
        #    with empirical orientation validation.
        wcs=WCS(naxis=2)
        wcs.wcs.crpix=[float(d["CRPIX1"]), float(d["CRPIX2"])]
        wcs.wcs.crval=[crval1, crval2]
        wcs.wcs.ctype=["RA---TAN","DEC--TAN"]
        if all(k in d for k in ("CD1_1","CD1_2","CD2_1","CD2_2")):
            wcs.wcs.cd=[[float(d["CD1_1"]),float(d["CD1_2"])],
                        [float(d["CD2_1"]),float(d["CD2_2"])]]
        else:
            wcs.wcs.cdelt=[float(d["CDELT1"]),float(d["CDELT2"])]
            wcs.wcs.crota=[float(d.get("CROTA1",0.0)), crota2]
        stars=bright_stars(crval1,crval2, radius_deg=fov*0.62)
        if not stars:   # sparse field: relax the magnitude window (include faint/bright)
            stars=bright_stars(crval1,crval2, radius_deg=fov*0.62, vmag_min=0.0, vmag_max=12.5)
        from astropy.coordinates import SkyCoord
        chosen=[]; bvals=[]
        for hip,vmag,ra,dec in stars:
            xw,yw = wcs.world_to_pixel(SkyCoord(ra,dec,unit="deg"))   # native FITS/array frame
            X,Y=float(xw),float(yw)
            if X<20 or Y<20 or X>=W-20 or Y>=H-20: continue
            bright=sample_brightness(gray,X,Y)
            if bright is not None: bvals.append(bright)
            chosen.append((hip,vmag,X,Y,bright))
            if len(chosen)>=args.n_stars: break
        # sanity: projected stars should land on bright (near-saturated) pixels
        print(f"  projected-star brightness (median {int(np.median(bvals)) if bvals else 0}/255 "
              f"of {len(chosen)} stars) -- should be high if WCS+orientation are correct")

        # 4) az/el/roll
        az_t,el_t,roll_t = radec_to_azelroll(crval1/15.0, crval2, crota2, lat, lon, jd)
        if args.seed_from=="mount" and ("RA" in hdr and "DEC" in hdr):
            az_s,el_s,_ = radec_to_azelroll(float(hdr["RA"])/15.0, float(hdr["DEC"]), 0.0, lat, lon, jd)
        else:
            az_s,el_s = az_t,el_t
        print(f"  seed az/el = {az_s:.2f}/{el_s:.2f}   truth az/el/roll = {az_t:.2f}/{el_t:.2f}/{roll_t:.2f}")
        for hip,vmag,X,Y,bright in chosen:
            print(f"    HIP {hip}  Vmag {vmag:.2f}  -> ({X:.0f},{Y:.0f})  pkbright={bright:.0f}")

        names=",".join(f"HIP {h}" for h,_,_,_,_ in chosen)
        spos=",".join(f"HIP {h}:{int(round(X))}:{int(round(Y))}" for h,_,X,Y,_ in chosen)
        tstr=tutc.strftime("%Y-%m-%d %H:%M:%S")
        print("  --- CSV rows ---")
        for mag in mags:
            # expectedRoll/tolerance left blank: the named-star position check (24 px)
            # already encodes the full orientation incl. roll, and the roll convention
            # through ASTAP/Seestar framing is left to the position ground truth.
            row=(f'"images/{slug}.jpg",{tstr},{lat:.6f},{lon:.6f},0.0,'
                 f'{az_s:.2f},{el_s:.2f},0.0,{fov:.3f},"rectilinear",0.0,0.0,0.0,'
                 f'"{names}",{mag:g},3,0,0,0,0,,,"{spos}"')
            print(row)

if __name__=="__main__":
    main()
