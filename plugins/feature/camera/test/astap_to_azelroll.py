#!/usr/bin/env python3
r"""Convert ASTAP plate solutions to camera azimuth / elevation / roll.

ASTAP solves in the equatorial (J2000) frame: each <image>.ini produced by
astap_solve_tests.py gives CRVAL1/CRVAL2 (RA/Dec in degrees) and CROTA2 (field
rotation about the optical axis, measured from celestial north). Using the
observer position (lat/lon) and date-time from star-tests.csv, this script
converts each solution to the horizontal frame used by the camera feature:
azimuth (from north toward east), elevation (above the horizon) and roll (about
the optical axis, measured from the local vertical).

The result is the "true" az/el/roll for each image and can be used to populate or
validate the azimuth/elevation/roll columns of star-tests.csv.

Conventions match sdrbase/util/astronomy.h (so the output matches what the C++
solver/harness expects):
  * the CSV date-time is UTC (the harness stores UTC and parses it as Qt::UTC),
    so no machine-timezone conversion is applied;
  * RA/Dec are precessed J2000 -> epoch-of-date (Astronomy::precess);
  * az/el use the Astronomy::raDecToAzAlt formula (az from north, east-positive);
  * roll = normalize(CROTA2 + parallactic_angle).

The roll convention was calibrated against known cases: galaxy-m51 (~0 deg),
galaxy-m101 (87.2 deg) and stars-narrow-2 (34.2 deg, which matches the SDRangel
solver's own solved roll), all reproduced to better than ~0.3 deg.

Usage:
    python astap_to_azelroll.py [star-tests.csv]
        [--ini-dir astap-output] [--out generated-azelroll.csv]
"""

import argparse
import configparser
import csv
import math
from pathlib import Path
import datetime as _dt

# --- Julian date / sidereal time / precession (ported from sdrbase/util/astronomy) ---

def julian_date(y, mo, d, h, mi, s):
    jday = ((1461 * (y + 4800 + (mo - 14) // 12)) // 4
            + (367 * (mo - 2 - 12 * ((mo - 14) // 12))) // 12
            - (3 * ((y + 4900 + (mo - 14) // 12) // 100)) // 4
            + d - 32075)
    return jday + (h / 24.0 - 0.5) + mi / 1440.0 + s / 86400.0

JD2000 = julian_date(2000, 1, 1, 12, 0, 0)
JDB1950 = julian_date(1949, 12, 31, 22, 9, 0)


def precess(ra_h, dec, jd_from, jd_to):
    dpc = 36524.219878
    t0 = (jd_from - JDB1950) / dpc
    t = (jd_to - jd_from) / dpc
    r = [[0.0] * 3 for _ in range(3)]
    r[0][0] = 1.0 - ((29696.0 + 26.0 * t0) * t * t - 13.0 * t * t * t) * 1e-8
    r[1][0] = ((2234941.0 + 1355.0 * t0) * t - 676.0 * t * t + 221.0 * t * t * t) * 1e-8
    r[2][0] = ((971690.0 - 414.0 * t0) * t + 207.0 * t * t + 96.0 * t * t * t) * 1e-8
    r[0][1] = -r[1][0]
    r[1][1] = 1.0 - ((24975.0 + 30.0 * t0) * t * t - 15.0 * t * t * t) * 1e-8
    r[2][1] = -((10858.0 + 2.0 * t0) * t * t) * 1e-8
    r[0][2] = -r[2][0]
    r[1][2] = r[2][1]
    r[2][2] = 1.0 - ((4721.0 - 4.0 * t0) * t * t) * 1e-8
    ra_deg = ra_h * 15.0
    x = math.cos(math.radians(ra_deg)) * math.cos(math.radians(dec))
    y = math.sin(math.radians(ra_deg)) * math.cos(math.radians(dec))
    z = math.sin(math.radians(dec))
    xp = r[0][0] * x + r[0][1] * y + r[0][2] * z
    yp = r[1][0] * x + r[1][1] * y + r[1][2] * z
    zp = r[2][0] * x + r[2][1] * y + r[2][2] * z
    ra = math.degrees(math.atan2(yp, xp)) % 360.0
    return ra / 15.0, math.degrees(math.asin(max(-1.0, min(1.0, zp))))


def local_sidereal_time_deg(jd, lon):
    d = jd - JD2000
    f = jd % 1.0
    ut = (f + 0.5) * 24.0
    return (100.46 + 0.985647 * d + lon + 15.0 * ut) % 360.0


def normalize_deg(x):
    return ((x + 180.0) % 360.0) - 180.0


def radec_to_azelroll(ra_h, dec, crota2, lat, lon, jd):
    """Return (az[0..360), el, roll) in degrees. roll = CROTA2 + parallactic."""
    ra_h, dec = precess(ra_h, dec, JD2000, jd)
    lst = local_sidereal_time_deg(jd, lon)
    ha = math.fmod(lst - ra_h * 15.0, 360.0)
    dec_r, lat_r, ha_r = math.radians(dec), math.radians(lat), math.radians(ha)
    alt = math.asin(math.sin(dec_r) * math.sin(lat_r)
                    + math.cos(dec_r) * math.cos(lat_r) * math.cos(ha_r))
    cos_a = (math.sin(dec_r) - math.sin(alt) * math.sin(lat_r)) / (math.cos(alt) * math.cos(lat_r))
    a = math.degrees(math.acos(max(-1.0, min(1.0, cos_a))))
    az = a if math.sin(ha_r) < 0.0 else 360.0 - a
    q = math.degrees(math.atan2(math.sin(ha_r),
                                math.tan(lat_r) * math.cos(dec_r) - math.sin(dec_r) * math.cos(ha_r)))
    roll = normalize_deg(crota2 + q)
    return az, math.degrees(alt), roll


def jd_from_utc_string(s):
    """Parse 'YYYY-MM-DD HH:MM:SS' as UTC, return Julian date.

    star-tests.csv stores timestamps in UTC (see the time-normalisation in the
    test harness), so no timezone conversion is applied here -- which is what makes
    the result machine-timezone-independent.
    """
    t = _dt.datetime.strptime(s.strip(), "%Y-%m-%d %H:%M:%S")
    return julian_date(t.year, t.month, t.day, t.hour, t.minute, t.second)


def parse_ini(path):
    cfg = configparser.ConfigParser(strict=False)
    try:
        cfg.read_string("[a]\n" + path.read_text(errors="ignore"))
    except configparser.Error:
        return {}
    return {k.upper(): v for k, v in cfg["a"].items()}


def main():
    ap = argparse.ArgumentParser(description="ASTAP RA/Dec -> camera az/el/roll using CSV position+time")
    ap.add_argument("csv", nargs="?", default=str(Path(__file__).with_name("star-tests.csv")))
    ap.add_argument("--ini-dir", default=None, help="dir with ASTAP <image>.ini (default <csv_dir>/astap-output)")
    ap.add_argument("--out", default=None, help="write generated az/el/roll to this CSV")
    args = ap.parse_args()

    csv_path = Path(args.csv).resolve()
    csv_dir = csv_path.parent
    ini_dir = Path(args.ini_dir).resolve() if args.ini_dir else (csv_dir / "astap-output")

    # one entry per unique image (metadata is constant across that image's rows)
    images = {}
    order = []
    with csv_path.open(newline="") as fh:
        for row in csv.DictReader(fh):
            img = (row.get("image") or "").strip().strip('"')
            if not img or img in images:
                continue
            order.append(img)
            images[img] = row

    hdr = (f"{'image':24}{'el':>8}{'az':>9}{'roll':>9}    "
           f"{'csv_el':>7}{'csv_az':>8}{'csv_roll':>9}    {'d_el':>6}{'d_az':>7}{'d_roll':>7}")
    print(hdr)
    print("-" * len(hdr))

    out_rows = []
    for img in order:
        row = images[img]
        name = Path(img).stem
        ini = ini_dir / (name + ".ini")
        data = parse_ini(ini) if ini.exists() else {}
        if data.get("PLTSOLVD", "F").upper() != "T":
            print(f"{name:24}   (no ASTAP solution{' - .ini missing' if not ini.exists() else ''})")
            continue
        try:
            ra_h = float(data["CRVAL1"]) / 15.0
            dec = float(data["CRVAL2"])
            crota2 = float(data["CROTA2"])
            lat = float(row["latitude"]); lon = float(row["longitude"])
            jd = jd_from_utc_string(row["time"])
        except (KeyError, ValueError) as e:
            print(f"{name:24}   (skipped: {e})")
            continue
        az, el, roll = radec_to_azelroll(ra_h, dec, crota2, lat, lon, jd)

        def fnum(v):
            try:
                return float(v)
            except (TypeError, ValueError):
                return None
        cel, caz, croll = fnum(row.get("elevation")), fnum(row.get("azimuth")), fnum(row.get("roll"))
        d_el = f"{el - cel:+6.2f}" if cel is not None else "    -"
        d_az = f"{normalize_deg(az - caz):+7.2f}" if caz is not None else "      -"
        d_roll = f"{normalize_deg(roll - croll):+7.2f}" if croll is not None else "      -"
        print(f"{name:24}{el:8.2f}{az:9.2f}{roll:9.2f}    "
              f"{cel if cel is not None else 0:7.1f}{caz if caz is not None else 0:8.1f}"
              f"{croll if croll is not None else 0:9.1f}    {d_el:>6}{d_az:>7}{d_roll:>7}")
        out_rows.append({"image": img, "azimuth": f"{az:.4f}", "elevation": f"{el:.4f}",
                         "roll": f"{roll:.4f}", "ra_hours": f"{ra_h:.6f}", "dec_deg": f"{dec:.6f}"})

    if args.out and out_rows:
        with open(args.out, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=["image", "azimuth", "elevation", "roll", "ra_hours", "dec_deg"])
            w.writeheader()
            w.writerows(out_rows)
        print(f"\nWrote {len(out_rows)} rows to {args.out}")


if __name__ == "__main__":
    main()
