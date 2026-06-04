#!/usr/bin/env python3
"""Verify that the images referenced by star-tests.csv can be plate-solved by ASTAP.

For each unique image in the CSV this runs the ASTAP command-line solver
(astap_cli.exe) and reports whether ASTAP solved it, how long it took, and the
solution (RA/Dec/roll). It is independent of SDRangel's own solver -- it is a
cross-check that the test images are solvable by a reference solver.

Notes
-----
* ASTAP is a rectilinear/SIP solver; it cannot solve the wide fisheye images
  (projection != rectilinear). Those are still attempted and reported, but are
  excluded from the "rectilinear" pass count since a failure there is expected.
* By default the solve is FOV-hinted but otherwise blind (all-sky, -r 180): no
  RA/Dec hint is supplied, so this tests that ASTAP can solve the image from
  scratch. The FOV hint is taken from the CSV `fov` column (ASTAP's -fov is the
  field's long dimension in degrees, which matches our long-edge fov). For wide
  (> 30 deg) / fisheye fields the hint is replaced by auto (-fov 0).
* ASTAP writes `<name>.ini` / `<name>.wcs`; we redirect them to an
  `astap-output/` directory next to the CSV via -o so the image folder stays
  clean. Solved flag = `PLTSOLVD=T` in the .ini.

Usage
-----
    python astap_solve_tests.py [star-tests.csv]
        [--astap "C:\\Program Files\\astap\\astap_cli.exe"]
        [--radius 180] [--timeout 180] [--blind] [--max-stars 500]

Exit code 0 if every unique image solved, else 1.
"""

import argparse
import configparser
import csv
import subprocess
import sys
import time
from collections import OrderedDict
from pathlib import Path

DEFAULT_ASTAP = r"C:\Program Files\astap\astap_cli.exe"
# Above this field size ASTAP's rectilinear FOV hint is meaningless (our wide
# cases are fisheye); fall back to auto-FOV so an absurd value isn't passed.
MAX_RECTILINEAR_FOV_DEG = 30.0
FISHEYE_TOKENS = ("fisheye", "equidistant", "equisolid")


def parse_ini(path: Path) -> dict:
    """ASTAP .ini files are key=value with no section header."""
    cfg = configparser.ConfigParser(strict=False)
    try:
        cfg.read_string("[astap]\n" + path.read_text(errors="ignore"))
    except configparser.Error:
        return {}
    return {k.upper(): v for k, v in cfg["astap"].items()}


def main() -> int:
    ap = argparse.ArgumentParser(description="Verify star-tests.csv images solve with ASTAP")
    ap.add_argument("csv", nargs="?",
                    default=str(Path(__file__).with_name("star-tests.csv")))
    ap.add_argument("--astap", default=DEFAULT_ASTAP)
    ap.add_argument("--radius", type=float, default=180.0,
                    help="ASTAP search radius in degrees (default 180 = all-sky)")
    ap.add_argument("--timeout", type=float, default=180.0,
                    help="per-image timeout in seconds")
    ap.add_argument("--blind", action="store_true",
                    help="force auto FOV detection (-fov 0) for every image")
    ap.add_argument("--max-stars", type=int, default=500)
    args = ap.parse_args()

    csv_path = Path(args.csv).resolve()
    if not csv_path.exists():
        print(f"CSV not found: {csv_path}", file=sys.stderr)
        return 2
    astap = Path(args.astap)
    if not astap.exists():
        print(f"ASTAP not found: {astap}", file=sys.stderr)
        return 2

    csv_dir = csv_path.parent
    out_dir = csv_dir / "astap-output"
    out_dir.mkdir(exist_ok=True)

    # Group the (possibly duplicated) test rows by image file.
    images = OrderedDict()
    with csv_path.open(newline="") as fh:
        for row in csv.DictReader(fh):
            img = (row.get("image") or "").strip().strip('"')
            if not img:
                continue
            info = images.setdefault(img, {"fovs": [], "proj": set(), "rows": 0})
            info["rows"] += 1
            try:
                info["fovs"].append(float(row.get("fov", "")))
            except ValueError:
                pass
            info["proj"].add((row.get("projection") or "").strip().strip('"').lower())

    print(f"ASTAP: {astap}")
    print(f"CSV  : {csv_path}  ({len(images)} unique images, "
          f"{sum(i['rows'] for i in images.values())} test rows)")
    print(f"Mode : {'blind auto-FOV' if args.blind else 'FOV-hinted'}, "
          f"radius {args.radius:g} deg, timeout {args.timeout:g}s\n")
    print(f"{'image':24}{'rows':>5}{'fov':>7}{'fish':>5}{'ASTAP':>7}{'sec':>8}   detail")
    print("-" * 100)

    results = []
    for img, info in images.items():
        img_path = (csv_dir / img).resolve()
        name = img_path.stem
        proj = ",".join(sorted(p for p in info["proj"] if p))
        fisheye = any(tok in proj for tok in FISHEYE_TOKENS)
        nominal_fov = max(info["fovs"]) if info["fovs"] else 0.0
        if args.blind or fisheye or nominal_fov > MAX_RECTILINEAR_FOV_DEG:
            fov_hint = 0.0
        else:
            fov_hint = nominal_fov

        out_base = out_dir / name
        for ext in (".ini", ".wcs", ".log"):
            f = out_base.with_suffix(ext)
            if f.exists():
                f.unlink()

        rec = {"img": name, "rows": info["rows"], "fov": nominal_fov,
               "fisheye": fisheye, "ok": False, "sec": 0.0, "detail": ""}

        if not img_path.exists():
            rec["detail"] = "image missing"
            results.append(rec)
            _print_row(rec)
            continue

        cmd = [str(astap), "-f", str(img_path), "-r", f"{args.radius:g}",
               "-fov", f"{fov_hint:g}", "-s", str(args.max_stars),
               "-o", str(out_base)]
        t0 = time.time()
        timed_out = False
        try:
            subprocess.run(cmd, capture_output=True, text=True, timeout=args.timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
        rec["sec"] = time.time() - t0

        ini = out_base.with_suffix(".ini")
        data = parse_ini(ini) if ini.exists() else {}
        rec["ok"] = data.get("PLTSOLVD", "F").upper() == "T"
        if rec["ok"]:
            try:
                ra_h = float(data["CRVAL1"]) / 15.0
                dec = float(data["CRVAL2"])
                rot = float(data.get("CROTA2", "nan"))
                rec["detail"] = f"RA={ra_h:7.3f}h  Dec={dec:+7.2f}  roll={rot:7.2f}"
            except (KeyError, ValueError):
                rec["detail"] = "solved"
        elif timed_out:
            rec["detail"] = f"TIMEOUT (> {args.timeout:g}s)"
        else:
            rec["detail"] = (data.get("ERROR") or data.get("WARNING")
                             or "not solved")[:48]
        results.append(rec)
        _print_row(rec)

    print("-" * 100)
    total = len(results)
    solved = sum(1 for r in results if r["ok"])
    rect = [r for r in results if not r["fisheye"]]
    rect_solved = sum(1 for r in rect if r["ok"])
    print(f"ASTAP solved {solved}/{total} unique images.")
    print(f"  rectilinear: {rect_solved}/{len(rect)} solved "
          f"(fisheye images excluded -- ASTAP is rectilinear/SIP only).")
    failed = [r["img"] for r in results if not r["ok"]]
    if failed:
        print("  not solved: " + ", ".join(failed))
    return 0 if solved == total else 1


def _print_row(rec: dict) -> None:
    status = "OK" if rec["ok"] else "FAIL"
    fov = f"{rec['fov']:.2f}" if rec["fov"] else "auto"
    fish = "Y" if rec["fisheye"] else ""
    print(f"{rec['img']:24}{rec['rows']:>5}{fov:>7}{fish:>5}{status:>7}"
          f"{rec['sec']:>8.1f}   {rec['detail']}")


if __name__ == "__main__":
    sys.exit(main())
