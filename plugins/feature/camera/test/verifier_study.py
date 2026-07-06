#!/usr/bin/env python3
"""Verifier study (Tier 1): label per-candidate dumps against corpus truth and evaluate
discriminator features offline.

Input: a harness log produced with SDRANGEL_CAMERA_PLATE_SOLVER_CANDIDATE_DUMP=1 and Qt debug
logging enabled, plus the corpus CSV that produced it. CandidateDump lines carry raw quantities
(pose, D/C/M, match-tightness ladder m2/m4/m8/m16, rms/med/max, radius, image size, bright stats);
features are engineered here, offline, so the solver never grows another tuned constant until a
feature demonstrably separates every labelled failure class.

Labelling: a candidate is RIGHT if its boresight is within max(1deg, 0.15*fov_true) of the row's
truth direction, its fov is within [0.77, 1.3] of truth, and (when the row carries a roll truth)
its roll is within 10deg. Rows without a usable truth direction (blind rows with az=el=0) are
skipped. Negative rows (expectSolved=0) label every candidate WRONG.

Output: labelled candidate CSV + per-class separation summary for each feature.
"""
import csv
import math
import re
import sys

DUMP_RE = re.compile(r"CandidateDump,(.*)$")
VERDICT_RE = re.compile(r"^(PASS|FAIL) (\S+?):")
POSE_RE = re.compile(
    r"poseAz=([-0-9.eE+]+) poseEl=([-0-9.eE+]+) poseRoll=([-0-9.eE+]+) poseFov=([-0-9.eE+]+)")


def parse_dump_fields(payload):
    fields = {}
    for part in payload.strip().split(","):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key] = value
    return fields


def ang_sep_deg(az1, el1, az2, el2):
    a1, e1, a2, e2 = map(math.radians, (az1, el1, az2, el2))
    c = math.sin(e1) * math.sin(e2) + math.cos(e1) * math.cos(e2) * math.cos(a1 - a2)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def roll_delta_deg(a, b):
    d = abs(a - b) % 360.0
    return 360.0 - d if d > 180.0 else d


def load_truth(csv_path):
    """Row order -> truth dict. Uses the harness CSV column names."""
    rows = []
    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            expect_solved = (row.get("expectSolved") or "1").strip().lower()
            truth = {
                "image": (row.get("image") or "").split("/")[-1],
                "az": float(row.get("azimuth") or 0.0),
                "el": float(row.get("elevation") or 0.0),
                "roll": row.get("expectedRoll") or row.get("roll") or "",
                "fov": float(row.get("fov") or 0.0),
                "negative": expect_solved in ("0", "false", "no"),
                # az=el=0 marks blind rows whose CSV carries no truth direction
                "hasDirection": not (float(row.get("azimuth") or 0) == 0.0
                                     and float(row.get("elevation") or 0) == 0.0),
                "trueFov": float(row.get("fov") or 0.0),
            }
            try:
                truth["rollVal"] = float(truth["roll"])
                truth["hasRoll"] = truth["roll"] != ""
            except ValueError:
                truth["rollVal"] = 0.0
                truth["hasRoll"] = False
            rows.append(truth)
    return rows


def label_candidate(truth, cand):
    """Return 'RIGHT', 'WRONG', or None (unlabellable)."""
    if truth["negative"]:
        return "WRONG"
    if not truth["hasDirection"] or truth["trueFov"] <= 0.0:
        return None
    tol = max(1.0, 0.15 * truth["trueFov"])
    if ang_sep_deg(cand["az"], cand["el"], truth["az"], truth["el"]) > tol:
        return "WRONG"
    ratio = cand["fov"] / truth["trueFov"] if truth["trueFov"] > 0 else 0.0
    if not (0.77 <= ratio <= 1.30):
        return "WRONG"
    if truth["hasRoll"] and roll_delta_deg(cand["roll"], truth["rollVal"]) > 10.0:
        return "WRONG"
    # Only a certified truth (PASS-case accepted pose, i.e. anchor-verified, incl. roll) may mint
    # a RIGHT label. On FAIL cases the roll truth is unknown, so a candidate passing the available
    # checks might still be a wrong-roll pose — leave it unlabelled rather than pollute RIGHT.
    return "RIGHT" if truth.get("certified") else None


def chance_lambda(D, C, area, radius):
    if area <= 0 or D <= 0:
        return 0.0
    p = 1.0 - math.exp(-C * math.pi * radius * radius / area)
    return D * p


def signed_poisson_llr(observed, lam):
    if lam <= 0.0:
        return 0.0 if observed == 0 else float(observed)  # matches against zero chance: strong
    if observed <= 0:
        return -lam
    llr = observed * math.log(observed / lam) - (observed - lam)
    return llr if observed >= lam else -llr


def parse_residuals(raw):
    """res=x:y:dx:dy;... -> list of (x, y, dx, dy)."""
    out = []
    for chunk in raw.split(";"):
        parts = chunk.split(":")
        if len(parts) != 4:
            continue
        try:
            out.append(tuple(float(p) for p in parts))
        except ValueError:
            continue
    return out


def solve_least_squares(rows_a, rows_b):
    """Tiny normal-equations solver: minimize |A p - b|; returns p or None."""
    n = len(rows_a[0])
    ata = [[0.0] * n for _ in range(n)]
    atb = [0.0] * n
    for a, b in zip(rows_a, rows_b):
        for i in range(n):
            atb[i] += a[i] * b
            for j in range(n):
                ata[i][j] += a[i] * a[j]
    # Gaussian elimination with partial pivoting
    m = [row[:] + [atb[i]] for i, row in enumerate(ata)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r2: abs(m[r2][col]))
        if abs(m[pivot][col]) < 1e-12:
            return None
        m[col], m[pivot] = m[pivot], m[col]
        for r2 in range(n):
            if r2 == col:
                continue
            f = m[r2][col] / m[col][col]
            for c2 in range(col, n + 1):
                m[r2][c2] -= f * m[col][c2]
    return [m[i][n] / m[i][i] for i in range(n)]


def field_coherence(residuals, w, h):
    """Fit residual vectors to a smooth linear+radial field of image position.

    True-pose residuals are dominated by lens-model error (radial) plus small rotation/scale
    error (linear in position) -> the smooth field explains most variance. Coincidence-match
    residuals are ~independent of position -> it explains ~basis/2M (chance level).
    Returns (r2, holdout_ratio): r2 = variance explained on all matches; holdout_ratio =
    post-fit rms on held-out half / pre-fit rms on held-out half (fit on even indices only).
    """
    if len(residuals) < 10:
        return 0.0, 1.0
    cx, cy = w / 2.0, h / 2.0
    scale = max(w, h) / 2.0

    def basis(x, y):
        xn, yn = (x - cx) / scale, (y - cy) / scale
        r2 = xn * xn + yn * yn
        return [1.0, xn, yn, r2 * xn, r2 * yn]

    def fit_and_score(fit_set, eval_set):
        arows, bx, by = [], [], []
        for (x, y, dx, dy) in fit_set:
            arows.append(basis(x, y))
            bx.append(dx)
            by.append(dy)
        px = solve_least_squares(arows, bx)
        py = solve_least_squares(arows, by)
        if px is None or py is None:
            return None
        pre = post = 0.0
        for (x, y, dx, dy) in eval_set:
            b = basis(x, y)
            mx = sum(pi * bi for pi, bi in zip(px, b))
            my = sum(pi * bi for pi, bi in zip(py, b))
            pre += dx * dx + dy * dy
            post += (dx - mx) ** 2 + (dy - my) ** 2
        return pre, post

    full = fit_and_score(residuals, residuals)
    r2 = 0.0
    if full and full[0] > 0:
        r2 = max(0.0, 1.0 - full[1] / full[0])
    even = residuals[0::2]
    odd = residuals[1::2]
    holdout = 1.0
    if len(even) >= 8 and len(odd) >= 4:
        ho = fit_and_score(even, odd)
        if ho and ho[0] > 0:
            holdout = math.sqrt(ho[1] / ho[0])
    return r2, holdout


def engineer_features(c):
    D, C = c["D"], c["C"]
    area = float(c["W"] * c["H"])
    feats = {}
    # Excess-over-chance at the tightness ladder + full radius
    for key, radius in (("eoc2", 2.0), ("eoc4", 4.0), ("eoc8", 8.0), ("eoc16", 16.0)):
        observed = c["m" + key[3:]]
        feats[key] = signed_poisson_llr(observed, chance_lambda(D, C, area, radius))
    feats["eocR"] = signed_poisson_llr(c["M"], chance_lambda(D, C, area, c["r"]))
    # Best (max) ladder EoC: the informative radius varies with density
    feats["eocBest"] = max(feats["eoc2"], feats["eoc4"], feats["eoc8"], feats["eoc16"])
    # Tightness concentration: fraction of matches within 4px vs chance fraction (~(4/r)^2)
    if c["M"] > 0 and c["r"] > 0:
        frac4 = c["m4"] / c["M"]
        chance_frac4 = min(1.0, (4.0 / c["r"]) ** 2)
        feats["tight4x"] = frac4 / chance_frac4 if chance_frac4 > 0 else 0.0
    else:
        feats["tight4x"] = 0.0
    # Bright agreement
    feats["bdFrac"] = (c["mBD"] / c["bD"]) if c["bD"] > 0 else 1.0
    feats["bpFrac"] = (c["mBP"] / c["bP"]) if c["bP"] > 0 else 1.0
    # v2: residual-field coherence (only present when the dump carried per-match residuals)
    if c.get("res"):
        feats["coherR2"], feats["holdout"] = field_coherence(c["res"], c["W"], c["H"])
    else:
        feats["coherR2"], feats["holdout"] = 0.0, 1.0
    return feats


def main():
    if len(sys.argv) < 4:
        print("usage: verifier_study.py <log> <corpus_csv> <out_csv> [<log> <corpus_csv> ...]")
        return 2
    out_csv = sys.argv[3]
    pairs = [(sys.argv[1], sys.argv[2])] + [
        (sys.argv[i], sys.argv[i + 1]) for i in range(4, len(sys.argv) - 1, 2)]

    all_rows = []
    for log_path, csv_path in pairs:
        truths = load_truth(csv_path)
        case_index = 0
        pending = []  # dumps seen since last verdict line
        dump_buffer = None  # PowerShell Out-File wraps stderr at ~120 cols; rejoin split records
        with open(log_path, encoding="utf-8", errors="replace") as f:
            for line in f:
                if dump_buffer is not None:
                    dump_buffer += line.rstrip("\r\n")
                    if "rank=" not in dump_buffer:
                        continue
                    line = dump_buffer + "\n"
                    dump_buffer = None
                m = DUMP_RE.search(line)
                if m and "rank=" not in line:
                    dump_buffer = line.rstrip("\r\n")
                    continue
                if m:
                    raw = parse_dump_fields(m.group(1))
                    try:
                        cand = {
                            "az": float(raw["az"]), "el": float(raw["el"]),
                            "roll": float(raw["roll"]), "fov": float(raw["fov"]),
                            "D": int(raw["D"]), "C": int(raw["C"]), "M": int(raw["M"]),
                            "m2": int(raw["m2"]), "m4": int(raw["m4"]),
                            "m8": int(raw["m8"]), "m16": int(raw["m16"]),
                            "rms": float(raw["rms"]), "r": float(raw["r"]),
                            "W": int(raw["W"]), "H": int(raw["H"]),
                            "bD": int(raw["bD"]), "mBD": int(raw["mBD"]),
                            "bP": int(raw["bP"]), "mBP": int(raw["mBP"]),
                        }
                    except (KeyError, ValueError):
                        continue
                    if "res" in raw:
                        cand["res"] = parse_residuals(raw["res"])
                    pending.append(cand)
                    continue
                v = VERDICT_RE.match(line)
                if v and case_index < len(truths):
                    truth = dict(truths[case_index])
                    # A PASSING case's accepted pose is anchor-certified (24px oracle), so it IS
                    # the truth — including roll/fov the CSV may not carry. This upgrades every
                    # passing case in any corpus to a full-pose truth source, which is what lets
                    # wrong-roll candidates be labelled on corpora whose rows have roll=0.
                    pose = POSE_RE.search(line)
                    if v.group(1) == "PASS" and pose and not truth["negative"]:
                        truth["az"] = float(pose.group(1))
                        truth["el"] = float(pose.group(2))
                        truth["rollVal"] = float(pose.group(3))
                        truth["hasRoll"] = True
                        truth["trueFov"] = float(pose.group(4)) or truth["trueFov"]
                        truth["hasDirection"] = True
                        truth["certified"] = True
                    for cand in pending:
                        lab = label_candidate(truth, cand)
                        if lab is None:
                            continue
                        feats = engineer_features(cand)
                        all_rows.append({
                            "source": truth["image"], "label": lab,
                            **{k: cand[k] for k in ("az", "el", "roll", "fov", "D", "C", "M",
                                                     "m2", "m4", "m8", "m16", "rms", "r")},
                            **{k: round(v2, 3) for k, v2 in feats.items()},
                        })
                    pending = []
                    case_index += 1

    if not all_rows:
        print("no labelled candidates found")
        return 1
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        writer.writeheader()
        writer.writerows(all_rows)

    # Separation summary per feature
    def pct(vals, q):
        s = sorted(vals)
        return s[min(len(s) - 1, int(q * len(s)))]

    right = [r for r in all_rows if r["label"] == "RIGHT"]
    wrong = [r for r in all_rows if r["label"] == "WRONG"]
    print(f"labelled candidates: RIGHT={len(right)} WRONG={len(wrong)} -> {out_csv}")
    for feat in ("eocBest", "eoc4", "eoc8", "eocR", "tight4x", "bdFrac", "bpFrac", "coherR2", "holdout"):
        rv = [r[feat] for r in right]
        wv = [w[feat] for w in wrong]
        if not rv or not wv:
            continue
        r10 = pct(rv, 0.10)
        w90 = pct(wv, 0.90)
        overlap = sum(1 for w in wv if w >= r10) / len(wv)
        print(f"  {feat:8s} RIGHT p10={r10:9.2f} median={pct(rv,0.5):9.2f} | "
              f"WRONG p90={w90:9.2f} median={pct(wv,0.5):9.2f} | "
              f"wrong-above-right-p10: {overlap:.1%}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
