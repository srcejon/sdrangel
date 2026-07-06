#!/usr/bin/env python3
"""Wrong-roll SEARCH-vs-SELECTION diagnostic.

Given a candidate dump collected with SDRANGEL_CAMERA_PLATE_SOLVER_FORCE_ROLL_RECOVERY=1
over a set of (synthetic) cases with EXACT CSV ground truth, answer per case:
  reached  = did any dumped candidate match the true pose (az/el/roll/fov within tol)?
  selected = did the accepted verdict pose match the true pose?
If reached but not selected -> SELECTION failure (candidate exists, logic picks wrong one).
If not reached             -> SEARCH failure (correct pose never generated even with recovery).

Usage: rollrecovery_reach.py <dump.log> <cases.csv>
"""
import csv, re, sys, math

DUMP_RE = re.compile(r"CandidateDump,(.*)$")
VERDICT_RE = re.compile(r"^(PASS|FAIL) (\S+?):")
POSE_RE = re.compile(r"poseAz=([\-0-9.]+).*?poseEl=([\-0-9.]+).*?poseRoll=([\-0-9.]+).*?poseFov=([\-0-9.]+)")
VMATCH_RE = re.compile(r"matched=(\d+).*?solved=(\w+)")

AZ_TOL, EL_TOL, ROLL_TOL, FOV_FRAC = 0.35, 0.35, 5.0, 0.08


def ang_delta(a, b):
    d = (a - b) % 360.0
    return min(d, 360.0 - d)


def parse_fields(s):
    out = {}
    for kv in s.split(","):
        if "=" in kv:
            k, v = kv.split("=", 1)
            out[k] = v
    return out


def matches_truth(az, el, roll, fov, t):
    if abs(fov - t["fov"]) > FOV_FRAC * t["fov"]:
        return False
    # boresight direction on the sky (az converges near the pole, so weight by cos el)
    if ang_delta(el, t["el"]) > EL_TOL:
        return False
    if ang_delta(az, t["az"]) * math.cos(math.radians(t["el"])) > AZ_TOL:
        return False
    return ang_delta(roll, t["roll"]) <= ROLL_TOL


def main():
    dump, cases = sys.argv[1], sys.argv[2]
    truths = []
    for r in csv.DictReader(open(cases, encoding="utf-8")):
        truths.append({
            "image": r["image"].split("/")[-1],
            "az": float(r["azimuth"]), "el": float(r["elevation"]),
            "roll": float(r["roll"]), "fov": float(r["fov"]),
        })

    ci = 0
    pending = []
    buf = None
    reached = selected = reached_not_selected = 0
    reach_by_min_rolldelta = []
    detail = []
    for line in open(dump, encoding="utf-8", errors="ignore"):
        if buf is not None:
            line = buf + line
            buf = None
        m = DUMP_RE.search(line)
        if m and "rank=" not in line:
            buf = line.rstrip("\r\n")
            continue
        if m:
            f = parse_fields(m.group(1))
            try:
                pending.append((float(f["az"]), float(f["el"]), float(f["roll"]),
                                float(f["fov"]), int(f["M"]), float(f["rms"]),
                                int(f["m8"])))
            except (KeyError, ValueError):
                pass
            continue
        v = VERDICT_RE.match(line)
        if v and ci < len(truths):
            t = truths[ci]
            hit = [c for c in pending if matches_truth(c[0], c[1], c[2], c[3], t)]
            was_reached = bool(hit)
            # best-fitting truth candidate (by matches, then tightness)
            best_true = max(hit, key=lambda c: (c[4], c[6], -c[5])) if hit else None
            # best-fitting candidate OVERALL in this case (what an ideal selector could pick)
            best_any = max(pending, key=lambda c: (c[4], c[6], -c[5])) if pending else None
            pose = POSE_RE.search(line)
            vm = VMATCH_RE.search(line)
            sel_matched = int(vm.group(1)) if vm else -1
            sel_solved = (vm.group(2) == "true") if vm else False
            was_selected = False
            if pose:
                was_selected = matches_truth(float(pose.group(1)), float(pose.group(2)),
                                             float(pose.group(3)), float(pose.group(4)), t)
            reached += was_reached
            selected += was_selected
            if was_reached and not was_selected:
                reached_not_selected += 1
            near = [c for c in pending
                    if abs(c[3] - t["fov"]) <= FOV_FRAC * t["fov"]
                    and ang_delta(c[1], t["el"]) <= EL_TOL
                    and ang_delta(c[0], t["az"]) * math.cos(math.radians(t["el"])) <= AZ_TOL]
            min_rd = min((ang_delta(c[2], t["roll"]) for c in near), default=None)
            if min_rd is not None:
                reach_by_min_rolldelta.append(min_rd)
            detail.append((t["image"], v.group(1), was_reached, was_selected, len(pending),
                           len(near), min_rd, best_true, best_any, sel_matched, sel_solved))
            pending = []
            ci += 1
    n = ci
    print(f"cases: {n}   (verdict PASS with forced recovery: "
          f"{sum(1 for d in detail if d[1]=='PASS')})")
    print(f"REACHED true pose (some candidate matches truth):  {reached}/{n}")
    print(f"SELECTED true pose (accepted verdict matches truth): {selected}/{n}")
    print(f"reached but NOT selected (=> selection failures):    {reached_not_selected}")
    print(f"not reached (=> search failures):                    {n - reached}")
    if reach_by_min_rolldelta:
        s = sorted(reach_by_min_rolldelta)
        print(f"among cases with an az/el/fov-correct candidate (n={len(s)}), "
              f"min roll delta to truth: median={s[len(s)//2]:.1f} p90={s[min(len(s)-1,int(0.9*len(s)))]:.1f} max={s[-1]:.1f}")
    # The decisive comparison: at the TRUE pose, how strong is the fit vs the selected pose?
    # If the true-pose fit is strongly better than the selected fit yet not chosen -> selection bug.
    # If the true-pose fit is weak (few matches) -> the image lacks matchable support at truth.
    strong_true = [d for d in detail if d[7] and d[7][4] >= 8]
    print(f"\ncases where the TRUE pose gets >=8 matches (would-be solvable): {len(strong_true)}/{n}")
    tm = sorted(d[7][4] for d in detail if d[7])
    if tm:
        print(f"true-pose match count: median={tm[len(tm)//2]} p90={tm[min(len(tm)-1,int(0.9*len(tm)))]} max={tm[-1]}")
    sm = sorted(d[9] for d in detail if d[9] >= 0)
    if sm:
        print(f"selected-pose match count: median={sm[len(sm)//2]} max={sm[-1]}")
    # best-achievable at truth vs best-achievable overall (is truth ever the strongest candidate?)
    truth_is_best = sum(1 for d in detail if d[7] and d[8] and d[7][4] >= d[8][4])
    print(f"cases where true pose is the (tied-)strongest candidate by matches: {truth_is_best}/{n}")

    print("\nper-case (image, verdict, trueM/trueRMS, selM, bestAnyM/bestAnyRoll, minRollD):")
    for d in detail:
        rd = f"{d[6]:.1f}" if d[6] is not None else "-"
        bt = f"{d[7][4]}/{d[7][5]:.1f}" if d[7] else "-"
        ba = f"{d[8][4]}@roll{d[8][2]:.0f}/rms{d[8][5]:.1f}" if d[8] else "-"
        print(f"  {d[0]:22s} {d[1]:4s} trueM={bt:>9s} selM={d[9]:3d} bestAny={ba:>22s} rollD={rd}")


if __name__ == "__main__":
    main()
