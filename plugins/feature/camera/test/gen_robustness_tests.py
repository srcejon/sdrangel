#!/usr/bin/env python3
"""Generate metamorphic + negative robustness tests for the plate solver.

Metamorphic: transform a known-good image in ways whose effect on the solution is
predictable (noise/brightness -> invariant; translate/rotate -> anchors move with a
known map). The solver should still solve and the named anchor should land where the
transform predicts. Brittleness shows up as a failure to solve a trivially-perturbed image.

Negative: garbage inputs (pure noise, random blobs) that have no real sky solution. The
solver MUST report solved=false. Hallucinating a solve here means the acceptance gates are
too loose -- the overfitting-toward-acceptance failure mode.
"""
import csv, os, sys
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_CSV = os.path.join(HERE, "star-tests.csv")
OUTDIR = os.path.join(HERE, "images-robustness")
os.makedirs(OUTDIR, exist_ok=True)

with open(SRC_CSV, newline="") as f:
    rows = list(csv.reader(f))
header = rows[0]
# pick the wide-9 mode-3 row as the metamorphic source (clean fisheye guided solve)
src = next(r for r in rows[1:] if "stars-wide-9.jpg" in r[0] and r[15] == "3")
col = {name: i for i, name in enumerate(header)}

img_path = os.path.join(HERE, src[col["image"]].strip())
img = cv2.imread(img_path, cv2.IMREAD_COLOR)
if img is None:
    sys.exit(f"could not read source image {img_path}")
H, W = img.shape[:2]
print(f"source {img_path}  {W}x{H}")

# source anchor (name:x:y) -- wide-9 has Vega:1603:1480
sp = src[col["starPositions"]]
aname, ax, ay = sp.split(":")
ax, ay = float(ax), float(ay)

def row_for(image_rel, *, star_positions=None, roll=None, expected_roll=None, expected_roll_tol=None, stars=None, startmode=None):
    r = list(src)
    r[col["image"]] = image_rel
    if star_positions is not None: r[col["starPositions"]] = star_positions
    if roll is not None:           r[col["roll"]] = f"{roll:.4f}"
    if expected_roll is not None:  r[col["expectedRoll"]] = f"{expected_roll:.4f}"
    if expected_roll_tol is not None: r[col["expectedRollTolerance"]] = f"{expected_roll_tol:.4f}"
    if stars is not None:          r[col["stars"]] = stars
    if startmode is not None:      r[col["plateSolveStartMode"]] = str(startmode)
    return r

meta_rows = []

# 1) additive Gaussian noise -- anchors + pose invariant
noisy = img.astype(np.float32) + np.random.default_rng(1).normal(0, 12.0, img.shape).astype(np.float32)
cv2.imwrite(os.path.join(OUTDIR, "meta-wide9-noise.jpg"), np.clip(noisy, 0, 255).astype(np.uint8))
meta_rows.append(row_for("images-robustness/meta-wide9-noise.jpg", star_positions=f"{aname}:{ax:.0f}:{ay:.0f}"))

# 2) brightness scaled down -- anchors + pose invariant
dim = np.clip(img.astype(np.float32) * 0.6, 0, 255).astype(np.uint8)
cv2.imwrite(os.path.join(OUTDIR, "meta-wide9-dim.jpg"), dim)
meta_rows.append(row_for("images-robustness/meta-wide9-dim.jpg", star_positions=f"{aname}:{ax:.0f}:{ay:.0f}"))

# 3) translation (+40,+30) -- anchor moves by the same offset, pose ~invariant (pp shift)
dx, dy = 40, 30
Mt = np.float32([[1, 0, dx], [0, 1, dy]])
shifted = cv2.warpAffine(img, Mt, (W, H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
cv2.imwrite(os.path.join(OUTDIR, "meta-wide9-shift.jpg"), shifted)
meta_rows.append(row_for("images-robustness/meta-wide9-shift.jpg", star_positions=f"{aname}:{ax+dx:.0f}:{ay+dy:.0f}"))

# 4) rotation about image centre by theta -- anchor rotated by the SAME matrix; rely on the
#    anchor-position check for equivariance (the solved roll should track but conventions vary).
for theta in (20.0,):
    Mr = cv2.getRotationMatrix2D((W / 2.0, H / 2.0), theta, 1.0)
    rot = cv2.warpAffine(img, Mr, (W, H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    rx = Mr[0, 0] * ax + Mr[0, 1] * ay + Mr[0, 2]
    ry = Mr[1, 0] * ax + Mr[1, 1] * ay + Mr[1, 2]
    name = f"meta-wide9-rot{int(theta)}.jpg"
    cv2.imwrite(os.path.join(OUTDIR, name), rot)
    meta_rows.append(row_for(f"images-robustness/{name}", star_positions=f"{aname}:{rx:.0f}:{ry:.0f}"))

# ---- negative inputs: must NOT solve ----
neg_rows = []
rng = np.random.default_rng(7)

# pure Gaussian noise, no stars
noise = rng.normal(40, 18, (H, W, 3)).clip(0, 255).astype(np.uint8)
cv2.imwrite(os.path.join(OUTDIR, "neg-noise.jpg"), noise)
neg_rows.append(row_for("images-robustness/neg-noise.jpg"))

# random bright blobs -- look like stars, random positions, no real sky correspondence
blobs = np.full((H, W, 3), 8, np.uint8)
for _ in range(80):
    cx, cy = rng.integers(50, W - 50), rng.integers(50, H - 50)
    cv2.circle(blobs, (int(cx), int(cy)), int(rng.integers(2, 5)), (255, 255, 255), -1)
blobs = cv2.GaussianBlur(blobs, (0, 0), 1.5)
cv2.imwrite(os.path.join(OUTDIR, "neg-blobs.jpg"), blobs)
neg_rows.append(row_for("images-robustness/neg-blobs.jpg"))

# Blind-mode negatives: the same garbage at startMode 1 (FoV-known blind) and 0 (fully blind),
# which exercise the more permissive blind acceptance gates (guided-mode rejection is the easy
# case). Must also yield solved=false.
neg_blind_rows = []
for img in ("images-robustness/neg-noise.jpg", "images-robustness/neg-blobs.jpg"):
    for mode in (1, 0):
        neg_blind_rows.append(row_for(img, startmode=mode))

def write_csv(path, rows_):
    with open(path, "w", newline="") as f:
        w = csv.writer(f, quoting=csv.QUOTE_MINIMAL)
        w.writerow(header)
        for r in rows_:
            w.writerow(r)

write_csv(os.path.join(HERE, "robustness-meta.csv"), meta_rows)
write_csv(os.path.join(HERE, "robustness-neg.csv"), neg_rows)
write_csv(os.path.join(HERE, "robustness-neg-blind.csv"), neg_blind_rows)
print(f"wrote {len(meta_rows)} metamorphic rows, {len(neg_rows)} guided-negative rows, "
      f"{len(neg_blind_rows)} blind-negative rows")
print("metamorphic: expect ALL PASS;  negatives (guided + blind): expect ALL solved=false")
