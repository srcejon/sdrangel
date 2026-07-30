#!/usr/bin/env python3
"""Embed capture geometry into the plate-solver corpus images.

Writes the same `SDRangel.Camera` JSON that CameraMediaMetadata::writeImage produces, so the
harness (and the GUI) can read an image's pose, site, time and lens straight out of the file
instead of needing it in star-tests.csv. JPEG carries it in a COM marker using Qt's
"key: value" Description convention; the pixel data is untouched.

The values embedded are the CSV's DECLARED CAPTURE GEOMETRY, not the solver's output. That is
what this metadata means -- CameraMediaMetadata::fromFrame() builds it from CameraSettings, and
the two images written by the real camera (hip113561/hip114104) carry roll 0 even though their
solved rolls are -14.5 and -9.6 degrees. Embedding solved poses instead would also weaken the
corpus, since a future blank-column row would then be seeded with the answer it is meant to
find.

Where several rows share an image, the row with the highest plateSolveStartMode wins: the
higher modes trust more of the values, so those are the ones authored as truth (galaxy-m101-1
takes its verified roll of 87.2 from the mode-6 row rather than the mode-3 placeholder 0).

Usage: embed_capture_metadata.py [--dry-run]
"""
import csv
import io
import json
import os
import struct
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, "star-tests.csv")
METADATA_KEY = "SDRangel.Camera"


def projection_type(text):
    """Mirror parseProjection() in camerastartests.cpp."""
    lowered = (text or "").strip().lower()
    if "equisolid" in lowered:
        return 2
    if ("equidistant" in lowered) or ("fisheye" in lowered):
        return 1
    return 0


def iso_utc(csv_time):
    """CSV timestamps are UTC (the harness forces Qt::UTC) -> ISODateWithMs."""
    text = (csv_time or "").strip()
    if not text:
        return None
    return text.replace(" ", "T") + ".000Z"


def number(text, default=0.0):
    try:
        return float((text or "").strip())
    except ValueError:
        return default


def truthy(text):
    lowered = (text or "").strip().lower()
    return bool(lowered) and lowered not in ("0", "false", "no")


def build_metadata(row):
    time_text = iso_utc(row.get("time"))
    if time_text is None:
        return None
    metadata = {
        "version": 1,
        "captureDateTimeUtc": time_text,
        "site": {
            "latitude": number(row.get("latitude")),
            "longitude": number(row.get("longitude")),
            "altitude": number(row.get("altitude")),
        },
        "direction": {
            "azimuth": number(row.get("azimuth")),
            "elevation": number(row.get("elevation")),
            "roll": number(row.get("roll")),
        },
        "projection": {
            "fov": number(row.get("fov")),
            "type": projection_type(row.get("projection")),
            "centerOffsetX": number(row.get("cx")),
            "centerOffsetY": number(row.get("cy")),
            "distortionK1": number(row.get("k1")),
            "mirror": truthy(row.get("mirror")),
        },
    }
    if metadata["projection"]["fov"] <= 0.0:
        return None
    return metadata


def inject_jpeg_comment(path, payload, dry_run=False):
    """Insert a COM segment holding 'key: value' right after SOI. Pixels are untouched."""
    with open(path, "rb") as handle:
        data = handle.read()
    if data[:2] != b"\xff\xd8":
        return "not a JPEG"
    if METADATA_KEY.encode("utf-8") in data:
        return "already has metadata"
    comment = ("%s: %s" % (METADATA_KEY, json.dumps(payload))).encode("utf-8")
    if len(comment) + 2 > 0xFFFF:
        return "metadata too large for one COM segment"
    segment = b"\xff\xfe" + struct.pack(">H", len(comment) + 2) + comment
    if not dry_run:
        with open(path, "wb") as handle:
            handle.write(data[:2] + segment + data[2:])
    return "added %d bytes" % len(segment)


def main():
    dry_run = "--dry-run" in sys.argv
    with io.open(CSV_PATH, encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    by_image = defaultdict(list)
    for row in rows:
        by_image[row["image"]].append(row)

    added = skipped = failed = 0
    for image in sorted(by_image):
        # Highest start mode wins: those rows trust the most values, so they carry the truth.
        best = max(by_image[image], key=lambda r: int((r.get("plateSolveStartMode") or "0").strip() or 0))
        path = image if os.path.isabs(image) else os.path.join(HERE, image)
        name = os.path.basename(path)
        if not os.path.exists(path):
            print("  %-26s MISSING" % name)
            failed += 1
            continue
        with open(path, "rb") as handle:
            if METADATA_KEY.encode("utf-8") in handle.read():
                print("  %-26s already has metadata" % name)
                skipped += 1
                continue
        metadata = build_metadata(best)
        if metadata is None:
            print("  %-26s skipped (row has no usable time/fov)" % name)
            skipped += 1
            continue
        result = inject_jpeg_comment(path, metadata, dry_run)
        if result == "already has metadata":
            print("  %-26s %s" % (name, result))
            skipped += 1
        elif result.startswith("added"):
            direction = metadata["direction"]
            print("  %-26s %s  az=%.4f el=%.4f roll=%.2f fov=%s mode=%s"
                  % (name, result, direction["azimuth"], direction["elevation"],
                     direction["roll"], metadata["projection"]["fov"],
                     best.get("plateSolveStartMode")))
            added += 1
        else:
            print("  %-26s FAILED: %s" % (name, result))
            failed += 1

    print("\n%s: %d image(s) updated, %d skipped, %d failed"
          % ("dry run" if dry_run else "done", added, skipped, failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
