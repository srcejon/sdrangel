#!/usr/bin/env python3
"""Render prioritized meteor candidate windows for manual labeling."""

import argparse
import csv
import math
import os
import random
import struct
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from scipy.signal import resample_poly, stft


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wav", required=True, help="Stereo I/Q WAV recording")
    parser.add_argument("--audit", required=True, help="Current candidate audit CSV")
    parser.add_argument("--baseline-audit", help="Earlier audit used to identify newly accepted candidates")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--limit-per-category", type=int, default=40)
    parser.add_argument("--quiet-windows", type=int, default=36)
    parser.add_argument("--context-before", type=float, default=2.0)
    parser.add_argument("--context-after", type=float, default=3.0)
    parser.add_argument("--panels-per-page", type=int, default=12)
    parser.add_argument("--seed", type=int, default=0)
    return parser.parse_args()


def read_audit(path):
    with open(path, newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def memory_map_wav(path):
    channels = None
    sample_rate = None
    bits_per_sample = None
    data_offset = None
    data_size = None

    with open(path, "rb") as handle:
        if handle.read(4) != b"RIFF":
            raise ValueError("Expected a RIFF WAV")

        handle.read(4)

        if handle.read(4) != b"WAVE":
            raise ValueError("Expected a WAVE header")

        while True:
            chunk_header = handle.read(8)

            if len(chunk_header) != 8:
                break

            chunk_id, chunk_size = struct.unpack("<4sI", chunk_header)
            chunk_offset = handle.tell()

            if chunk_id == b"fmt ":
                format_data = handle.read(min(chunk_size, 16))

                if len(format_data) < 16:
                    raise ValueError("Truncated WAV format chunk")

                audio_format, channels, sample_rate, _, _, bits_per_sample = struct.unpack(
                    "<HHIIHH", format_data)

                if audio_format != 1 or bits_per_sample != 16:
                    raise ValueError("Only stereo 16-bit PCM I/Q WAVs are supported")
            elif chunk_id == b"data":
                data_offset = chunk_offset
                data_size = chunk_size
                break

            handle.seek(chunk_offset + chunk_size + (chunk_size & 1))

    if not channels or not sample_rate or data_offset is None:
        raise ValueError("WAV is missing format or data chunks")

    if data_size == 0:
        data_size = os.path.getsize(path) - data_offset

    frame_size = channels * (bits_per_sample // 8)
    frame_count = data_size // frame_size
    data = np.memmap(
        path,
        dtype="<i2",
        mode="r",
        offset=data_offset,
        shape=(frame_count, channels),
    )
    return sample_rate, data


def number(row, name, fallback=0.0):
    try:
        return float(row.get(name, fallback))
    except (TypeError, ValueError):
        return fallback


def integer(row, name, fallback=0):
    return int(round(number(row, name, fallback)))


def candidate_key(row):
    return integer(row, "startSample"), integer(row, "endSample")


def spaced(rows, limit, minimum_gap_s=0.5):
    selected = []

    for row in rows:
        sample_rate = max(1, integer(row, "sampleRate", 1000))
        center_s = 0.5 * (number(row, "startSample") + number(row, "endSample")) / sample_rate

        if all(abs(center_s - prior[0]) >= minimum_gap_s for prior in selected):
            selected.append((center_s, row))

        if len(selected) >= limit:
            break

    return [row for _, row in selected]


def select_candidates(rows, baseline_rows, limit, quiet_count, recording_duration_s, seed):
    categories = defaultdict(list)
    baseline_accepted = {
        candidate_key(row)
        for row in baseline_rows
        if row.get("accepted") == "1"
    }

    newly_accepted = [
        row for row in rows
        if row.get("accepted") == "1" and candidate_key(row) not in baseline_accepted
    ]
    newly_accepted.sort(key=lambda row: number(row, "startSample"))
    categories["newly-accepted"] = newly_accepted

    reject_categories = (
        ("positive-frame-reject", "frames"),
        ("positive-evidence-reject", "spectral-evidence"),
        ("sweep-reject", "smooth-sweep"),
        ("duplicate-reject", "duplicate"),
    )

    for category, reason in reject_categories:
        candidates = [
            row for row in rows
            if row.get("accepted") == "0"
            and row.get("rejectionReason") == reason
            and number(row, "scoreMargin") >= 0.0
        ]
        candidates.sort(key=lambda row: number(row, "scoreMargin"), reverse=True)
        categories[category] = spaced(candidates, limit)

    occupied = []

    for row in rows:
        sample_rate = max(1, integer(row, "sampleRate", 1000))
        occupied.append((
            number(row, "startSample") / sample_rate - 2.0,
            number(row, "endSample") / sample_rate + 2.0,
        ))

    random_generator = random.Random(seed)
    quiet_candidates = [
        2.5 + i * max(5.0, (recording_duration_s - 5.0) / max(quiet_count * 8, 1))
        for i in range(max(quiet_count * 8, quiet_count))
    ]
    random_generator.shuffle(quiet_candidates)

    for center_s in quiet_candidates:
        if any(start_s <= center_s <= end_s for start_s, end_s in occupied):
            continue

        categories["candidate-free"].append({
            "index": "",
            "sampleRate": "1000",
            "startSample": f"{int(round(center_s * 1000.0))}",
            "endSample": f"{int(round(center_s * 1000.0))}",
            "centerFrequencyHz": "0",
            "frequencySpanHz": "0",
            "durationS": "0",
            "frameCount": "0",
            "scoreMargin": "0",
            "rejectionReason": "no-candidate",
            "accepted": "0",
        })

        if len(categories["candidate-free"]) >= quiet_count:
            break

    return categories


def extract_iq(data, source_rate, detector_rate, center_s, before_s, after_s):
    first = max(0, int(math.floor((center_s - before_s) * source_rate)))
    last = min(len(data), int(math.ceil((center_s + after_s) * source_rate)))
    raw = data[first:last]

    if raw.ndim != 2 or raw.shape[1] < 2:
        raise ValueError("Expected a stereo I/Q WAV")

    iq = raw[:, 0].astype(np.float32) + 1j * raw[:, 1].astype(np.float32)

    if source_rate != detector_rate:
        divisor = math.gcd(source_rate, detector_rate)
        iq = resample_poly(iq, detector_rate // divisor, source_rate // divisor)

    return iq


def render_panel(axis, row, category, data, source_rate, before_s, after_s):
    detector_rate = max(1, integer(row, "sampleRate", 1000))
    start_s = number(row, "startSample") / detector_rate
    end_s = number(row, "endSample") / detector_rate
    center_s = 0.5 * (start_s + end_s)
    iq = extract_iq(data, source_rate, detector_rate, center_s, before_s, after_s)
    frame_size = min(128, len(iq))
    overlap = max(0, frame_size - max(1, frame_size // 4))
    frequencies, times, spectrum = stft(
        iq,
        fs=detector_rate,
        window="hann",
        nperseg=frame_size,
        noverlap=overlap,
        nfft=max(256, frame_size),
        return_onesided=False,
        boundary=None,
        padded=False,
    )
    frequencies = np.fft.fftshift(frequencies)
    power_db = 10.0 * np.log10(np.maximum(np.abs(np.fft.fftshift(spectrum, axes=0)) ** 2, 1e-20))
    excess_db = power_db - np.median(power_db, axis=1, keepdims=True)
    low_scale = 3.0
    high_scale = max(12.0, float(np.percentile(excess_db, 99.7)))
    relative_times = times - before_s
    axis.pcolormesh(
        relative_times,
        frequencies,
        excess_db,
        shading="auto",
        cmap="turbo",
        vmin=low_scale,
        vmax=high_scale,
    )
    axis.axvline(0.0, color="white", linewidth=0.6, alpha=0.7)

    duration_s = max(0.0, end_s - start_s)
    span_hz = number(row, "frequencySpanHz")
    center_hz = number(row, "centerFrequencyHz")

    if duration_s > 0.0 and span_hz > 0.0:
        relative_start = start_s - center_s
        axis.add_patch(Rectangle(
            (relative_start, center_hz - 0.5 * span_hz),
            duration_s,
            span_hz,
            fill=False,
            edgecolor="white",
            linewidth=0.9,
        ))

    axis.set_xlim(-before_s, after_s)
    half_view_hz = max(180.0, span_hz)
    view_center_hz = center_hz if span_hz > 0.0 else 0.0
    nyquist_limit = 0.48 * detector_rate
    axis.set_ylim(
        max(-nyquist_limit, view_center_hz - half_view_hz),
        min(nyquist_limit, view_center_hz + half_view_hz),
    )
    axis.set_ylabel("Hz")
    axis.set_title(
        f"{category}  t={center_s:.3f}s  idx={row.get('index', '')}  "
        f"frames={row.get('frameCount', '')}  margin={number(row, 'scoreMargin'):.2f}  "
        f"{row.get('rejectionReason', '')}",
        fontsize=8,
    )


def write_manifest(path, categories):
    fields = (
        "category", "priority", "index", "sampleRate", "startSample", "endSample",
        "durationS", "centerFrequencyHz", "frequencySpanHz", "frequencyDriftHz",
        "frameCount", "scoreMargin", "accepted", "rejectionReason", "label", "eventId", "notes",
    )

    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()

        for category, rows in categories.items():
            for priority, row in enumerate(rows, 1):
                output = {field: row.get(field, "") for field in fields}
                output["category"] = category
                output["priority"] = priority
                writer.writerow(output)


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    rows = read_audit(args.audit)
    baseline_rows = read_audit(args.baseline_audit) if args.baseline_audit else []
    source_rate, data = memory_map_wav(args.wav)
    duration_s = len(data) / float(source_rate)
    categories = select_candidates(
        rows,
        baseline_rows,
        args.limit_per_category,
        args.quiet_windows,
        duration_s,
        args.seed,
    )
    write_manifest(os.path.join(args.output_dir, "review.csv"), categories)

    for category, category_rows in categories.items():
        for page_start in range(0, len(category_rows), args.panels_per_page):
            page_rows = category_rows[page_start:page_start + args.panels_per_page]
            figure, axes = plt.subplots(len(page_rows), 1, figsize=(14, 2.2 * len(page_rows)), squeeze=False)

            for axis, row in zip(axes[:, 0], page_rows):
                render_panel(
                    axis,
                    row,
                    category,
                    data,
                    source_rate,
                    args.context_before,
                    args.context_after,
                )

            axes[-1, 0].set_xlabel("Seconds relative to candidate center")
            figure.tight_layout()
            page = page_start // args.panels_per_page + 1
            figure.savefig(os.path.join(args.output_dir, f"{category}-{page:02d}.png"), dpi=130)
            plt.close(figure)

    print("Review rows:", sum(len(rows) for rows in categories.values()))

    for category, category_rows in categories.items():
        print(f"  {category}: {len(category_rows)}")


if __name__ == "__main__":
    main()
