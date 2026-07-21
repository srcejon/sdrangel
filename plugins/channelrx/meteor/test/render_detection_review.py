#!/usr/bin/env python3
"""Render targeted accepted detections and rejected-candidate gaps for review."""

import argparse
import csv
import html
import math
import os
import random
import struct

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from scipy.signal import resample_poly, stft

plt.style.use("dark_background")


def parse_numbers(value):
    return [int(item.strip()) for item in value.split(",") if item.strip()]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wav", required=True, help="Stereo signed 16-bit I/Q WAV")
    parser.add_argument("--detections", required=True, help="Accepted detection CSV")
    parser.add_argument("--audit", required=True, help="Candidate audit CSV")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--miss-after", type=parse_numbers, required=True,
                        help="Comma-separated one-based detection numbers")
    parser.add_argument("--span-detections", type=parse_numbers, required=True,
                        help="Comma-separated one-based detection numbers")
    parser.add_argument("--detector-rate", type=int, default=1000)
    parser.add_argument("--minimum-candidate-margin", type=float, default=-2.0)
    parser.add_argument("--correct-controls", type=int, default=12)
    parser.add_argument("--wide-controls", type=int, default=12)
    parser.add_argument("--seed", type=int, default=20260719)
    return parser.parse_args()


def read_csv(path):
    with open(path, newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def number(row, name, fallback=0.0):
    try:
        return float(row.get(name, fallback))
    except (TypeError, ValueError):
        return fallback


def integer(row, name, fallback=0):
    return int(round(number(row, name, fallback)))


def read_wav(path):
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

                if audio_format != 1 or channels < 2 or bits_per_sample != 16:
                    raise ValueError("Expected stereo signed 16-bit PCM I/Q")
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


def accepted_record(row, detector_rate):
    start_s = number(row, "timeOffsetS")
    duration_s = number(row, "durationS")
    return {
        "number": integer(row, "index"),
        "startSample": int(round(start_s * detector_rate)),
        "endSample": int(round((start_s + duration_s) * detector_rate)),
        "durationS": duration_s,
        "centerFrequencyHz": number(row, "centerFrequencyHz"),
        "frequencySpanHz": number(row, "frequencySpanHz"),
        "frequencyDriftHz": number(row, "frequencyDriftHz"),
        "totalPowerDB": number(row, "totalPowerDB"),
    }


def extract_iq(data, source_rate, detector_rate, start_s, end_s):
    first = max(0, int(math.floor(start_s * source_rate)))
    last = min(len(data), int(math.ceil(end_s * source_rate)))
    raw = data[first:last]
    iq = (raw[:, 0].astype(np.float32) + 1j * raw[:, 1].astype(np.float32)) / 32768.0

    if source_rate != detector_rate:
        divisor = math.gcd(source_rate, detector_rate)
        iq = resample_poly(iq, detector_rate // divisor, source_rate // divisor)

    return iq, first / float(source_rate)


def spectrum(iq, detector_rate, frame_size, overlap):
    frame_size = min(frame_size, len(iq))

    if frame_size < 8:
        raise ValueError("Review window is too short")

    frequencies, times, values = stft(
        iq,
        fs=detector_rate,
        window="hann",
        nperseg=frame_size,
        noverlap=min(overlap, frame_size - 1),
        nfft=frame_size,
        return_onesided=False,
        boundary=None,
        padded=False,
    )
    frequencies = np.fft.fftshift(frequencies)
    power_db = 10.0 * np.log10(
        np.maximum(np.abs(np.fft.fftshift(values, axes=0)) ** 2, 1.0e-20))
    return frequencies, times, power_db


def draw_spectrum(axis, iq, detector_rate, frame_size, overlap, absolute_start_s,
                  reference_s, frequency_low, frequency_high):
    frequencies, times, power_db = spectrum(iq, detector_rate, frame_size, overlap)
    times = times + absolute_start_s - reference_s
    low_scale = float(np.percentile(power_db, 55.0))
    high_scale = low_scale + 20.0
    axis.pcolormesh(
        frequencies,
        times,
        power_db.T,
        shading="auto",
        cmap="turbo",
        vmin=low_scale,
        vmax=high_scale,
    )
    axis.set_xlim(frequency_low, frequency_high)
    axis.set_xlabel("Frequency (Hz)")
    axis.grid(False)


def draw_box(axis, start_s, end_s, center_hz, span_hz, reference_s,
             color, linestyle="-", label=None, text_color=None):
    if end_s <= start_s or span_hz <= 0.0:
        return

    axis.add_patch(Rectangle(
        (center_hz - 0.5 * span_hz, start_s - reference_s),
        span_hz,
        end_s - start_s,
        fill=False,
        edgecolor=color,
        linewidth=1.2,
        linestyle=linestyle,
    ))

    if label:
        axis.text(
            center_hz + 0.5 * span_hz,
            start_s - reference_s,
            label,
            color=text_color or color,
            fontsize=7,
            va="top",
        )


def render_detection_case(output_path, title, detection, data, source_rate,
                          detector_rate, context_before=2.5, context_after=3.0):
    start_s = detection["startSample"] / detector_rate
    end_s = detection["endSample"] / detector_rate
    center_s = 0.5 * (start_s + end_s)
    window_start = max(0.0, start_s - context_before)
    window_end = end_s + context_after
    iq, actual_start = extract_iq(data, source_rate, detector_rate, window_start, window_end)
    center_hz = detection["centerFrequencyHz"]
    span_hz = max(detection["frequencySpanHz"], 8.0)
    half_view = max(180.0, 0.85 * span_hz)
    nyquist = 0.5 * detector_rate
    y_low = max(-nyquist, center_hz - half_view)
    y_high = min(nyquist, center_hz + half_view)
    figure, axes = plt.subplots(2, 1, figsize=(15, 7), constrained_layout=True)

    draw_spectrum(axes[0], iq, detector_rate, 128, 96, actual_start, center_s, y_low, y_high)
    draw_spectrum(axes[1], iq, detector_rate, 1024, 512, actual_start, center_s, y_low, y_high)

    for axis in axes:
        draw_box(
            axis,
            start_s,
            end_s,
            center_hz,
            span_hz,
            center_s,
            "white",
            label=f"#{detection['number']}",
        )
        axis.axhline(0.0, color="white", linewidth=0.5, alpha=0.35)
        axis.set_ylim(window_start - center_s, window_end - center_s)
        axis.invert_yaxis()
        axis.set_ylabel("Seconds relative to detection center")

    axes[0].set_title(f"{title} - head resolution (128 point)")
    axes[1].set_title("Trail resolution (1024 point)")
    figure.savefig(output_path, dpi=140)
    plt.close(figure)


def render_candidate_case(output_path, title, candidate, accepted, data, source_rate,
                          detector_rate, context_before=2.5, context_after=3.0):
    start_s = number(candidate, "startSample") / detector_rate
    end_s = number(candidate, "endSample") / detector_rate
    center_s = 0.5 * (start_s + end_s)
    window_start = max(0.0, start_s - context_before)
    window_end = end_s + context_after
    iq, actual_start = extract_iq(data, source_rate, detector_rate, window_start, window_end)
    center_hz = number(candidate, "centerFrequencyHz")
    span_hz = max(number(candidate, "frequencySpanHz"), 8.0)
    half_view = max(180.0, span_hz)
    nyquist = 0.5 * detector_rate
    y_low = max(-nyquist, center_hz - half_view)
    y_high = min(nyquist, center_hz + half_view)
    figure, axes = plt.subplots(2, 1, figsize=(15, 7), constrained_layout=True)

    draw_spectrum(axes[0], iq, detector_rate, 128, 96, actual_start, center_s, y_low, y_high)
    draw_spectrum(axes[1], iq, detector_rate, 1024, 512, actual_start, center_s, y_low, y_high)

    for axis in axes:
        draw_box(
            axis,
            start_s,
            end_s,
            center_hz,
            span_hz,
            center_s,
            "cyan",
            "--",
            f"candidate {candidate.get('index', '')}",
        )

        for detection in accepted:
            detection_start = detection["startSample"] / detector_rate
            detection_end = detection["endSample"] / detector_rate

            if detection_end < window_start or detection_start > window_end:
                continue

            draw_box(
                axis,
                detection_start,
                detection_end,
                detection["centerFrequencyHz"],
                max(detection["frequencySpanHz"], 8.0),
                center_s,
                "white",
                label=f"#{detection['number']}",
            )

        axis.axhline(0.0, color="cyan", linewidth=0.5, alpha=0.5)
        axis.set_ylim(window_start - center_s, window_end - center_s)
        axis.invert_yaxis()
        axis.set_ylabel("Seconds relative to candidate center")

    axes[0].set_title(f"{title} - head resolution (128 point)")
    axes[1].set_title("Trail resolution (1024 point)")
    figure.savefig(output_path, dpi=140)
    plt.close(figure)


def render_gap_overview(output_path, after_number, prior, following, candidates,
                        data, source_rate, detector_rate):
    prior_end_s = prior["endSample"] / detector_rate
    following_start_s = following["startSample"] / detector_rate
    padding = 1.0
    window_start = max(0.0, prior_end_s - padding)
    window_end = following_start_s + padding
    reference_s = prior_end_s
    iq, actual_start = extract_iq(data, source_rate, detector_rate, window_start, window_end)
    frequencies, times, power_db = spectrum(iq, detector_rate, 128, 64)
    relative_times = times + actual_start - reference_s
    strip_duration_s = 15.0
    first_relative_s = window_start - reference_s
    last_relative_s = window_end - reference_s
    strip_count = max(1, int(math.ceil((last_relative_s - first_relative_s) / strip_duration_s)))
    figure, axes = plt.subplots(
        1,
        strip_count,
        figsize=(4.0 * strip_count, 9.0),
        constrained_layout=True,
        squeeze=False,
    )
    axes = axes[0]
    nyquist = 0.45 * detector_rate
    low_scale = float(np.percentile(power_db, 55.0))
    high_scale = low_scale + 20.0

    for strip, axis in enumerate(axes):
        strip_start = first_relative_s + strip * strip_duration_s
        strip_end = min(last_relative_s, strip_start + strip_duration_s)
        strip_display_end = strip_start + strip_duration_s
        mask = (relative_times >= strip_start) & (relative_times <= strip_end)

        if np.any(mask):
            axis.pcolormesh(
                frequencies,
                relative_times[mask],
                power_db[:, mask].T,
                shading="auto",
                cmap="turbo",
                vmin=low_scale,
                vmax=high_scale,
            )

        for candidate in candidates:
            candidate_start = number(candidate, "startSample") / detector_rate - reference_s
            candidate_end = number(candidate, "endSample") / detector_rate - reference_s

            if candidate_end < strip_start or candidate_start > strip_end:
                continue

            draw_box(
                axis,
                number(candidate, "startSample") / detector_rate,
                number(candidate, "endSample") / detector_rate,
                number(candidate, "centerFrequencyHz"),
                max(number(candidate, "frequencySpanHz"), 8.0),
                reference_s,
                "cyan",
                "--",
                f"c{candidate.get('index', '')}",
                "white",
            )

        if strip_start <= 0.0 <= strip_end:
            axis.axhline(0.0, color="white", linewidth=0.8)

        next_start = following_start_s - reference_s

        if strip_start <= next_start <= strip_end:
            axis.axhline(next_start, color="yellow", linewidth=0.8)

        axis.set_xlim(-nyquist, nyquist)
        axis.set_ylim(strip_start, strip_display_end)
        axis.invert_yaxis()
        axis.set_title(f"{strip_start:+.1f} to {strip_end:+.1f} s", fontsize=9)
        axis.set_xlabel("Frequency (Hz)")

        if strip == 0:
            axis.set_ylabel(f"Seconds after #{after_number}")

    figure.suptitle(
        f"Possible miss after #{after_number}: {following_start_s - prior_end_s:.3f} s gap; "
        f"{len(candidates)} plausible rejected candidates",
        fontsize=12,
    )
    figure.savefig(output_path, dpi=140)
    plt.close(figure)


def choose_controls(accepted, excluded, correct_count, wide_count, seed):
    eligible = [item for item in accepted if item["number"] not in excluded]
    wide = sorted(eligible, key=lambda item: item["frequencySpanHz"], reverse=True)[:wide_count]
    wide_numbers = {item["number"] for item in wide}
    ordinary = [
        item for item in eligible
        if item["number"] not in wide_numbers
        and 0.20 <= item["durationS"] <= 2.0
        and 30.0 <= item["frequencySpanHz"] <= 180.0
    ]
    random.Random(seed).shuffle(ordinary)
    ordinary = sorted(ordinary[:correct_count], key=lambda item: item["number"])
    return ordinary, wide


def write_manifest(path, rows):
    fields = (
        "caseId", "issueType", "sourceDetectionNumber", "candidateIndex",
        "startSample", "endSample", "timeOffsetS", "durationS",
        "centerFrequencyHz", "frequencySpanHz", "frequencyDriftHz",
        "frameCount", "scoreMargin", "rejectionReason", "imageFile",
        "label", "eventId", "expectedStartS", "expectedEndS",
        "expectedLowHz", "expectedHighHz", "confidence", "notes",
    )

    review_fields = (
        "label", "eventId", "expectedStartS", "expectedEndS",
        "expectedLowHz", "expectedHighHz", "confidence", "notes",
    )
    existing = {}

    if os.path.exists(path):
        existing = {row.get("caseId", ""): row for row in read_csv(path)}

    for row in rows:
        prior = existing.get(row["caseId"])

        if not prior:
            continue

        for field in review_fields:
            row[field] = prior.get(field, row.get(field, ""))

    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_index(path, rows, gap_images):
    sections = []

    for title, predicate in (
        ("Rejected candidates in those gaps", lambda row: row["issueType"] == "possible-miss-candidate"),
        ("Possibly narrow spans", lambda row: row["issueType"] == "span-too-narrow"),
        ("Ordinary controls", lambda row: row["issueType"] == "ordinary-control"),
        ("Wide-span controls", lambda row: row["issueType"] == "wide-control"),
    ):
        cards = []

        for row in filter(predicate, rows):
            cards.append(
                "<article><h3>{}</h3><p>{}</p><a href=\"{}\"><img src=\"{}\"></a></article>".format(
                    html.escape(row["caseId"]),
                    html.escape(
                        f"detection #{row['sourceDetectionNumber']} candidate {row['candidateIndex']} "
                        f"{row['rejectionReason']} margin {row['scoreMargin']}"),
                    html.escape(row["imageFile"]),
                    html.escape(row["imageFile"]),
                ))

        if cards:
            sections.append(f"<h2>{html.escape(title)}</h2><section>{''.join(cards)}</section>")

    gap_cards = "".join(
        f"<article><h3>After #{number}</h3><a href=\"{image}\"><img src=\"{image}\"></a></article>"
        for number, image in gap_images
    )
    document = """<!doctype html>
<html><head><meta charset="utf-8"><title>Meteor detector review</title>
<style>
body {{ background:#17191c; color:#e8eaed; font:14px sans-serif; margin:24px; }}
section {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(420px,1fr)); gap:16px; }}
article {{ background:#24272b; border:1px solid #454a50; padding:10px; }}
img {{ display:block; width:100%; height:auto; }}
h1,h2,h3 {{ letter-spacing:0; }} code {{ color:#7dd3fc; }}
</style></head><body>
<h1>July 19 meteor detector review</h1>
<p>Edit <code>review.csv</code>. Review the gap overview first, then its candidate rows.
For possible misses use label <code>meteor</code>, <code>noise</code>, or
<code>uncertain</code>. For span cases and controls use
<code>correct</code>, <code>too-narrow</code>, <code>too-wide</code>, or <code>uncertain</code>.
Expected bounds are optional.</p>
<h2>Full gap overviews</h2><section>{}</section>{}
</body></html>""".format(gap_cards, "".join(sections))

    with open(path, "w", encoding="utf-8") as handle:
        handle.write(document)


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    accepted_rows = read_csv(args.detections)
    audit_rows = read_csv(args.audit)
    source_rate, data = read_wav(args.wav)
    accepted = [accepted_record(row, args.detector_rate) for row in accepted_rows]
    accepted_by_number = {item["number"]: item for item in accepted}
    rejected = [row for row in audit_rows if row.get("accepted") == "0"]
    manifest = []
    gap_images = []

    for after_number in args.miss_after:
        prior = accepted_by_number[after_number]
        following = accepted_by_number[after_number + 1]
        gap_start = prior["endSample"]
        gap_end = following["startSample"]
        candidates = [
            row for row in rejected
            if gap_start <= integer(row, "startSample") <= gap_end
            and number(row, "scoreMargin") >= args.minimum_candidate_margin
        ]
        candidates.sort(key=lambda row: integer(row, "startSample"))
        overview_name = f"miss-after-{after_number:04d}-overview.png"
        render_gap_overview(
            os.path.join(args.output_dir, overview_name),
            after_number,
            prior,
            following,
            candidates,
            data,
            source_rate,
            args.detector_rate,
        )
        gap_images.append((after_number, overview_name))
        overview_case_id = f"miss-after-{after_number:04d}-overview"
        manifest.append({
            "caseId": overview_case_id,
            "issueType": "possible-miss-gap",
            "sourceDetectionNumber": after_number,
            "candidateIndex": ";".join(row.get("index", "") for row in candidates),
            "startSample": gap_start,
            "endSample": gap_end,
            "timeOffsetS": f"{gap_start / args.detector_rate:.6f}",
            "durationS": f"{(gap_end - gap_start) / args.detector_rate:.6f}",
            "centerFrequencyHz": "",
            "frequencySpanHz": "",
            "frequencyDriftHz": "",
            "frameCount": "",
            "scoreMargin": "",
            "rejectionReason": "gap-overview",
            "imageFile": overview_name,
            "label": "",
            "eventId": "",
            "expectedStartS": "",
            "expectedEndS": "",
            "expectedLowHz": "",
            "expectedHighHz": "",
            "confidence": "",
            "notes": "",
        })

        for sequence, candidate in enumerate(candidates, 1):
            case_id = f"miss-after-{after_number:04d}-{sequence:02d}"
            image_name = f"{case_id}.png"
            render_candidate_case(
                os.path.join(args.output_dir, image_name),
                f"Possible miss after #{after_number}; rejected as {candidate.get('rejectionReason', '')}",
                candidate,
                accepted,
                data,
                source_rate,
                args.detector_rate,
            )
            manifest.append({
                "caseId": case_id,
                "issueType": "possible-miss-candidate",
                "sourceDetectionNumber": after_number,
                "candidateIndex": candidate.get("index", ""),
                "startSample": candidate.get("startSample", ""),
                "endSample": candidate.get("endSample", ""),
                "timeOffsetS": f"{number(candidate, 'startSample') / args.detector_rate:.6f}",
                "durationS": candidate.get("durationS", ""),
                "centerFrequencyHz": candidate.get("centerFrequencyHz", ""),
                "frequencySpanHz": candidate.get("frequencySpanHz", ""),
                "frequencyDriftHz": candidate.get("frequencyDriftHz", ""),
                "frameCount": candidate.get("frameCount", ""),
                "scoreMargin": candidate.get("scoreMargin", ""),
                "rejectionReason": candidate.get("rejectionReason", ""),
                "imageFile": image_name,
                "label": "",
                "eventId": "",
                "expectedStartS": "",
                "expectedEndS": "",
                "expectedLowHz": "",
                "expectedHighHz": "",
                "confidence": "",
                "notes": "",
            })

    excluded = set(args.span_detections) | set(args.miss_after)
    ordinary_controls, wide_controls = choose_controls(
        accepted,
        excluded,
        args.correct_controls,
        args.wide_controls,
        args.seed,
    )

    for issue_type, detections in (
        ("span-too-narrow", [accepted_by_number[number] for number in args.span_detections]),
        ("ordinary-control", ordinary_controls),
        ("wide-control", wide_controls),
    ):
        for detection in detections:
            case_id = f"{issue_type}-{detection['number']:04d}"
            image_name = f"{case_id}.png"
            render_detection_case(
                os.path.join(args.output_dir, image_name),
                issue_type.replace("-", " ").title(),
                detection,
                data,
                source_rate,
                args.detector_rate,
            )
            manifest.append({
                "caseId": case_id,
                "issueType": issue_type,
                "sourceDetectionNumber": detection["number"],
                "candidateIndex": "",
                "startSample": detection["startSample"],
                "endSample": detection["endSample"],
                "timeOffsetS": f"{detection['startSample'] / args.detector_rate:.6f}",
                "durationS": f"{detection['durationS']:.6f}",
                "centerFrequencyHz": f"{detection['centerFrequencyHz']:.6f}",
                "frequencySpanHz": f"{detection['frequencySpanHz']:.6f}",
                "frequencyDriftHz": f"{detection['frequencyDriftHz']:.6f}",
                "frameCount": "",
                "scoreMargin": "",
                "rejectionReason": "",
                "imageFile": image_name,
                "label": "",
                "eventId": "",
                "expectedStartS": "",
                "expectedEndS": "",
                "expectedLowHz": "",
                "expectedHighHz": "",
                "confidence": "",
                "notes": "",
            })

    write_manifest(os.path.join(args.output_dir, "review.csv"), manifest)
    write_index(os.path.join(args.output_dir, "index.html"), manifest, gap_images)
    print(f"WAV source rate: {source_rate} Hz")
    print(f"Possible-miss gaps: {sum(row['issueType'] == 'possible-miss-gap' for row in manifest)}")
    print(f"Possible-miss candidates: {sum(row['issueType'] == 'possible-miss-candidate' for row in manifest)}")
    print(f"Span cases: {sum(row['issueType'] == 'span-too-narrow' for row in manifest)}")
    print(f"Ordinary controls: {sum(row['issueType'] == 'ordinary-control' for row in manifest)}")
    print(f"Wide controls: {sum(row['issueType'] == 'wide-control' for row in manifest)}")


if __name__ == "__main__":
    main()
