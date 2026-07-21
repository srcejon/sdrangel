#!/usr/bin/env python3
"""Fit and evaluate the meteor candidate logistic model from labeled audit CSVs."""

import argparse
import csv
import json
import math
import os
import random
import re
from collections import defaultdict


FEATURES = (
    "peakAboveBackgroundDB",
    "maxContrastDB",
    "integratedSupportDB",
    "logPeakRatio",
    "durationS",
    "frameCount",
    "trackOccupancy",
    "frequencyCoherence",
    "frameOccupiedFraction",
    "centerFrequencyRateFraction",
    "frequencySpanRateFraction",
    "frequencyDriftRateFraction",
    "maxBandwidthRateFraction",
    "matchedEnvelopeScore",
    "quadraticSweepImprovement",
    "noiseFloorDeltaDB",
)

POSITIVE_LABELS = {"1", "meteor", "positive", "true"}
NEGATIVE_LABELS = {"0", "interference", "noise", "negative", "false", "sweep"}


def verify_feature_order_against_cpp():
    """Fail if FEATURES no longer matches MeteorDemodSink::candidateLearnedFeatures().

    The frozen model coefficients are applied positionally in C++, so the order of
    FEATURES here must match the member order in candidateLearnedFeatures() exactly.
    The C++ source cannot express the CSV column names, so parse them out of the
    sibling meteordemodsink.cpp and compare.
    """
    source_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "meteordemodsink.cpp")

    if not os.path.exists(source_path):
        print("WARNING: meteordemodsink.cpp not found; cannot verify feature order")
        return

    with open(source_path, encoding="utf-8") as handle:
        source = handle.read()

    match = re.search(
        r"candidateLearnedFeatures\([^)]*\)\s*const\s*\{\s*return\s*\{\{(.*?)\}\};",
        source,
        re.DOTALL)

    if not match:
        raise SystemExit(
            "Cannot locate candidateLearnedFeatures() in meteordemodsink.cpp; "
            "update verify_feature_order_against_cpp()")

    cpp_features = []
    for entry in match.group(1).split(","):
        member = re.search(r"m_(\w+)", entry)
        if not member:
            continue
        if "maxPeakRatio" in entry:
            cpp_features.append("logPeakRatio")
        else:
            cpp_features.append(member.group(1))

    if list(FEATURES) != cpp_features:
        raise SystemExit(
            "FEATURE ORDER MISMATCH between train_candidate_model.py and "
            "MeteorDemodSink::candidateLearnedFeatures().\n"
            f"  Python FEATURES ({len(FEATURES)}): {list(FEATURES)}\n"
            f"  C++ features    ({len(cpp_features)}): {cpp_features}\n"
            "Frozen coefficients are applied positionally in C++; fix the order "
            "before training.")


def read_rows(paths):
    rows = []
    for path in paths:
        with open(path, newline="", encoding="utf-8-sig") as handle:
            for row in csv.DictReader(handle):
                label = row.get("label", "").strip().lower()
                if label not in POSITIVE_LABELS | NEGATIVE_LABELS:
                    continue
                try:
                    features = [float(row[name]) for name in FEATURES]
                except (KeyError, TypeError, ValueError) as error:
                    raise ValueError(f"{path}: invalid feature row {row.get('index', '?')}: {error}") from error
                rows.append({
                    "group": row.get("recording", path).strip() or path,
                    "label": 1 if label in POSITIVE_LABELS else 0,
                    "features": features,
                    "row": row,
                })
    return rows


def normalization(rows):
    count = max(1, len(rows))
    means = [sum(row["features"][i] for row in rows) / count for i in range(len(FEATURES))]
    scales = []
    for i, mean in enumerate(means):
        variance = sum((row["features"][i] - mean) ** 2 for row in rows) / count
        scales.append(max(math.sqrt(variance), 1e-9))
    return means, scales


def normalized_features(row, means, scales):
    return [(value - means[i]) / scales[i] for i, value in enumerate(row["features"])]


def sigmoid(value):
    if value >= 0.0:
        return 1.0 / (1.0 + math.exp(-value))
    exponential = math.exp(value)
    return exponential / (1.0 + exponential)


def fit(rows, l2, epochs, learning_rate):
    means, scales = normalization(rows)
    weights = [0.0] * len(FEATURES)
    intercept = 0.0
    positives = sum(row["label"] for row in rows)
    negatives = len(rows) - positives
    positive_weight = len(rows) / max(1.0, 2.0 * positives)
    negative_weight = len(rows) / max(1.0, 2.0 * negatives)
    order = list(range(len(rows)))
    random.Random(0x4D4554454F52).shuffle(order)

    for epoch in range(epochs):
        gradient = [0.0] * len(weights)
        intercept_gradient = 0.0
        for index in order:
            row = rows[index]
            features = normalized_features(row, means, scales)
            probability = sigmoid(intercept + sum(w * x for w, x in zip(weights, features)))
            sample_weight = positive_weight if row["label"] else negative_weight
            residual = sample_weight * (probability - row["label"])
            intercept_gradient += residual
            for i, value in enumerate(features):
                gradient[i] += residual * value
        step = learning_rate / math.sqrt(1.0 + 0.02 * epoch)
        intercept -= step * intercept_gradient / max(1, len(rows))
        for i in range(len(weights)):
            weights[i] -= step * (gradient[i] / max(1, len(rows)) + l2 * weights[i])
    return intercept, weights, means, scales


def predict(model, row):
    intercept, weights, means, scales = model
    features = normalized_features(row, means, scales)
    return sigmoid(intercept + sum(w * x for w, x in zip(weights, features)))


def metrics(model, rows, threshold=0.5):
    true_positive = false_positive = true_negative = false_negative = 0
    loss = 0.0
    for row in rows:
        probability = min(max(predict(model, row), 1e-12), 1.0 - 1e-12)
        prediction = probability >= threshold
        label = bool(row["label"])
        loss -= label * math.log(probability) + (not label) * math.log(1.0 - probability)
        true_positive += prediction and label
        false_positive += prediction and not label
        true_negative += not prediction and not label
        false_negative += not prediction and label
    precision = true_positive / max(1, true_positive + false_positive)
    recall = true_positive / max(1, true_positive + false_negative)
    return {
        "rows": len(rows),
        "logLoss": loss / max(1, len(rows)),
        "precision": precision,
        "recall": recall,
        "specificity": true_negative / max(1, true_negative + false_positive),
        "f1": 2.0 * precision * recall / max(1e-12, precision + recall),
    }


def grouped_evaluation(rows, args):
    by_group = defaultdict(list)
    for row in rows:
        by_group[row["group"]].append(row)
    results = {}
    if len(by_group) < 2:
        return results
    for group, validation in sorted(by_group.items()):
        training = [row for row in rows if row["group"] != group]
        if len({row["label"] for row in training}) < 2:
            continue
        results[group] = metrics(
            fit(training, args.l2, args.epochs, args.learning_rate), validation)
    return results


def cpp_array(values):
    return "{{" + ", ".join(f"{value:.17g}" for value in values) + "}}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", help="Labeled candidate audit CSV files")
    parser.add_argument("--l2", type=float, default=0.05)
    parser.add_argument("--epochs", type=int, default=1000)
    parser.add_argument("--learning-rate", type=float, default=0.15)
    parser.add_argument("--boundary-csv", help="Write rows ordered by distance from probability 0.5")
    args = parser.parse_args()
    verify_feature_order_against_cpp()
    rows = read_rows(args.csv)
    if len(rows) < 4 or len({row["label"] for row in rows}) < 2:
        raise SystemExit("Need at least four labeled rows containing both positive and negative examples")

    model = fit(rows, args.l2, args.epochs, args.learning_rate)
    intercept, weights, means, scales = model
    report = {
        "features": FEATURES,
        "training": metrics(model, rows),
        "leaveOneRecordingOut": grouped_evaluation(rows, args),
        "intercept": intercept,
        "weights": weights,
        "means": means,
        "scales": scales,
    }
    print(json.dumps(report, indent=2))
    print("\nDetectorTunables model initializer:")
    print("m_learnedModelEnabled = true;")
    print(f"m_learnedModelIntercept = {intercept:.17g};")
    print(f"m_learnedModelMeans = {cpp_array(means)};")
    print(f"m_learnedModelScales = {cpp_array(scales)};")
    print(f"m_learnedModelWeights = {cpp_array(weights)};")

    if args.boundary_csv:
        ordered = sorted(rows, key=lambda row: abs(predict(model, row) - 0.5))
        fieldnames = list(ordered[0]["row"].keys()) + ["modelProbability"]
        with open(args.boundary_csv, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for row in ordered:
                output = dict(row["row"])
                output["modelProbability"] = f"{predict(model, row):.9f}"
                writer.writerow(output)


if __name__ == "__main__":
    main()
