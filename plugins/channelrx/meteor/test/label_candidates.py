#!/usr/bin/env python3
"""Interactive web labeler for meteor candidate audits.

Serves a local page showing one candidate waterfall at a time with
one-keystroke classification buttons. Labels are written as
``startSample,endSample,lowFrequencyHz,highFrequencyHz,label,eventId`` rows.
Audit CSVs are produced by the offline research tooling in ``detector2d/``
(e.g. its review-set generators), which also consumes the labels.

Example:
    python label_candidates.py --wav rec.wav --audit rec.audit.csv \
        --labels rec.labels.csv --port 8765

The default queue shows unlabeled candidates in priority order: recovery-path
acceptances, then other acceptances, then positive-margin rejects. Use
``--queue all`` to include every candidate and ``--relabel`` to revisit
already-labeled ones. Existing labels (for example ranges seeded from a
regression expectation CSV) are respected for resume: overlapping candidates
count as labeled.
"""

import argparse
import csv
import io
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

import render_candidate_review as rcr

RECOVERY_REASONS = {"accepted-settled-envelope", "accepted-active-overlap"}
LABEL_KEYS = ("meteor", "sweep", "interference", "noise", "unsure")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wav", required=True, help="Stereo I/Q WAV recording")
    parser.add_argument("--audit", required=True, help="Candidate audit CSV for the recording")
    parser.add_argument("--labels", required=True, help="Labels CSV to read and update")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--queue", choices=("priority", "accepted", "rejected", "all", "relabel-meteors"), default="priority")
    parser.add_argument("--relabel", action="store_true", help="Include already-labeled candidates")
    parser.add_argument("--label-set", default=",".join(LABEL_KEYS),
                        help="Comma-separated label buttons (number keys 1..N select them)")
    parser.add_argument("--context-before", type=float, default=2.0)
    parser.add_argument("--context-after", type=float, default=3.0)
    return parser.parse_args()


def load_labels(path):
    labels = []

    if not os.path.exists(path):
        return labels

    with open(path, newline="", encoding="utf-8-sig") as handle:
        for line in handle:
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            fields = line.split(",")

            try:
                start = int(fields[0])
            except ValueError:
                continue

            entry = {
                "start": start,
                "end": int(fields[1]),
                "low": None,
                "high": None,
                "label": "",
                "eventId": "",
            }

            if len(fields) >= 5:
                try:
                    entry["low"] = float(fields[2])
                    entry["high"] = float(fields[3])
                    entry["label"] = fields[4].strip()
                    entry["eventId"] = ",".join(fields[5:]).strip()
                except ValueError:
                    entry["label"] = ",".join(fields[2:]).strip()
            else:
                entry["label"] = ",".join(fields[2:]).strip()

            if entry["label"]:
                labels.append(entry)

    return labels


def save_labels(path, labels):
    temporary = path + ".tmp"

    with open(temporary, "w", newline="", encoding="utf-8") as handle:
        for entry in labels:
            if entry["low"] is not None and entry["high"] is not None:
                handle.write(
                    f"{entry['start']},{entry['end']},{entry['low']:.1f},"
                    f"{entry['high']:.1f},{entry['label']},{entry['eventId']}\n")
            else:
                handle.write(f"{entry['start']},{entry['end']},{entry['label']}\n")

    os.replace(temporary, path)


def best_entry(labels, start, end):
    best = None
    best_overlap = 0

    for entry in labels:
        overlap = min(end, entry["end"]) - max(start, entry["start"]) + 1

        if overlap > best_overlap:
            best_overlap = overlap
            best = entry

    return best


def best_label(labels, start, end):
    entry = best_entry(labels, start, end)
    return entry["label"] if entry else ""


def candidate_category(row):
    accepted = row.get("accepted", "0").strip() == "1"
    reason = row.get("rejectionReason", "").strip()
    rescue = row.get("calibratedRescue", "0").strip() == "1"

    if accepted and (rescue or reason in RECOVERY_REASONS):
        return "recovery-accepted"

    if accepted:
        return "accepted"

    if rcr.number(row, "scoreMargin") >= 0.0:
        return "boundary-reject"

    return "reject"


def build_queue(rows, labels, mode, relabel):
    order = {"recovery-accepted": 0, "accepted": 1, "boundary-reject": 2, "reject": 3}

    if mode == "relabel-meteors":
        # Serve back previously labeled meteors for re-adjudication, weakest
        # first. Only labeler-written labels (frequency-bounded rows) qualify:
        # seed labels imported from a verified expectation CSV are trusted.
        queue = []
        seen_ranges = set()

        for index, row in enumerate(rows):
            start = rcr.integer(row, "startSample")
            end = rcr.integer(row, "endSample")
            entry = best_entry(labels, start, end)

            if (not entry) or (entry["label"] != "meteor") or (entry["low"] is None):
                continue

            label_range = (entry["start"], entry["end"])

            if label_range in seen_ranges:
                continue  # one review per label, not per overlapping fragment

            seen_ranges.add(label_range)
            queue.append({
                "audit_index": index,
                "category": candidate_category(row),
                "existing": "meteor",
            })

        queue.sort(key=lambda item: (
            rcr.number(rows[item["audit_index"]], "maxContrastDB"),
            rcr.number(rows[item["audit_index"]], "scoreMargin"),
        ))
        return queue

    included = {
        "priority": {"recovery-accepted", "accepted", "boundary-reject"},
        "accepted": {"recovery-accepted", "accepted"},
        "rejected": {"boundary-reject", "reject"},
        "all": set(order),
    }[mode]
    queue = []

    for index, row in enumerate(rows):
        category = candidate_category(row)

        if category not in included:
            continue

        start = rcr.integer(row, "startSample")
        end = rcr.integer(row, "endSample")
        existing = best_label(labels, start, end)

        if existing and not relabel:
            continue

        queue.append({
            "audit_index": index,
            "category": category,
            "existing": existing,
        })

    queue.sort(key=lambda item: (
        order[item["category"]],
        -rcr.number(rows[item["audit_index"]], "scoreMargin"),
    ))
    return queue


PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>Meteor candidate labeler</title>
<style>
body { background: #14161a; color: #d8dce2; font-family: system-ui, sans-serif;
       margin: 0; display: flex; flex-direction: column; align-items: center; }
#panel { max-width: 96vw; margin-top: 10px; border: 1px solid #333; min-height: 200px; }
#meta { font-size: 14px; margin: 8px; color: #9ab; }
#progress { font-size: 13px; color: #789; }
#buttons { margin: 10px; display: flex; gap: 10px; flex-wrap: wrap; justify-content: center; }
button { font-size: 15px; padding: 10px 18px; border-radius: 6px; border: 1px solid #444;
         background: #23262c; color: #d8dce2; cursor: pointer; }
button:hover { background: #2e323a; }
button b { color: #fc6; }
#done { font-size: 20px; margin: 40px; display: none; }
kbd { background: #333; border-radius: 3px; padding: 1px 5px; font-size: 12px; }
#help { font-size: 12px; color: #667; margin-bottom: 12px; }
</style></head><body>
<div id="progress"></div>
<img id="panel" alt="candidate waterfall">
<div id="meta"></div>
<div id="buttons">
%LABEL_BUTTONS%
  <button data-action="skip">Skip</button>
  <button data-action="undo">Undo</button>
  <button data-action="back">&larr; Back</button>
</div>
<div id="help">Keys: %KEY_HELP% <kbd>k</kbd>/<kbd>space</kbd> skip,
<kbd>z</kbd> undo, <kbd>&larr;</kbd> back</div>
<div id="done">All queued candidates reviewed. Labels saved.</div>
<script>
let pos = 0, total = 0;

async function show(newPos) {
  const response = await fetch(`/api/item?pos=${newPos}`);
  if (response.status === 404) {
    document.getElementById("done").style.display = "block";
    document.getElementById("panel").style.display = "none";
    document.getElementById("meta").textContent = "";
    document.getElementById("progress").textContent = `${total} candidates reviewed`;
    return;
  }
  const item = await response.json();
  pos = item.pos; total = item.total;
  document.getElementById("done").style.display = "none";
  document.getElementById("panel").style.display = "";
  document.getElementById("panel").src = `/image?pos=${pos}&r=${Date.now()}`;
  document.getElementById("progress").textContent =
    `Candidate ${pos + 1} of ${total} — ${item.labeled} labeled this session`;
  document.getElementById("meta").textContent =
    `${item.category}  t=${item.timeS}s  dur=${item.durationS}s  ` +
    `f=${item.centerHz} Hz  span=${item.spanHz} Hz  frames=${item.frames}  ` +
    `margin=${item.margin}  ${item.reason}` +
    (item.existing ? `  [current label: ${item.existing}]` : "");
}

async function label(name) {
  await fetch("/api/label", {method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({pos: pos, label: name})});
  show(pos + 1);
}

async function undo() {
  const response = await fetch("/api/undo", {method: "POST"});
  const result = await response.json();
  if (result.pos >= 0) { show(result.pos); }
}

document.getElementById("buttons").addEventListener("click", (event) => {
  const button = event.target.closest("button");
  if (!button) return;
  if (button.dataset.label) label(button.dataset.label);
  else if (button.dataset.action === "skip") show(pos + 1);
  else if (button.dataset.action === "undo") undo();
  else if (button.dataset.action === "back") show(Math.max(0, pos - 1));
});

document.addEventListener("keydown", (event) => {
  const keys = %KEY_MAP%;
  if (keys[event.key]) label(keys[event.key]);
  else if (event.key === "k" || event.key === " ") { event.preventDefault(); show(pos + 1); }
  else if (event.key === "z") undo();
  else if (event.key === "ArrowLeft") show(Math.max(0, pos - 1));
  else if (event.key === "ArrowRight") show(pos + 1);
});

show(0);
</script></body></html>"""


class LabelerState:
    def __init__(self, args):
        self.args = args
        self.label_keys = tuple(k.strip() for k in args.label_set.split(",") if k.strip())
        self.rows = rcr.read_audit(args.audit)
        self.labels = load_labels(args.labels)
        self.queue = build_queue(self.rows, self.labels, args.queue, args.relabel)
        self.source_rate, self.data = rcr.memory_map_wav(args.wav)
        self.history = []
        self.session_labeled = 0
        self.lock = threading.Lock()

    def item(self, pos):
        entry = self.queue[pos]
        row = self.rows[entry["audit_index"]]
        rate = max(1, rcr.integer(row, "sampleRate", 1000))
        start = rcr.integer(row, "startSample")
        end = rcr.integer(row, "endSample")
        return {
            "pos": pos,
            "total": len(self.queue),
            "labeled": self.session_labeled,
            "category": entry["category"],
            "existing": best_label(self.labels, start, end),
            "timeS": f"{0.5 * (start + end) / rate:.2f}",
            "durationS": row.get("durationS", ""),
            "centerHz": row.get("centerFrequencyHz", ""),
            "spanHz": row.get("frequencySpanHz", ""),
            "frames": row.get("frameCount", ""),
            "margin": row.get("scoreMargin", ""),
            "reason": row.get("rejectionReason", ""),
        }

    def render(self, pos):
        entry = self.queue[pos]
        row = self.rows[entry["audit_index"]]
        # Two scales: the full detector band exposes sweeps anywhere in the
        # channel; the zoomed view shows whether the boxed candidate itself
        # looks like a small meteor.
        figure, axes = plt.subplots(2, 1, figsize=(13, 8.6), squeeze=False)
        rcr.render_panel(
            axes[0, 0], row, entry["category"], self.data, self.source_rate,
            self.args.context_before, self.args.context_after, full_band=True)
        rcr.render_panel(
            axes[1, 0], row, "zoom", self.data, self.source_rate,
            self.args.context_before, self.args.context_after)
        buffer = io.BytesIO()
        figure.tight_layout()
        figure.savefig(buffer, format="png", dpi=110)
        plt.close(figure)
        return buffer.getvalue()

    def apply_label(self, pos, label):
        entry = self.queue[pos]
        row = self.rows[entry["audit_index"]]
        start = rcr.integer(row, "startSample")
        end = rcr.integer(row, "endSample")
        center = rcr.number(row, "centerFrequencyHz")
        span = rcr.number(row, "frequencySpanHz")
        record = {
            "start": start,
            "end": end,
            "low": center - 0.5 * span if span > 0.0 else None,
            "high": center + 0.5 * span if span > 0.0 else None,
            "label": label,
            "eventId": "",
        }
        self.labels = [
            existing for existing in self.labels
            if not (existing["start"] == start and existing["end"] == end)
        ]
        self.labels.append(record)
        self.history.append((pos, record))
        self.session_labeled += 1
        save_labels(self.args.labels, self.labels)

    def undo(self):
        if not self.history:
            return -1

        pos, record = self.history.pop()
        self.labels = [entry for entry in self.labels if entry is not record]
        self.session_labeled = max(0, self.session_labeled - 1)
        save_labels(self.args.labels, self.labels)
        return pos


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, status, content_type, body):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def query_pos(self):
            from urllib.parse import parse_qs, urlparse
            values = parse_qs(urlparse(self.path).query)
            return int(values.get("pos", ["0"])[0])

        def render_page(self):
            keys = state.label_keys
            buttons = "\n".join(
                f'  <button data-label="{k}"><b>{i+1}</b> {k}</button>' for i, k in enumerate(keys))
            key_map = json.dumps({str(i + 1): k for i, k in enumerate(keys)})
            key_help = ", ".join(f"<kbd>{i+1}</kbd> {k}" for i, k in enumerate(keys)) + ","
            return (PAGE.replace("%LABEL_BUTTONS%", buttons)
                        .replace("%KEY_MAP%", key_map)
                        .replace("%KEY_HELP%", key_help))

        def do_GET(self):
            try:
                if self.path == "/" or self.path.startswith("/?"):
                    self.send(200, "text/html; charset=utf-8", self.render_page().encode())
                elif self.path.startswith("/api/item"):
                    pos = self.query_pos()

                    if 0 <= pos < len(state.queue):
                        self.send(200, "application/json", json.dumps(state.item(pos)).encode())
                    else:
                        self.send(404, "application/json", b"{}")
                elif self.path.startswith("/image"):
                    with state.lock:
                        image = state.render(self.query_pos())

                    self.send(200, "image/png", image)
                else:
                    self.send(404, "text/plain", b"not found")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            try:
                if self.path == "/api/label":
                    length = int(self.headers.get("Content-Length", "0"))
                    request = json.loads(self.rfile.read(length) or b"{}")
                    label = request.get("label", "")
                    pos = int(request.get("pos", -1))

                    if (label in state.label_keys) and (0 <= pos < len(state.queue)):
                        with state.lock:
                            state.apply_label(pos, label)

                        self.send(200, "application/json", b'{"ok": true}')
                    else:
                        self.send(400, "application/json", b'{"ok": false}')
                elif self.path == "/api/undo":
                    with state.lock:
                        pos = state.undo()

                    self.send(200, "application/json", json.dumps({"pos": pos}).encode())
                else:
                    self.send(404, "text/plain", b"not found")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    args = parse_args()
    state = LabelerState(args)
    print(f"Queue: {len(state.queue)} candidates "
          f"({sum(1 for item in state.queue if item['category'] == 'recovery-accepted')} recovery, "
          f"{sum(1 for item in state.queue if item['category'] == 'accepted')} accepted, "
          f"{sum(1 for item in state.queue if item['category'] == 'boundary-reject')} boundary)")
    print(f"Existing labels: {len(state.labels)}")
    print(f"Serving on http://127.0.0.1:{args.port}/ — Ctrl+C to stop; labels save on every click")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(state))

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
