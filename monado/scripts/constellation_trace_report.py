#!/usr/bin/env python3
# Copyright 2026, Beyley Cardellio
#
# SPDX-License-Identifier: BSL-1.0
#
# Original author: Beyley Cardellio <ep1cm1n10n123@gmail.com>
"""
Turn the constellation tracker trace CSVs written by `monado-cli constellationbatch --trace-dir <dir>` into a plain
HTML report on how well each device was tracked.

Usage: constellation_trace_report.py <dir>/<dataset-name> [output.html]

Reads every <dir>/<dataset-name>_matching_device_<id>.csv, and <dir>/<dataset-name>_timings.csv if it exists.
Writes to stdout if no output file is given.
"""
import csv
import glob
import html
import os
import re
import sys
from collections import Counter

# Solve types that mean the device had a pose for that camera frame.
TRACKED_TYPES = {
    "predicted-prior",
    "blob-labels",
    "last-known-pose",
    "shallow-search",
    "deep-search",
}


def pct(part, whole):
    return "0.0%" if whole == 0 else "%.1f%%" % (100.0 * part / whole)


def longest_lost_run(rows):
    """Longest run of consecutive frames (in trace-idx order) where the device was not tracked."""
    longest = current = 0
    for row in sorted(rows, key=lambda r: int(r["trace-idx"])):
        if row["solve-type"] in TRACKED_TYPES:
            current = 0
        else:
            current += 1
            longest = max(longest, current)
    return longest


def table(header, rows):
    out = ["<table border=\"1\">", "<tr>" + "".join("<th>%s</th>" % html.escape(h) for h in header) + "</tr>"]
    for row in rows:
        out.append("<tr>" + "".join("<td>%s</td>" % html.escape(str(c)) for c in row) + "</tr>")
    out.append("</table>")
    return out


def percentile(sorted_values, p):
    """Nearest-rank percentile of an already sorted list."""
    index = max(0, min(len(sorted_values) - 1, int(round(p / 100.0 * len(sorted_values))) - 1))
    return sorted_values[index]


def ms(ns):
    return "%.3f" % (ns / 1e6)


STAT_HEADER = ["count", "min", "median", "mean", "p90", "p99", "p99.9", "max"]


def stat_cells(values):
    """Stats over a list of nanosecond values, formatted in milliseconds."""
    if not values:
        return [0] + ["-"] * (len(STAT_HEADER) - 1)
    v = sorted(values)
    return [
        len(v),
        ms(v[0]),
        ms(percentile(v, 50)),
        ms(sum(v) / len(v)),
        ms(percentile(v, 90)),
        ms(percentile(v, 99)),
        ms(percentile(v, 99.9)),
        ms(v[-1]),
    ]


def column(rows, name):
    return [int(r[name]) for r in rows if r.get(name, "") != ""]


def timing_table(rows, cams, metrics):
    """
    One row per (metric, camera), with "all" cameras first. metrics is a list of (label, function from row to value
    or None).
    """
    table_rows = []
    for label, get in metrics:
        groups = [("all", rows)] + [
            ("%d/%d" % c, [r for r in rows if (int(r["mosaic-idx"]), int(r["cam-idx"])) == c]) for c in cams
        ]
        for cam_label, group in groups:
            values = [x for x in (get(r) for r in group) if x is not None]
            table_rows.append([label, cam_label] + stat_cells(values))
    return table(["step", "mosaic/camera"] + STAT_HEADER, table_rows)


def field(name):
    return lambda r: int(r[name]) if r.get(name, "") != "" else None


def since_blobwatch(name):
    """Latency of a step relative to the blobwatch start, valid even if the frame timestamps use another clock."""

    def get(r):
        if r.get(name, "") == "" or r.get("blobwatch-latency-ns", "") == "":
            return None
        return int(r[name]) - int(r["blobwatch-latency-ns"])

    return get


def pipeline_end_since_blobwatch(r):
    """
    Blobwatch start to the end of matching: matching start, plus the fast path, plus the slow path if it ran. Doesn't
    include any time the sample spent waiting to be picked up by the slow thread.
    """
    start = since_blobwatch("matching-latency-ns")(r)
    if start is None or r.get("matching-fast-ns", "") == "":
        return None
    end = start + int(r["matching-fast-ns"])
    if r.get("matching-slow-ns", "") != "":
        end += int(r["matching-slow-ns"])
    return end


def clock_note(values):
    """A warning to show when latencies are negative, ie the frame timestamps aren't on the tracker's clock."""
    if values and min(values) < 0:
        return [
            "<p>Note: some latencies are negative, so the frame timestamps are not from the same clock as the "
            "tracker (eg a dataset played back with its original timestamps). Latencies from the frame timestamp are "
            "meaningless, use the ones measured from the blobwatch or fusion start instead.</p>"
        ]
    return []


def row_key(r):
    return (r["mosaic-idx"], r["cam-idx"], r["trace-idx"])


def counts_table(rows, name, label):
    counts = Counter(r[name] for r in rows)
    return table([label, "count", "share"], [[k, n, pct(n, len(rows))] for k, n in counts.most_common()])


def fusion_section(rows, timings_by_key):
    """The sensor fusion part of a device's section, from the fusion columns of its matching file."""
    if "fusion-solve-result" not in (rows[0] if rows else {}):
        return ["<p>No sensor fusion columns in this file.</p>"]

    out = ["<h3>Sensor fusion</h3>"]
    fused = [r for r in rows if r["fusion-solve-result"] != ""]
    available = [r for r in fused if r["fusion-solve-result"] == "succeeded"]
    out.append(
        "<p>Sensor fusion was given the sample in %d of %d camera frames, and its pose was made available to "
        "consumers (a solve covered it) in %d (%s of all camera frames).</p>"
        % (len(fused), len(rows), len(available), pct(len(available), len(rows)))
    )
    if not fused:
        return out

    out.append("<h4>What happened to the sample</h4>")
    out += counts_table(fused, "fusion-sample-result", "sample result")

    out.append("<h4>How the wait for a solve ended</h4>")
    out.append(
        "<p>succeeded: a solve covered it, its pose is available. failed: a solve failed while it waited, fusion "
        "reset. never-covered: it left the window before any solve covered it. not-run: dropped before it was "
        "folded in.</p>"
    )
    out += counts_table(fused, "fusion-solve-result", "solve result")

    out.append("<h4>Successful solves that skipped it before it was covered</h4>")
    waited = Counter(int(r["fusion-solves-waited"]) for r in fused if r.get("fusion-solves-waited", "") != "")
    out += table(
        ["solves waited", "count", "share"], [[k, n, pct(n, len(fused))] for k, n in sorted(waited.items())]
    )

    cams = sorted({(int(r["mosaic-idx"]), int(r["cam-idx"])) for r in fused})

    out.append("<h4>Fusion times</h4>")
    out.append(
        "<p>All times are in milliseconds. Solve is the whole fusion pass that covered (or failed on) the sample, "
        "shared by every sample it released.</p>"
    )
    out += timing_table(
        fused,
        cams,
        [
            ("fold sample in", field("fusion-process-ns")),
            ("solve", field("fusion-solve-ns")),
        ],
    )

    def since_fusion_start(r):
        if r["pose-available-latency-ns"] == "":
            return None
        return int(r["pose-available-latency-ns"]) - int(r["fusion-start-latency-ns"])

    def since_blobwatch_start(r):
        t = timings_by_key.get(row_key(r))
        if r["pose-available-latency-ns"] == "" or t is None or t.get("blobwatch-latency-ns", "") == "":
            return None
        return int(r["pose-available-latency-ns"]) - int(t["blobwatch-latency-ns"])

    out.append("<h4>Latency to the pose being available</h4>")
    out.append(
        "<p>When the fused pose from a sample could first be read through the get tracked pose interface. Blobwatch "
        "start to pose available is the whole pipeline after the frame arrived.</p>"
    )
    out += clock_note(column(fused, "fusion-start-latency-ns"))
    out += timing_table(
        fused,
        cams,
        [
            ("frame timestamp -> fusion start", field("fusion-start-latency-ns")),
            ("frame timestamp -> pose available", field("pose-available-latency-ns")),
            ("fusion start -> pose available", since_fusion_start),
            ("blobwatch start -> pose available", since_blobwatch_start),
        ],
    )

    return out


def timings_sections(rows):
    out = ["<h2>Timings</h2>", "<p>All times are in milliseconds. Steps that never ran are left out.</p>"]
    cams = sorted({(int(r["mosaic-idx"]), int(r["cam-idx"])) for r in rows})

    def present(metrics):
        return [(label, get) for label, get in metrics if any(get(r) is not None for r in rows)]

    out.append("<h3>Process times</h3>")
    out.append("<p>How long each step spent working on a frame.</p>")
    out += timing_table(
        rows,
        cams,
        present(
            [
                ("blobwatch", field("blobwatch-process-ns")),
                ("blob visualizer", field("visualizer-process-ns")),
                ("matching fast path", field("matching-fast-ns")),
                ("matching slow path", field("matching-slow-ns")),
            ]
        ),
    )

    out.append("<h3>Latency</h3>")
    out.append("<p>Time from the frame timestamp to when each step started.</p>")
    latency = present(
        [
            ("blobwatch", field("blobwatch-latency-ns")),
            ("blob visualizer", field("visualizer-latency-ns")),
            ("matching", field("matching-latency-ns")),
        ]
    )
    out += clock_note(column(rows, "blobwatch-latency-ns"))
    out += timing_table(rows, cams, latency)

    out.append("<h3>Latency since blobwatch start</h3>")
    out.append(
        "<p>Time from the blobwatch starting on a frame to when each later step started. Pipeline end is the end of "
        "matching (matching start + fast path + slow path if it ran), not counting any wait for the slow thread.</p>"
    )
    out += timing_table(
        rows,
        cams,
        present(
            [
                ("blob visualizer", since_blobwatch("visualizer-latency-ns")),
                ("matching", since_blobwatch("matching-latency-ns")),
                ("pipeline end", pipeline_end_since_blobwatch),
            ]
        ),
    )

    return out


def device_section(device_id, rows, timings_by_key):
    out = ["<h2>Device %s</h2>" % device_id]

    tracked = sum(1 for r in rows if r["solve-type"] in TRACKED_TYPES)
    out.append("<p>Tracked in %d of %d camera frames (%s).</p>" % (tracked, len(rows), pct(tracked, len(rows))))

    # Per camera
    out.append("<h3>Per camera</h3>")
    cams = sorted({(int(r["mosaic-idx"]), int(r["cam-idx"])) for r in rows})
    cam_rows = []
    for mosaic, cam in cams:
        cr = [r for r in rows if int(r["mosaic-idx"]) == mosaic and int(r["cam-idx"]) == cam]
        ct = sum(1 for r in cr if r["solve-type"] in TRACKED_TYPES)
        cam_rows.append(
            ["%d/%d" % (mosaic, cam), len(cr), ct, pct(ct, len(cr)), longest_lost_run(cr)]
        )
    out += table(["mosaic/camera", "frames", "tracked", "tracked %", "longest untracked run (frames)"], cam_rows)

    # How each frame was solved (or why it wasn't)
    out.append("<h3>Solve types</h3>")
    counts = Counter(r["solve-type"] for r in rows)
    type_rows = [
        [t, "yes" if t in TRACKED_TYPES else "no", n, pct(n, len(rows))] for t, n in counts.most_common()
    ]
    out += table(["solve type", "tracked", "count", "share"], type_rows)

    out += fusion_section(rows, timings_by_key)

    return out


def main():
    if len(sys.argv) not in (2, 3):
        print(__doc__.strip(), file=sys.stderr)
        return 1

    prefix = sys.argv[1]
    devices = {}
    for path in glob.glob(glob.escape(prefix) + "_matching_device_*.csv"):
        match = re.search(r"_matching_device_(-?\d+)\.csv$", path)
        if match:
            with open(path, newline="") as f:
                devices[int(match.group(1))] = list(csv.DictReader(f))

    if not devices:
        print("No %s_matching_device_*.csv files found" % prefix, file=sys.stderr)
        return 1

    out = ["<!DOCTYPE html>", "<html>", "<head>", "<meta charset=\"ascii\">"]
    out.append("<title>Tracking report: %s</title>" % html.escape(prefix))
    out += ["</head>", "<body>", "<h1>Tracking report: %s</h1>" % html.escape(prefix)]
    out.append(
        "<p>A device counts as tracked in a camera frame if it was solved by any of: %s.</p>"
        % html.escape(", ".join(sorted(TRACKED_TYPES)))
    )

    timings_path = prefix + "_timings.csv"
    timings = None
    if os.path.exists(timings_path):
        with open(timings_path, newline="") as f:
            timings = list(csv.DictReader(f))
    timings_by_key = {row_key(r): r for r in timings or []}

    # Overview of every device
    out.append("<h2>Summary</h2>")
    out.append(
        "<p>Pose available means a sensor fusion solve covered the sample, so its fused pose could be read through "
        "the get tracked pose interface.</p>"
    )
    summary_rows = []
    for device_id in sorted(devices):
        rows = devices[device_id]
        tracked = sum(1 for r in rows if r["solve-type"] in TRACKED_TYPES)
        available = sum(1 for r in rows if r.get("fusion-solve-result", "") == "succeeded")
        summary_rows.append(
            [device_id, len(rows), tracked, pct(tracked, len(rows)), available, pct(available, len(rows))]
        )
    out += table(
        ["device", "camera frames", "tracked", "tracked %", "pose available", "pose available %"], summary_rows
    )

    for device_id in sorted(devices):
        out += device_section(device_id, devices[device_id], timings_by_key)

    if timings is not None:
        out += timings_sections(timings)
    else:
        out.append("<p>No %s found, skipping timings.</p>" % html.escape(timings_path))

    out += ["</body>", "</html>", ""]
    text = "\n".join(out)

    if len(sys.argv) == 3:
        with open(sys.argv[2], "w", encoding="ascii") as f:
            f.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
