#!/usr/bin/env python3
# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
"""HTML report of benchmark/run_datasets.sh: a summary table of all datasets, then one tab per
dataset with the top-down trajectory (estimate, ground truth, and the noisy GPS fixes coloured by
satellite count, each joined to the estimate pose at its time) and the horizontal and height error
over time. Self-contained (inline SVG), no packages.

usage: tools/benchmark_report.py <runs_dir> <datasets_dir> <report.html> [title]
  <runs_dir>/<name>.tum: run_dataset_main output, with "# gps t_s x y z satellites" lines
"""
import bisect
import html
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trajectory_error import MAX_TIME_OFFSET_S, errors, percentile, read_tum  # noqa: E402

MAX_LINE_POINTS = 3000
MIN_SATELLITES = 4
MAX_SATELLITES = 30
MIN_FLOAT_SATELLITES = 11
MIN_FIX_SATELLITES = 21
PLOT_WIDTH = 900
TRAJECTORY_HEIGHT = 640
ERROR_HEIGHT = 240
MARGIN = 48


def read_gps(path):
    """Returns (t, x, y, z, satellites) of the "# gps" lines."""
    fixes = []
    with open(path) as f:
        for line in f:
            if line.startswith("# gps "):
                fields = line.split()[2:]
                fixes.append(tuple(map(float, fields[:4])) + (int(fields[4]),))
    return fixes


def nearest(poses, times, t):
    i = bisect.bisect_left(times, t)
    candidates = [j for j in (i - 1, i) if 0 <= j < len(poses)]
    if not candidates:
        return None
    j = min(candidates, key=lambda k: abs(times[k] - t))
    return poses[j] if abs(times[j] - t) <= MAX_TIME_OFFSET_S else None


def thin(points):
    step = max(1, len(points) // MAX_LINE_POINTS)
    return points[::step]


def satellite_colour(count):
    # red (4 satellites, single) to green (30, RTK fix)
    fraction = (min(max(count, MIN_SATELLITES), MAX_SATELLITES) - MIN_SATELLITES) / (
        MAX_SATELLITES - MIN_SATELLITES)
    return f"hsl({round(130 * fraction)} 75% 45%)"


def solution(count):
    if count >= MIN_FIX_SATELLITES:
        return "fix"
    return "float" if count >= MIN_FLOAT_SATELLITES else "single"


def nice_step(span, target_ticks):
    raw = span / target_ticks
    magnitude = 10 ** math.floor(math.log10(raw))
    for factor in (1, 2, 5, 10):
        if raw <= factor * magnitude:
            return factor * magnitude
    return 10 * magnitude


def ticks(low, high, target_ticks):
    step = nice_step(high - low, target_ticks)
    value = math.ceil(low / step) * step
    result = []
    while value <= high + 1e-9:
        result.append(round(value, 10))
        value += step
    return result


def polyline(points, css_class):
    coordinates = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    return f'<polyline class="{css_class}" points="{coordinates}"/>'


class Axes:
    """Maps data to SVG pixels inside a plot of `width` x `height` with a margin for labels."""

    def __init__(self, width, height, x_range, y_range, equal=False):
        self.width, self.height = width, height
        (x0, x1), (y0, y1) = x_range, y_range
        if x1 - x0 < 1e-6:
            x0, x1 = x0 - 1, x1 + 1
        if y1 - y0 < 1e-6:
            y0, y1 = y0 - 1, y1 + 1
        inner_width, inner_height = width - 2 * MARGIN, height - 2 * MARGIN
        if equal:
            scale = min(inner_width / (x1 - x0), inner_height / (y1 - y0))
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            x0, x1 = cx - inner_width / scale / 2, cx + inner_width / scale / 2
            y0, y1 = cy - inner_height / scale / 2, cy + inner_height / scale / 2
        self.x_range, self.y_range = (x0, x1), (y0, y1)

    def x(self, value):
        x0, x1 = self.x_range
        return MARGIN + (value - x0) / (x1 - x0) * (self.width - 2 * MARGIN)

    def y(self, value):
        y0, y1 = self.y_range
        return self.height - MARGIN - (value - y0) / (y1 - y0) * (self.height - 2 * MARGIN)

    def frame(self, x_label, y_label):
        parts = []
        for value in ticks(*self.x_range, 8):
            x = self.x(value)
            parts.append(f'<line class="grid" x1="{x:.1f}" y1="{MARGIN}" x2="{x:.1f}" '
                         f'y2="{self.height - MARGIN}"/>'
                         f'<text class="tick" x="{x:.1f}" y="{self.height - MARGIN + 16}" '
                         f'text-anchor="middle">{value:g}</text>')
        for value in ticks(*self.y_range, 6):
            y = self.y(value)
            parts.append(f'<line class="grid" x1="{MARGIN}" y1="{y:.1f}" '
                         f'x2="{self.width - MARGIN}" y2="{y:.1f}"/>'
                         f'<text class="tick" x="{MARGIN - 6}" y="{y + 4:.1f}" '
                         f'text-anchor="end">{value:g}</text>')
        parts.append(f'<rect class="frame" x="{MARGIN}" y="{MARGIN}" '
                     f'width="{self.width - 2 * MARGIN}" height="{self.height - 2 * MARGIN}"/>'
                     f'<text class="label" x="{self.width / 2}" y="{self.height - 8}" '
                     f'text-anchor="middle">{x_label}</text>'
                     f'<text class="label" transform="translate(14 {self.height / 2}) rotate(-90)" '
                     f'text-anchor="middle">{y_label}</text>')
        return "".join(parts)

    def svg(self, body):
        return (f'<svg viewBox="0 0 {self.width} {self.height}" '
                f'xmlns="http://www.w3.org/2000/svg">'
                f'<clipPath id="c"><rect x="{MARGIN}" y="{MARGIN}" '
                f'width="{self.width - 2 * MARGIN}" height="{self.height - 2 * MARGIN}"/>'
                f'</clipPath>{body}</svg>')


def trajectory_plot(estimate, truth, gps):
    xs = [p[1] for p in truth] + [p[1] for p in estimate]
    ys = [p[2] for p in truth] + [p[2] for p in estimate]
    axes = Axes(PLOT_WIDTH, TRAJECTORY_HEIGHT, (min(xs), max(xs)), (min(ys), max(ys)), equal=True)
    estimate_times = [p[0] for p in estimate]
    links, dots = [], []
    for t, x, y, _, satellites in gps:
        pose = nearest(estimate, estimate_times, t)
        if pose:
            links.append(f'<line class="link" x1="{axes.x(x):.1f}" y1="{axes.y(y):.1f}" '
                         f'x2="{axes.x(pose[1]):.1f}" y2="{axes.y(pose[2]):.1f}"/>')
        dots.append(f'<circle cx="{axes.x(x):.1f}" cy="{axes.y(y):.1f}" r="3" '
                    f'fill="{satellite_colour(satellites)}"><title>t {t - truth[0][0]:.0f} s, '
                    f'{satellites} satellites ({solution(satellites)})</title></circle>')
    body = (axes.frame("x (m)", "y (m)") + '<g clip-path="url(#c)">' +
            polyline([(axes.x(p[1]), axes.y(p[2])) for p in thin(truth)], "truth") +
            polyline([(axes.x(p[1]), axes.y(p[2])) for p in thin(estimate)], "estimate") +
            "".join(links) + "".join(dots) + "</g>")
    return axes.svg(body)


def error_plot(rows, gps_errors, column, label, signed):
    """rows: (t, horizontal, height) of the estimate; gps_errors: (t, horizontal, height, sats)."""
    values = [row[column] for row in rows] + [e[column] for e in gps_errors]
    times = [row[0] for row in rows] + [e[0] for e in gps_errors]
    low = min(values + [0.0]) if signed else 0.0
    high = max(values + [0.0])
    axes = Axes(PLOT_WIDTH, ERROR_HEIGHT, (min(times), max(times)), (low, high))
    dots = "".join(
        f'<circle cx="{axes.x(e[0]):.1f}" cy="{axes.y(e[column]):.1f}" r="2.5" '
        f'fill="{satellite_colour(e[3])}"><title>GPS {e[column]:.2f} m, {e[3]} satellites'
        f'</title></circle>' for e in gps_errors)
    zero = (f'<line class="zero" x1="{MARGIN}" y1="{axes.y(0):.1f}" x2="{PLOT_WIDTH - MARGIN}" '
            f'y2="{axes.y(0):.1f}"/>' if signed else "")
    body = (axes.frame("time (s)", label) + '<g clip-path="url(#c)">' + zero + dots +
            polyline([(axes.x(r[0]), axes.y(r[column])) for r in thin(rows)], "estimate") +
            "</g>")
    return axes.svg(body)


def rotation_plot(matched, start):
    """matched: trajectory_error.errors rows (t, horizontal, vertical, attitude_deg, yaw_deg)."""
    rows = thin(matched)
    axes = Axes(PLOT_WIDTH, ERROR_HEIGHT, (matched[0][0] - start, matched[-1][0] - start),
                (0.0, max(max(r[3], r[4]) for r in matched)))
    body = (axes.frame("time (s)", "rotation error (°)") + '<g clip-path="url(#c)">' +
            polyline([(axes.x(r[0] - start), axes.y(r[3])) for r in rows], "estimate") +
            polyline([(axes.x(r[0] - start), axes.y(r[4])) for r in rows], "yaw") + "</g>")
    return axes.svg(body)


def statistics(values):
    return percentile(values, 0.5), percentile(values, 0.95), max(values)


def dataset_report(name, run_path, truth_path):
    truth = read_tum(truth_path)
    if not os.path.exists(run_path):
        return {"name": name, "initialized": False}
    estimate = read_tum(run_path)
    gps = read_gps(run_path)
    matched = errors(estimate, truth)
    if not matched:
        return {"name": name, "initialized": False}
    start = truth[0][0]
    truth_times = [p[0] for p in truth]
    # signed height error for the plot, absolute in the summary
    estimate_by_time = {p[0]: p for p in estimate}
    rows = []
    for t, horizontal, _, _, _ in matched:
        reference = nearest(truth, truth_times, t)
        rows.append((t - start, horizontal, estimate_by_time[t][3] - reference[3]))
    gps_errors = []
    for t, x, y, z, satellites in gps:
        # GPS samples are interpolated ground truth; compare with the truth interpolated likewise
        i = bisect.bisect_left(truth_times, t)
        if i == 0 or i == len(truth):
            continue
        a, b = truth[i - 1], truth[i]
        w = (t - a[0]) / (b[0] - a[0])
        reference = [a[k] + w * (b[k] - a[k]) for k in (1, 2, 3)]
        gps_errors.append((t - start, math.hypot(x - reference[0], y - reference[1]),
                           z - reference[2], satellites))
    solutions = [solution(fix[4]) for fix in gps]
    return {
        "name": name,
        "initialized": True,
        "initialized_s": estimate[0][0] - start,
        "duration_s": truth[-1][0] - start,
        "horizontal": statistics([r[1] for r in matched]),
        "vertical": statistics([r[2] for r in matched]),
        "yaw": statistics([r[4] for r in matched]),
        "gps_horizontal": statistics([e[1] for e in gps_errors]) if gps_errors else None,
        "solutions": {s: solutions.count(s) / max(len(solutions), 1) for s in
                      ("fix", "float", "single")},
        "trajectory": trajectory_plot(estimate, truth, gps),
        "horizontal_plot": error_plot(rows, gps_errors, 1, "horizontal error (m)", False),
        "height_plot": error_plot(rows, gps_errors, 2, "height error (m)", True),
        "attitude": statistics([r[3] for r in matched]),
        "rotation_plot": rotation_plot(matched, start),
    }


STYLE = """
:root { --bg: #ffffff; --fg: #1f2328; --muted: #656d76; --line: #d0d7de; --panel: #f6f8fa;
  --truth: #1f2328; --estimate: #0969da; --link: #8c959f; --yaw: #bc4c00; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) { --bg: #0d1117;
  --fg: #e6edf3; --muted: #8d96a0; --line: #30363d; --panel: #161b22; --truth: #e6edf3;
  --estimate: #4493f8; --link: #6e7681; --yaw: #f0883e; } }
:root[data-theme="dark"] { --bg: #0d1117; --fg: #e6edf3; --muted: #8d96a0; --line: #30363d;
  --panel: #161b22; --truth: #e6edf3; --estimate: #4493f8; --link: #6e7681;
  --yaw: #f0883e; }
body { background: var(--bg); color: var(--fg); margin: 0 auto; max-width: 1240px;
  padding: 16px; font: 14px/1.5 system-ui, sans-serif; }
h1 { font-size: 22px; margin: 8px 0; } h2 { font-size: 17px; margin: 20px 0 6px; }
p.note { color: var(--muted); margin: 4px 0 12px; }
.tabs { display: flex; flex-wrap: wrap; gap: 4px; border-bottom: 1px solid var(--line);
  margin: 16px 0; }
.tabs button { background: none; border: 1px solid transparent; border-bottom: none;
  color: var(--muted); cursor: pointer; font: inherit; padding: 6px 10px;
  border-radius: 6px 6px 0 0; }
.tabs button.active { border-color: var(--line); background: var(--panel); color: var(--fg); }
.tab { display: none; } .tab.active { display: block; }
.table-wrap { overflow-x: auto; }
table { border-collapse: collapse; width: 100%; font-size: 13px;
  font-variant-numeric: tabular-nums; }
th, td { border-bottom: 1px solid var(--line); padding: 4px 6px; text-align: right;
  white-space: nowrap; }
th:first-child, td:first-child { text-align: left; }
th { color: var(--muted); font-weight: 600; white-space: normal; }
td a { color: var(--estimate); cursor: pointer; text-decoration: none; }
svg { width: 100%; height: auto; display: block; background: var(--panel);
  border-radius: 6px; margin: 4px 0 8px; }
svg .grid { stroke: var(--line); stroke-width: 0.5; }
svg .frame { fill: none; stroke: var(--line); }
svg .tick, svg .label { fill: var(--muted); font-size: 11px; }
svg .truth { fill: none; stroke: var(--truth); stroke-width: 1.5; }
svg .estimate { fill: none; stroke: var(--estimate); stroke-width: 1.5; }
svg .yaw { fill: none; stroke: var(--yaw); stroke-width: 1.2; }
svg .link { stroke: var(--link); stroke-width: 0.7; }
svg .zero { stroke: var(--muted); stroke-dasharray: 3 3; }
.legend { display: flex; flex-wrap: wrap; gap: 16px; align-items: center; color: var(--muted); }
.legend i { display: inline-block; width: 18px; height: 3px; vertical-align: middle;
  margin-right: 6px; }
.scale { display: inline-block; width: 140px; height: 10px; vertical-align: middle;
  margin: 0 6px; border-radius: 2px;
  background: linear-gradient(90deg, hsl(0 75% 45%), hsl(65 75% 45%), hsl(130 75% 45%)); }
"""

SCRIPT = """
function show(id) {
  document.querySelectorAll('.tab').forEach(t => t.classList.toggle('active', t.id === id));
  document.querySelectorAll('.tabs button').forEach(
      b => b.classList.toggle('active', b.dataset.tab === id));
  try { history.replaceState(null, '', '#' + id); } catch (e) {}
}
document.querySelectorAll('[data-tab]').forEach(b => b.addEventListener('click', () =>
    show(b.dataset.tab)));
show(document.getElementById(location.hash.slice(1)) ? location.hash.slice(1) : 'summary');
"""


def number(values, index=None):
    if values is None:
        return "–"
    return f"{values[index]:.2f}" if index is not None else f"{values:.1f}"


def legend():
    return ('<div class="legend"><span><i style="background:var(--truth)"></i>ground truth</span>'
            '<span><i style="background:var(--estimate)"></i>estimate</span>'
            '<span><i style="background:var(--link)"></i>GPS fix to estimate at its time</span>'
            f'<span>GPS satellites {MIN_SATELLITES}<span class="scale"></span>'
            f'{MAX_SATELLITES} (single &lt; {MIN_FLOAT_SATELLITES} ≤ float &lt; '
            f'{MIN_FIX_SATELLITES} ≤ fix)</span></div>')


def write_report(reports, title, path):
    def tab_id(name):
        return "d-" + "".join(c if c.isalnum() else "-" for c in name)

    rows = []
    for r in reports:
        link = f'<a data-tab="{tab_id(r["name"])}">{html.escape(r["name"])}</a>'
        if not r["initialized"]:
            rows.append(f'<tr><td>{link}</td><td colspan="12">not initialized</td></tr>')
            continue
        s = r["solutions"]
        rows.append(
            f'<tr><td>{link}</td><td>{r["duration_s"]:.0f}</td><td>{r["initialized_s"]:.1f}</td>'
            f'<td>{number(r["horizontal"], 0)}</td><td>{number(r["horizontal"], 1)}</td>'
            f'<td>{number(r["horizontal"], 2)}</td><td>{number(r["vertical"], 0)}</td>'
            f'<td>{number(r["vertical"], 1)}</td><td>{number(r["vertical"], 2)}</td>'
            f'<td>{r["attitude"][0]:.1f}</td><td>{r["yaw"][0]:.1f}</td>'
            f'<td>{number(r["gps_horizontal"], 0)}</td>'
            f'<td>{100 * s["fix"]:.0f} / {100 * s["float"]:.0f} / {100 * s["single"]:.0f}</td>'
            "</tr>")
    summary = (
        '<div class="tab" id="summary"><h2>Summary</h2>'
        '<p class="note">Error of the estimate against gt.txt at every estimated pose after '
        'initialization. Click a dataset for its plots.</p><div class="table-wrap"><table>'
        '<tr><th rowspan="2">dataset</th><th rowspan="2">length s</th><th rowspan="2">init s</th>'
        '<th colspan="3">horizontal m</th><th colspan="3">height m</th><th colspan="2">rotation ° median</th>'
        '<th>GPS horizontal m</th><th>GPS fix/float/single %</th></tr>'
        '<tr><th>median</th><th>p95</th><th>max</th><th>median</th><th>p95</th><th>max</th>'
        '<th>attitude</th><th>yaw</th><th>median</th><th></th></tr>' + "".join(rows) + "</table></div></div>")

    tabs = ['<button data-tab="summary">Summary</button>'] + [
        f'<button data-tab="{tab_id(r["name"])}">{html.escape(r["name"])}</button>'
        for r in reports]
    pages = [summary]
    for r in reports:
        page = f'<div class="tab" id="{tab_id(r["name"])}"><h2>{html.escape(r["name"])}</h2>'
        if not r["initialized"]:
            pages.append(page + '<p class="note">Not initialized.</p></div>')
            continue
        page += (f'<p class="note">horizontal median {r["horizontal"][0]:.2f} m, p95 '
                 f'{r["horizontal"][1]:.2f} m; height median {r["vertical"][0]:.2f} m, p95 '
                 f'{r["vertical"][1]:.2f} m; attitude median {r["attitude"][0]:.1f}°, yaw median '
                 f'{r["yaw"][0]:.1f}°; initialized after '
                 f'{r["initialized_s"]:.1f} s.</p>' + legend() +
                 "<h2>Trajectory (top-down)</h2>" + r["trajectory"] +
                 '<h2>Horizontal error</h2><p class="note">Line: estimate. Dots: GPS fixes.</p>' +
                 r["horizontal_plot"] +
                 '<h2>Height error</h2><p class="note">Estimate minus ground truth. Line: '
                 'estimate. Dots: GPS fixes.</p>' + r["height_plot"] +
                 '<h2>Rotation error</h2><p class="note"><span class="legend"><span><i '
                 'style="background:var(--estimate)"></i>attitude (total)</span><span><i '
                 'style="background:var(--yaw)"></i>yaw</span></span></p>' + r["rotation_plot"] +
                 "</div>")
        pages.append(page)
    document = (f'<!doctype html><html lang="en"><head><meta charset="utf-8">'
                f'<meta name="viewport" content="width=device-width, initial-scale=1">'
                f'<title>Benchmark report</title><style>{STYLE}</style></head><body>'
                f'<h1>Benchmark report</h1><p class="note">{html.escape(title)}</p>'
                f'<nav class="tabs">{"".join(tabs)}</nav>{"".join(pages)}'
                f'<script>{SCRIPT}</script></body></html>')
    with open(path, "w") as f:
        f.write(document)


def main():
    if len(sys.argv) not in (4, 5):
        sys.exit(__doc__)
    runs_dir, datasets_dir, output = sys.argv[1:4]
    title = sys.argv[4] if len(sys.argv) == 5 else "run_dataset_main"
    names = sorted(n for n in os.listdir(datasets_dir)
                   if os.path.exists(os.path.join(datasets_dir, n, "gt.txt")))
    reports = [dataset_report(n, os.path.join(runs_dir, n + ".tum"),
                              os.path.join(datasets_dir, n, "gt.txt")) for n in names]
    write_report(reports, title, output)


if __name__ == "__main__":
    main()
