#!/usr/bin/env python3
# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
"""HTML report of benchmark/run_datasets.sh comparing methods run on the same noisy GPS: a summary
table of all datasets and methods, then one tab per dataset with the 3D trajectory (ground truth,
each method's estimate, and the noisy GPS fixes coloured by satellite count, each joined to the
estimates at its time; drag to orbit) and the horizontal, height and rotation error over time.
Self-contained (inline SVG and canvas), no packages.

usage: tools/benchmark_report.py <datasets_dir> <report.html> <title> NAME=DIR [NAME=DIR ...]
  DIR/<dataset>.tum: the method's estimate in the dataset frame, with "# gps t_s x y z satellites"
  lines (the GPS is read from the first method that has them)
"""
import bisect
import html
import json
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
ERROR_HEIGHT = 240
MARGIN = 48
# method i is drawn in METHOD_COLOURS[i] (light theme) / METHOD_COLOURS_DARK[i]
METHOD_COLOURS = ("#0969da", "#bc4c00", "#8250df")
METHOD_COLOURS_DARK = ("#4493f8", "#f0883e", "#ab7df8")


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

    def __init__(self, width, height, x_range, y_range):
        self.width, self.height = width, height
        (x0, x1), (y0, y1) = x_range, y_range
        if x1 - x0 < 1e-6:
            x0, x1 = x0 - 1, x1 + 1
        if y1 - y0 < 1e-6:
            y0, y1 = y0 - 1, y1 + 1
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


def trajectory_view(truth, gps, results, view_id):
    """3D view drawn by SCRIPT from inline JSON, in metres relative to the ground-truth centre."""
    centre = [sum(p[k] for p in truth) / len(truth) for k in (1, 2, 3)]

    def local(x, y, z):
        return [round(x - centre[0], 2), round(y - centre[1], 2), round(z - centre[2], 2)]

    methods = []
    for result in results:
        if result is None:
            methods.append({"estimate": [], "links": []})
            continue
        estimate = result["estimate"]
        times = [p[0] for p in estimate]
        links = []
        for t, x, y, z, _ in gps:
            pose = nearest(estimate, times, t)
            if pose:
                links.append(local(x, y, z) + local(*pose[1:4]))
        methods.append({"estimate": [local(*p[1:4]) for p in thin(estimate)], "links": links})
    data = {"truth": [local(*p[1:4]) for p in thin(truth)],
            "gps": [local(x, y, z) + [satellite_colour(n)] for _, x, y, z, n in gps],
            "methods": methods}
    return (f'<div class="view3d-bar">height &times; <select data-view="{view_id}">'
            + "".join(f'<option value="{v}"{" selected" if v == 1 else ""}>{v}</option>'
                      for v in (1, 2, 5, 10, 20)) +
            '</select><span class="note">drag: orbit, shift-drag or right-drag: pan, wheel: '
            'zoom, double-click: reset; grid 10 m</span></div>'
            f'<canvas class="view3d" id="{view_id}"></canvas>'
            f'<script type="application/json" id="{view_id}-data">{json.dumps(data)}</script>')


def error_plot(results, gps_errors, column, label, signed):
    """results: per method a method_result or None; gps_errors: (t, horizontal, height,
    satellites)."""
    rows = [r for result in results if result for r in result["rows"]]
    values = [r[column] for r in rows] + [e[column] for e in gps_errors]
    times = [r[0] for r in rows] + [e[0] for e in gps_errors]
    low = min(values + [0.0]) if signed else 0.0
    high = max(values + [0.0])
    axes = Axes(PLOT_WIDTH, ERROR_HEIGHT, (min(times), max(times)), (low, high))
    dots = "".join(
        f'<circle cx="{axes.x(e[0]):.1f}" cy="{axes.y(e[column]):.1f}" r="2.5" '
        f'fill="{satellite_colour(e[3])}"><title>GPS {e[column]:.2f} m, {e[3]} satellites'
        f'</title></circle>' for e in gps_errors)
    zero = (f'<line class="zero" x1="{MARGIN}" y1="{axes.y(0):.1f}" x2="{PLOT_WIDTH - MARGIN}" '
            f'y2="{axes.y(0):.1f}"/>' if signed else "")
    lines = "".join(
        polyline([(axes.x(r[0]), axes.y(r[column])) for r in thin(result["rows"])], f"method{i}")
        for i, result in enumerate(results) if result)
    body = axes.frame("time (s)", label) + '<g clip-path="url(#c)">' + zero + dots + lines + "</g>"
    return axes.svg(body)


def rotation_plot(results, start):
    """Attitude (solid) and yaw (dashed) error per method."""
    matched = [r for result in results if result for r in result["matched"]]
    axes = Axes(PLOT_WIDTH, ERROR_HEIGHT,
                (min(r[0] for r in matched) - start, max(r[0] for r in matched) - start),
                (0.0, max(max(r[3], r[4]) for r in matched)))
    lines = []
    for i, result in enumerate(results):
        if result:
            rows = thin(result["matched"])
            lines.append(polyline([(axes.x(r[0] - start), axes.y(r[3])) for r in rows],
                                  f"method{i}"))
            lines.append(polyline([(axes.x(r[0] - start), axes.y(r[4])) for r in rows],
                                  f"method{i} dashed"))
    body = (axes.frame("time (s)", "rotation error (°)") + '<g clip-path="url(#c)">' +
            "".join(lines) + "</g>")
    return axes.svg(body)


def statistics(values):
    return percentile(values, 0.5), percentile(values, 0.95), max(values)


def method_result(path, truth, start):
    """Errors of one method's estimate against `truth`, or None if it has no estimate."""
    if not os.path.exists(path):
        return None
    estimate = read_tum(path)
    matched = errors(estimate, truth)
    if not matched:
        return None
    truth_times = [p[0] for p in truth]
    estimate_by_time = {p[0]: p for p in estimate}
    # signed height error for the plot, absolute in the summary
    rows = [(t - start, horizontal, estimate_by_time[t][3] - nearest(truth, truth_times, t)[3])
            for t, horizontal, _, _, _ in matched]
    return {
        "estimate": estimate,
        "matched": matched,
        "rows": rows,
        "initialized_s": estimate[0][0] - start,
        "horizontal": statistics([r[1] for r in matched]),
        "vertical": statistics([r[2] for r in matched]),
        "attitude": statistics([r[3] for r in matched]),
        "yaw": statistics([r[4] for r in matched]),
    }


def gps_errors_of(gps, truth, start):
    """(t, horizontal, signed height, satellites) of each fix against the truth interpolated
    like the GPS samples were made."""
    truth_times = [p[0] for p in truth]
    result = []
    for t, x, y, z, satellites in gps:
        i = bisect.bisect_left(truth_times, t)
        if i == 0 or i == len(truth):
            continue
        a, b = truth[i - 1], truth[i]
        w = (t - a[0]) / (b[0] - a[0])
        reference = [a[k] + w * (b[k] - a[k]) for k in (1, 2, 3)]
        result.append((t - start, math.hypot(x - reference[0], y - reference[1]),
                       z - reference[2], satellites))
    return result


def dataset_report(name, methods, truth_path, view_id):
    truth = read_tum(truth_path)
    start = truth[0][0]
    paths = [os.path.join(directory, name + ".tum") for _, directory in methods]
    results = [method_result(path, truth, start) for path in paths]
    gps = next((read_gps(path) for path in paths if os.path.exists(path)), [])
    gps_errors = gps_errors_of(gps, truth, start)
    solutions = [solution(fix[4]) for fix in gps]
    report = {
        "name": name,
        "duration_s": truth[-1][0] - start,
        "results": results,
        "gps_horizontal": statistics([e[1] for e in gps_errors]) if gps_errors else None,
        "solutions": ({s: solutions.count(s) / len(solutions) for s in ("fix", "float", "single")}
                      if solutions else None),
    }
    if any(results):
        report["trajectory"] = trajectory_view(truth, gps, results, view_id)
        report["horizontal_plot"] = error_plot(results, gps_errors, 1, "horizontal error (m)",
                                               False)
        report["height_plot"] = error_plot(results, gps_errors, 2, "height error (m)", True)
        report["rotation_plot"] = rotation_plot(results, start)
    return report


def style():
    light = " ".join(f"--method{i}: {c};" for i, c in enumerate(METHOD_COLOURS))
    dark = " ".join(f"--method{i}: {c};" for i, c in enumerate(METHOD_COLOURS_DARK))
    method_lines = "".join(
        f"svg .method{i} {{ fill: none; stroke: var(--method{i}); stroke-width: 1.5; }}\n"
        f".hide-method{i} svg .method{i} {{ display: none; }}\n"
        for i in range(len(METHOD_COLOURS)))
    return f"""
:root {{ --bg: #ffffff; --fg: #1f2328; --muted: #656d76; --line: #d0d7de; --panel: #f6f8fa;
  --truth: #1f2328; {light} }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg: #0d1117;
  --fg: #e6edf3; --muted: #8d96a0; --line: #30363d; --panel: #161b22; --truth: #e6edf3;
  {dark} }} }}
:root[data-theme="dark"] {{ --bg: #0d1117; --fg: #e6edf3; --muted: #8d96a0; --line: #30363d;
  --panel: #161b22; --truth: #e6edf3; {dark} }}
body {{ background: var(--bg); color: var(--fg); margin: 0 auto; max-width: 1240px;
  padding: 16px; font: 14px/1.5 system-ui, sans-serif; }}
h1 {{ font-size: 22px; margin: 8px 0; }} h2 {{ font-size: 17px; margin: 20px 0 6px; }}
p.note {{ color: var(--muted); margin: 4px 0 12px; }}
.tabs {{ display: flex; flex-wrap: wrap; gap: 4px; border-bottom: 1px solid var(--line);
  margin: 16px 0; }}
.tabs button {{ background: none; border: 1px solid transparent; border-bottom: none;
  color: var(--muted); cursor: pointer; font: inherit; padding: 6px 10px;
  border-radius: 6px 6px 0 0; }}
.tabs button.active {{ border-color: var(--line); background: var(--panel); color: var(--fg); }}
.tab {{ display: none; }} .tab.active {{ display: block; }}
.table-wrap {{ overflow-x: auto; }}
table {{ border-collapse: collapse; width: 100%; font-size: 13px;
  font-variant-numeric: tabular-nums; }}
th, td {{ border-bottom: 1px solid var(--line); padding: 4px 6px; text-align: right;
  white-space: nowrap; }}
th:first-child, td:first-child, td.method {{ text-align: left; }}
th {{ color: var(--muted); font-weight: 600; white-space: normal; }}
td a {{ color: var(--method0); cursor: pointer; text-decoration: none; }}
td.method i {{ display: inline-block; width: 10px; height: 10px; border-radius: 2px;
  margin-right: 6px; vertical-align: -1px; }}
tr.first td {{ border-top: 1px solid var(--muted); }}
canvas.view3d {{ width: 100%; height: 600px; display: block; background: var(--panel);
  border-radius: 6px; margin: 4px 0 8px; cursor: grab; touch-action: none; }}
.view3d-bar {{ display: flex; flex-wrap: wrap; gap: 12px; align-items: center;
  color: var(--muted); }}
.view3d-bar select {{ font: inherit; }}
.view3d-bar .note {{ font-size: 12px; }}
svg {{ width: 100%; height: auto; display: block; background: var(--panel);
  border-radius: 6px; margin: 4px 0 8px; }}
svg .grid {{ stroke: var(--line); stroke-width: 0.5; }}
svg .frame {{ fill: none; stroke: var(--line); }}
svg .tick, svg .label {{ fill: var(--muted); font-size: 11px; }}
svg .zero {{ stroke: var(--muted); stroke-dasharray: 3 3; }}
{method_lines}svg .dashed {{ stroke-dasharray: 4 3; stroke-width: 1.2; }}
.legend {{ display: flex; flex-wrap: wrap; gap: 16px; align-items: center; color: var(--muted); }}
.legend i {{ display: inline-block; width: 18px; height: 3px; vertical-align: middle;
  margin-right: 6px; }}
.legend label {{ cursor: pointer; }}
.scale {{ display: inline-block; width: 140px; height: 10px; vertical-align: middle;
  margin: 0 6px; border-radius: 2px;
  background: linear-gradient(90deg, hsl(0 75% 45%), hsl(65 75% 45%), hsl(130 75% 45%)); }}
"""


SCRIPT = """
const views = {};

function View(canvas) {
  const data = JSON.parse(document.getElementById(canvas.id + '-data').textContent);
  const tab = canvas.closest('.tab');
  const all = data.truth.concat(...data.methods.map(m => m.estimate));
  const extent = Math.max(1, ...all.map(p => Math.hypot(p[0], p[1])));
  const minZ = Math.min(...data.truth.map(p => p[2]));
  const home = {azimuth: -0.6, elevation: 0.9, scale: 0, panX: 0, panY: 0};
  let view = Object.assign({}, home), exaggeration = 1;

  function project(p, width, height) {
    // orbit about the vertical, then tilt: elevation pi/2 is top-down
    const ca = Math.cos(view.azimuth), sa = Math.sin(view.azimuth);
    const x = ca * p[0] - sa * p[1], y = sa * p[0] + ca * p[1];
    const z = (p[2] - minZ) * exaggeration;
    const u = x, v = y * Math.sin(view.elevation) + z * Math.cos(view.elevation);
    return [width / 2 + view.panX + view.scale * u, height / 2 + view.panY - view.scale * v];
  }

  function draw() {
    const ratio = window.devicePixelRatio || 1;
    const width = canvas.clientWidth, height = canvas.clientHeight;
    if (!width) return;
    canvas.width = width * ratio; canvas.height = height * ratio;
    const g = canvas.getContext('2d');
    g.setTransform(ratio, 0, 0, ratio, 0, 0);
    if (!view.scale) view.scale = home.scale = 0.45 * Math.min(width, height) / extent;
    const style = getComputedStyle(document.documentElement);
    const colour = name => style.getPropertyValue(name).trim();
    const line = (points, stroke, widthPx) => {
      g.beginPath(); g.strokeStyle = stroke; g.lineWidth = widthPx;
      points.forEach((p, i) => { const q = project(p, width, height);
        i ? g.lineTo(q[0], q[1]) : g.moveTo(q[0], q[1]); });
      g.stroke();
    };
    // ground grid at the lowest ground-truth height
    const step = 10, reach = Math.ceil(extent / step) * step;
    g.globalAlpha = 0.6;
    for (let c = -reach; c <= reach; c += step) {
      line([[c, -reach, minZ], [c, reach, minZ]], colour('--line'), 1);
      line([[-reach, c, minZ], [reach, c, minZ]], colour('--line'), 1);
    }
    const shown = data.methods.map((m, i) => !tab.classList.contains('hide-method' + i));
    g.globalAlpha = 0.5;
    data.methods.forEach((m, i) => { if (shown[i])
      m.links.forEach(l => line([l.slice(0, 3), l.slice(3)], colour('--method' + i), 0.7)); });
    g.globalAlpha = 1;
    line(data.truth, colour('--truth'), 1.5);
    data.methods.forEach((m, i) => { if (shown[i]) line(m.estimate, colour('--method' + i), 1.5); });
    data.gps.forEach(f => { const q = project(f, width, height);
      g.beginPath(); g.fillStyle = f[3]; g.arc(q[0], q[1], 3, 0, 2 * Math.PI); g.fill(); });
  }

  let drag = null;
  canvas.addEventListener('contextmenu', e => e.preventDefault());
  canvas.addEventListener('pointerdown', e => {
    drag = {x: e.clientX, y: e.clientY, pan: e.shiftKey || e.button === 2};
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener('pointermove', e => {
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    drag.x = e.clientX; drag.y = e.clientY;
    if (drag.pan) { view.panX += dx; view.panY += dy; }
    else {
      view.azimuth += dx * 0.01;
      view.elevation = Math.min(Math.PI / 2, Math.max(0, view.elevation + dy * 0.01));
    }
    draw();
  });
  canvas.addEventListener('pointerup', () => { drag = null; });
  canvas.addEventListener('wheel', e => {
    e.preventDefault(); view.scale *= Math.exp(-e.deltaY * 0.001); draw();
  }, {passive: false});
  canvas.addEventListener('dblclick', () => { view = Object.assign({}, home); draw(); });
  document.querySelector(`select[data-view="${canvas.id}"]`).addEventListener('change', e => {
    exaggeration = Number(e.target.value); draw();
  });
  return draw;
}

function redraw() { Object.values(views).forEach(draw => draw()); }

function show(id) {
  document.querySelectorAll('.tab').forEach(t => t.classList.toggle('active', t.id === id));
  document.querySelectorAll('.tabs button').forEach(
      b => b.classList.toggle('active', b.dataset.tab === id));
  try { history.replaceState(null, '', '#' + id); } catch (e) {}
  const canvas = document.querySelector('#' + id + ' canvas.view3d');
  if (canvas) { views[canvas.id] = views[canvas.id] || View(canvas); views[canvas.id](); }
}
document.querySelectorAll('[data-tab]').forEach(b => b.addEventListener('click', () =>
    show(b.dataset.tab)));
document.querySelectorAll('input[data-method]').forEach(box => box.addEventListener('change',
    () => { box.closest('.tab').classList.toggle('hide-method' + box.dataset.method,
                                                 !box.checked); redraw(); }));
window.addEventListener('resize', redraw);
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', redraw);
show(document.getElementById(location.hash.slice(1)) ? location.hash.slice(1) : 'summary');
"""


def number(values, index):
    return "–" if values is None else f"{values[index]:.2f}"


def legend(method_names):
    methods = "".join(
        f'<label><input type="checkbox" data-method="{i}" checked> <i '
        f'style="background:var(--method{i})"></i>{html.escape(name)}</label>'
        for i, name in enumerate(method_names))
    return ('<div class="legend"><span><i style="background:var(--truth)"></i>ground truth</span>'
            f'{methods}<span>thin lines: GPS fix to each estimate at its time</span>'
            f'<span>GPS satellites {MIN_SATELLITES}<span class="scale"></span>'
            f'{MAX_SATELLITES} (single &lt; {MIN_FLOAT_SATELLITES} ≤ float &lt; '
            f'{MIN_FIX_SATELLITES} ≤ fix)</span></div>')


def summary_rows(report, method_names, link):
    span = len(method_names)
    s = report["solutions"]
    gps = (f'<td rowspan="{span}">{number(report["gps_horizontal"], 0)}</td>'
           f'<td rowspan="{span}">' +
           (f'{100 * s["fix"]:.0f} / {100 * s["float"]:.0f} / {100 * s["single"]:.0f}' if s
            else "–") + "</td>")
    rows = []
    for i, (method, result) in enumerate(zip(method_names, report["results"])):
        cells = (f'<td class="method"><i style="background:var(--method{i})"></i>'
                 f'{html.escape(method)}</td>')
        if result is None:
            cells += '<td colspan="9">not initialized</td>'
        else:
            cells += (f'<td>{result["initialized_s"]:.1f}</td>' +
                      "".join(f'<td>{number(result[key], k)}</td>'
                              for key in ("horizontal", "vertical") for k in range(3)) +
                      f'<td>{result["attitude"][0]:.1f}</td><td>{result["yaw"][0]:.1f}</td>')
        if i == 0:
            rows.append(f'<tr class="first"><td rowspan="{span}">{link}</td>'
                        f'<td rowspan="{span}">{report["duration_s"]:.0f}</td>{cells}{gps}</tr>')
        else:
            rows.append(f"<tr>{cells}</tr>")
    return rows


def write_report(reports, method_names, title, path):
    def tab_id(name):
        return "d-" + "".join(c if c.isalnum() else "-" for c in name)

    rows = []
    for r in reports:
        rows += summary_rows(
            r, method_names, f'<a data-tab="{tab_id(r["name"])}">{html.escape(r["name"])}</a>')
    summary = (
        '<div class="tab" id="summary"><h2>Summary</h2>'
        '<p class="note">Error of each estimate against gt.txt at every estimated pose after its '
        'initialization; all methods get the same noisy GPS. Click a dataset for its plots.</p>'
        '<div class="table-wrap"><table>'
        '<tr><th rowspan="2">dataset</th><th rowspan="2">length s</th><th rowspan="2">method</th>'
        '<th rowspan="2">init s</th><th colspan="3">horizontal m</th><th colspan="3">height m</th>'
        '<th colspan="2">rotation ° median</th><th>GPS horizontal m</th>'
        '<th>GPS fix/float/single %</th></tr>'
        '<tr><th>median</th><th>p95</th><th>max</th><th>median</th><th>p95</th><th>max</th>'
        '<th>attitude</th><th>yaw</th><th>median</th><th></th></tr>' + "".join(rows) +
        "</table></div></div>")

    tabs = ['<button data-tab="summary">Summary</button>'] + [
        f'<button data-tab="{tab_id(r["name"])}">{html.escape(r["name"])}</button>'
        for r in reports]
    pages = [summary]
    for r in reports:
        page = f'<div class="tab" id="{tab_id(r["name"])}"><h2>{html.escape(r["name"])}</h2>'
        if "trajectory" not in r:
            pages.append(page + '<p class="note">No method initialized.</p></div>')
            continue
        for method, result in zip(method_names, r["results"]):
            if result is None:
                page += f'<p class="note">{html.escape(method)}: not initialized.</p>'
                continue
            page += (f'<p class="note">{html.escape(method)}: horizontal median '
                     f'{result["horizontal"][0]:.2f} m, p95 {result["horizontal"][1]:.2f} m; '
                     f'height median {result["vertical"][0]:.2f} m, p95 '
                     f'{result["vertical"][1]:.2f} m; attitude median '
                     f'{result["attitude"][0]:.1f}°, yaw median {result["yaw"][0]:.1f}°; '
                     f'initialized after {result["initialized_s"]:.1f} s.</p>')
        page += (legend(method_names) + "<h2>Trajectory</h2>" + r["trajectory"] +
                 '<h2>Horizontal error</h2><p class="note">Lines: estimates. Dots: GPS fixes.</p>' +
                 r["horizontal_plot"] +
                 '<h2>Height error</h2><p class="note">Estimate minus ground truth. Lines: '
                 'estimates. Dots: GPS fixes.</p>' + r["height_plot"] +
                 '<h2>Rotation error</h2><p class="note">Solid: attitude (total). Dashed: yaw.'
                 '</p>' + r["rotation_plot"] + "</div>")
        pages.append(page)
    document = (f'<!doctype html><html lang="en"><head><meta charset="utf-8">'
                f'<meta name="viewport" content="width=device-width, initial-scale=1">'
                f'<title>Benchmark report</title><style>{style()}</style></head><body>'
                f'<h1>Benchmark report</h1><p class="note">{html.escape(title)}</p>'
                f'<nav class="tabs">{"".join(tabs)}</nav>{"".join(pages)}'
                f'<script>{SCRIPT}</script></body></html>')
    with open(path, "w") as f:
        f.write(document)


def main():
    if len(sys.argv) < 5 or not all("=" in a for a in sys.argv[4:]):
        sys.exit(__doc__)
    datasets_dir, output, title = sys.argv[1:4]
    methods = [tuple(a.split("=", 1)) for a in sys.argv[4:]]
    if len(methods) > len(METHOD_COLOURS):
        sys.exit(f"at most {len(METHOD_COLOURS)} methods")
    names = sorted(n for n in os.listdir(datasets_dir)
                   if os.path.exists(os.path.join(datasets_dir, n, "gt.txt")))
    reports = [dataset_report(n, methods, os.path.join(datasets_dir, n, "gt.txt"), f"view{i}")
               for i, n in enumerate(names)]
    write_report(reports, [name for name, _ in methods], title, output)


if __name__ == "__main__":
    main()
