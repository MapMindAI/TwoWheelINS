#!/usr/bin/env python3
# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
"""Error of an estimated TUM trajectory against a ground-truth TUM trajectory, both IMU poses in
the same world frame (z up), e.g. run_dataset_main output vs. a benchmark dataset's gt.txt.

Each estimate pose is compared with the truth pose closest in time (within 5 ms). Prints median,
p95 and max of the horizontal, vertical, attitude and yaw error.

usage: tools/trajectory_error.py <estimate.tum> <truth.tum>
"""
import bisect
import math
import sys

MAX_TIME_OFFSET_S = 0.005


def read_tum(path):
    poses = []
    with open(path) as f:
        for line in f:
            fields = line.split()
            if fields and not line.startswith("#"):
                poses.append(tuple(map(float, fields[:8])))
    poses.sort()
    return poses


def rotation_matrix(qx, qy, qz, qw):
    return [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]]


def errors(estimate, truth):
    """Returns (t, horizontal_m, vertical_m, attitude_deg, yaw_deg) per matched pose."""
    truth_times = [pose[0] for pose in truth]
    result = []
    for pose in estimate:
        i = bisect.bisect_left(truth_times, pose[0])
        candidates = [j for j in (i - 1, i) if 0 <= j < len(truth)]
        j = min(candidates, key=lambda k: abs(truth_times[k] - pose[0]))
        if abs(truth_times[j] - pose[0]) > MAX_TIME_OFFSET_S:
            continue
        reference = truth[j]
        horizontal = math.hypot(pose[1] - reference[1], pose[2] - reference[2])
        vertical = abs(pose[3] - reference[3])
        # rotation error in the world frame: R_estimate R_truth^T
        a, b = rotation_matrix(*pose[4:8]), rotation_matrix(*reference[4:8])
        e = [[sum(a[r][k] * b[c][k] for k in range(3)) for c in range(3)] for r in range(3)]
        trace = max(-1.0, min(3.0, e[0][0] + e[1][1] + e[2][2]))
        attitude = math.degrees(math.acos((trace - 1.0) / 2.0))
        yaw = abs(math.degrees(math.atan2(e[1][0] - e[0][1], e[0][0] + e[1][1])))
        result.append((pose[0], horizontal, vertical, attitude, yaw))
    return result


def percentile(values, fraction):
    values = sorted(values)
    return values[min(len(values) - 1, int(fraction * len(values)))]


def report(name, rows):
    if not rows:
        print(f"{name}: no matched poses")
        return
    print(f"{name} ({len(rows)} poses, {rows[0][0]:.1f}-{rows[-1][0]:.1f} s)")
    for column, label, unit in ((1, "horizontal", "m"), (2, "vertical", "m"),
                                (3, "attitude", "deg"), (4, "yaw", "deg")):
        values = [row[column] for row in rows]
        print(f"  {label:10s} median {percentile(values, 0.5):7.2f}  p95 "
              f"{percentile(values, 0.95):7.2f}  max {max(values):7.2f} {unit}")


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    report("all", errors(read_tum(sys.argv[1]), read_tum(sys.argv[2])))


if __name__ == "__main__":
    main()
