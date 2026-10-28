#!/usr/bin/env bash
# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
# Runs InsFusion on every benchmark/datasets/<name> (fusion/run_dataset_main: drifting GPS noise,
# seed --seed), then INSLIB on the same noisy GPS (run_inslib_datasets.sh), prints their errors
# against gt.txt (tools/trajectory_error.py) and writes the estimates <name>.tum and
# inslib/<name>.tum and the HTML report comparing them, report.html (tools/benchmark_report.py), to
# output/benchmark/runs/<YYYYmmdd_HHMMSS>/. Extra arguments go to run_dataset_main.
#
#   benchmark/run_datasets.sh [--seed N] ...
set -euo pipefail
cd "$(dirname "$0")/.."
bazel build //fusion:run_dataset_main >/dev/null 2>&1
started=$(date '+%Y-%m-%d %H:%M:%S')
runs="output/benchmark/runs/$(date -d "${started}" '+%Y%m%d_%H%M%S')"
mkdir -p "${runs}"
echo "InsFusion"
printf '%-48s %14s %14s %10s\n' dataset "horizontal_m" "vertical_m" "yaw_deg"
printf '%-48s %14s %14s %10s\n' "" "median/p95" "median/p95" "median"
for dataset in benchmark/datasets/*/; do
  name=$(basename "${dataset}")
  output="${runs}/${name}.tum"
  if ! bazel-bin/fusion/run_dataset_main "$@" "${dataset}" "${output}" >/dev/null 2>&1; then
    printf '%-48s %s\n' "${name}" "not initialized"
    continue
  fi
  tools/trajectory_error.py "${output}" "${dataset}gt.txt" | awk -v name="${name}" '
    $1 == "horizontal" { horizontal = $3 "/" $5 }
    $1 == "vertical" { vertical = $3 "/" $5 }
    $1 == "yaw" { yaw = $3 }
    END { printf "%-48s %14s %14s %10s\n", name, horizontal, vertical, yaw }'
done
echo "INSLIB"
benchmark/run_inslib_datasets.sh "${runs}"
tools/benchmark_report.py benchmark/datasets "${runs}/report.html" \
  "run_dataset_main $* ($(git rev-parse --short HEAD), ${started})" \
  "InsFusion=${runs}" "INSLIB=${runs}/inslib"
echo "report: ${runs}/report.html"
