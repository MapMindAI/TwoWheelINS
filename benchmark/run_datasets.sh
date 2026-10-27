#!/usr/bin/env bash
# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
# Runs InsFusion on every benchmark/datasets/<name> (fusion/run_dataset_main: drifting GPS noise,
# seed --seed) and prints its error against gt.txt (tools/trajectory_error.py).
# Estimates go to output/benchmark/runs/<name>.tum. Extra arguments go to run_dataset_main.
#
#   benchmark/run_datasets.sh [--seed N] ...
set -euo pipefail
cd "$(dirname "$0")/.."
bazel build //fusion:run_dataset_main >/dev/null 2>&1
mkdir -p output/benchmark/runs
printf '%-48s %14s %14s %10s\n' dataset "horizontal_m" "vertical_m" "yaw_deg"
printf '%-48s %14s %14s %10s\n' "" "median/p95" "median/p95" "median"
for dataset in benchmark/datasets/*/; do
  name=$(basename "${dataset}")
  output="output/benchmark/runs/${name}.tum"
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
