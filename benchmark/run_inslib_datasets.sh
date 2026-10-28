#!/usr/bin/env bash
# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
# Runs INSLIB (https://github.com/jnz/INSLIB, AGPL-3.0) on every benchmark dataset with the noisy
# GPS of a benchmark/run_datasets.sh run (benchmark/inslib_dataset.py), prints its error against
# gt.txt and writes <runs_dir>/inslib/<name>.tum; run_datasets.sh calls it and reports both.
# INSLIB is fetched and built under output/benchmark, never committed.
#
# usage: benchmark/run_inslib_datasets.sh output/benchmark/runs/<YYYYmmdd_HHMMSS>

source "$(dirname "$0")/common.sh"
[ $# -eq 1 ] || { echo "usage: $0 <run_datasets.sh output dir>" >&2; exit 1; }
RUNS="$(realpath "$1")"

INSLIB_COMMIT=4aa8040cbee55a194988717042f8de8d8dd22196
fetch INSLIB https://github.com/jnz/INSLIB.git "$INSLIB_COMMIT"
SRC="$WORK/src/INSLIB"
make_venv
if ! "$VENV/bin/python" -c "import INSLIB" 2> /dev/null; then
  make -C "$SRC" pylib > /dev/null
  "$VENV/bin/pip" install --quiet -e "$SRC/python"
fi

OUT="$RUNS/inslib"
mkdir -p "$OUT"
cd "$REPO"
printf '%-48s %14s %14s %10s\n' dataset "horizontal_m" "vertical_m" "yaw_deg"
printf '%-48s %14s %14s %10s\n' "" "median/p95" "median/p95" "median"
for dataset in benchmark/datasets/*/; do
  name=$(basename "${dataset}")
  if ! grep -q '^# gps ' "$RUNS/${name}.tum" 2> /dev/null; then
    printf '%-48s %s\n' "${name}" "no InsFusion run (its GPS is reused)"
    continue
  fi
  if ! "$VENV/bin/python" benchmark/inslib_dataset.py "${dataset}" "$RUNS/${name}.tum" \
      "$OUT/work/${name}" "$OUT/${name}.tum" --replay "$SRC/tools/replay.py" > /dev/null; then
    printf '%-48s %s\n' "${name}" "failed, see $OUT/work/${name}/log.txt"
    continue
  fi
  tools/trajectory_error.py "$OUT/${name}.tum" "${dataset}gt.txt" | awk -v name="${name}" '
    $1 == "horizontal" { horizontal = $3 "/" $5 }
    $1 == "vertical" { vertical = $3 "/" $5 }
    $1 == "yaw" { yaw = $3 }
    END { printf "%-48s %14s %14s %10s\n", name, horizontal, vertical, yaw }'
done
