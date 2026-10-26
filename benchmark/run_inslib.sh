#!/usr/bin/env bash
# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
# Runs INSLIB (https://github.com/jnz/INSLIB, AGPL-3.0) on a MOKUKU record and writes
# output/<record>_inslib.tum. INSLIB is fetched and built under output/benchmark, never committed.
#
# usage: benchmark/run_inslib.sh data/mobili_2.record [--mount-yaw-deg 40] [--outage 560:10 ...]

source "$(dirname "$0")/common.sh"
[ $# -ge 1 ] || usage
RECORD="$(realpath "$1")"
shift

INSLIB_COMMIT=4aa8040cbee55a194988717042f8de8d8dd22196
fetch INSLIB https://github.com/jnz/INSLIB.git "$INSLIB_COMMIT"
SRC="$WORK/src/INSLIB"
make_venv
if ! "$VENV/bin/python" -c "import INSLIB" 2> /dev/null; then
  make -C "$SRC" pylib > /dev/null
  "$VENV/bin/pip" install --quiet -e "$SRC/python"
fi

NAME="$(record_name "$RECORD")"
"$VENV/bin/python" "$REPO/benchmark/ins_benchmark.py" inslib "$RECORD" "$WORK/$NAME/inslib" \
  "$REPO/output/${NAME}_inslib.tum" --replay "$SRC/tools/replay.py" "$@"
