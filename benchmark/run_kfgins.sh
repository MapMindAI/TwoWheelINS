#!/usr/bin/env bash
# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
# Runs KF-GINS (https://github.com/i2Nav-WHU/KF-GINS, GPL-3.0) on a MOKUKU record and writes
# output/<record>_kfgins.tum. KF-GINS is fetched and built under output/benchmark, never committed.
#
# usage: benchmark/run_kfgins.sh data/mobili_2.record [--mount-yaw-deg 40] [--outage 560:10 ...]

source "$(dirname "$0")/common.sh"
[ $# -ge 1 ] || usage
RECORD="$(realpath "$1")"
shift

KFGINS_COMMIT=8291a93e49de513fe9d21f819500d39082ded611
fetch KF-GINS https://github.com/i2Nav-WHU/KF-GINS.git "$KFGINS_COMMIT"
SRC="$WORK/src/KF-GINS"
if [ ! -x "$SRC/bin/KF-GINS" ]; then
  cmake -S "$SRC" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release > /dev/null
  cmake --build "$SRC/build" -j "$(nproc)" > /dev/null
fi
make_venv

NAME="$(record_name "$RECORD")"
"$VENV/bin/python" "$REPO/benchmark/ins_benchmark.py" kfgins "$RECORD" "$WORK/$NAME/kfgins" \
  "$REPO/output/${NAME}_kfgins.tum" --binary "$SRC/bin/KF-GINS" "$@"
