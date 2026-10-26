# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
# Shared setup for run_kfgins.sh and run_inslib.sh (sourced, not run).

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Third-party sources, builds and runs live here, outside the tracked tree (git-ignored).
WORK="${INS_BENCHMARK_DIR:-$REPO/output/benchmark}"
VENV="$WORK/venv"

usage() {
  echo "usage: $0 <record> [ins_benchmark.py options, e.g. --mount-yaw-deg 40 --outage 560:10]" >&2
  exit 1
}

# fetch <name> <url> <commit>: clones once, then pins the checkout to <commit>.
fetch() {
  local dir="$WORK/src/$1"
  if [ ! -d "$dir/.git" ]; then
    git clone --quiet "$2" "$dir"
  fi
  git -C "$dir" checkout --quiet "$3"
  git -C "$dir" submodule update --init --quiet
}

make_venv() {
  if [ ! -x "$VENV/bin/python" ]; then
    python3 -m venv "$VENV"
    "$VENV/bin/pip" install --quiet numpy pyyaml
  fi
}

# record stem, e.g. data/mobili_2.record -> mobili_2
record_name() {
  local name
  name="$(basename "$1")"
  echo "${name%.*}"
}
