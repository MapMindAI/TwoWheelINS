Mandatory rules for AI coding agents contributing to this repo. Direct user instructions win — flag the conflict when they do.

## Repository layout

TwoWheelINS (Bazel workspace `two_wheel_ins`) fuses IMU and GPS measurements recorded with MOKUKU
into a 6-DoF pose estimate. See `README.md` for the directory tree and build commands, and the
[GitHub wiki](https://github.com/MapMindAI/TwoWheelINS/wiki) for design notes.

| Path | Contents |
|---|---|
| `common/` | Shared C++ helpers: `math/` (so3/se3 Jacobians, covariance, UTM projection), `sensor/` (IMU, GPS, wheel-speed data), `reader/` (MOKUKU text record reader, benchmark dataset reader), `simulation/` (simulated IMU data with ground truth, drifting GPS noise with a satellite count) |
| `fusion/` | `KfcoreIns` (15-state error-state INS on the KFCore UDU filter), `InsFusion` (IMU + GPS on `KfcoreIns`; wheel speed detects standstill), `ins_fusion_main` (runs it on a record, writes TUM poses in UTM), `run_dataset_main` (runs it on a benchmark dataset with simulated drifting GPS noise, writes TUM in the dataset frame) |
| `tools/` | `trajectory_viewer.html` (TUM viewer: 3D via WebGL over satellite imagery, 2D map via Leaflet), `trajectory_error.py` (TUM estimate vs. ground truth), `benchmark_report.py` (HTML report comparing methods on the benchmark datasets) |
| `doc/` | Design notes (`initialization.md`: how `InsFusion` levels, fits the yaw and starts) |
| `third_party/kfcore/` | KFCore UDU Kalman filter (C, unmodified; BSD-3-Clause + attribution clause) |
| `MODULE.bazel` | All external deps (Bazel Central Registry); commit `MODULE.bazel.lock` with it |
| `benchmark/` | `run_kfgins.sh` / `run_inslib.sh`: fetch, build and run KF-GINS (GPL-3.0) / INSLIB (AGPL-3.0) on a record under `output/benchmark/`; `ins_benchmark.py` converts the record and writes TUM; `run_datasets.sh` runs `run_dataset_main` and then INSLIB (`run_inslib_datasets.sh`, `inslib_dataset.py`) on every dataset with the same noisy GPS, prints their errors and writes an HTML report comparing them; `make_datasets.py` builds `benchmark/datasets/` (100 Hz IMU, 1 Hz GPS, ground truth) and map images from looper rosbags (CoreMap ground truth) and unilidar sessions (lidar SLAM ground truth) |
| `toolchain/` | aarch64 platform + cross-compile toolchain (`--config=aarch64`) |
| `.github/workflows/ci.yml` | CI: `bazel test //...` |
| `data/` | Local MOKUKU captures — git-ignored, never commit |

## 1. Language and portability

* New filter / fusion code that will ship to the embedded target is **C11** (`.c`/`.h`, `cc_library` with C sources only). No heap allocation in the per-sample path, no C++ or glog dependency, fixed-size arrays sized at compile time.
* Every C module gets a `cc_test` (gtest is fine on the host side) that compares against a dense reference (e.g. Eigen) or against simulated ground truth.
* The ikfom C++ filter was removed after release v0.1.0 (GPL-2.0); don't bring its code back.

## 2. Self-contained dependencies

* Bazel 9 (bzlmod only, no WORKSPACE). All external code comes through `bazel_dep` in `MODULE.bazel` at a pinned BCR version, or is vendored under `third_party/`. No git submodules, no private repos.
* Bazel 9 has no native C/C++ rules: every BUILD file loads `cc_library`/`cc_test` from `@rules_cc//cc:defs.bzl`.
* Need a helper from another codebase? Copy the minimal subset into `common/` (no protobuf or unrelated transitive deps) and give it its own `BUILD` target.
* Don't reference other private repositories by name in code, docs, or commit messages.
* The repo is open source under BSD-3-Clause. New MapMind files start with:
  `// Copyright <year> MapMind` and `// SPDX-License-Identifier: BSD-3-Clause`.
  Only add permissive dependencies (BSD, MIT, Apache-2.0, MPL-2.0); no GPL, LGPL or AGPL code.
* `benchmark/` fetches KF-GINS (GPL-3.0) and INSLIB (AGPL-3.0) at run time only: never vendor, link or copy their code into the repo.
* Third-party code keeps its original copyright and license notice; add a "Modified by MapMind" line instead of replacing it (see `third_party/kfcore/README.md`). KFCore requires the acknowledgment "This product includes KFCore, developed by Jan Zwiener" in every redistribution; keep it in `README.md`.

## 3. Build and test before opening a PR

* `bazel test //...` must pass (CI runs it on every PR).
* `bazel build --config=aarch64 //...` must pass when you touch C code, `toolchain/`, or deps.
* Format with `.clang-format` (Google style, 100 col); code should pass `.clang-tidy` and `CPPLINT.cfg`.
* A new test must fail when the behavior it guards is broken — check it once by breaking the code.

## 4. Update docs with the code

| Change | Update |
|---|---|
| New/changed top-level directory or build command | `README.md` + the table above |
| New dependency | `MODULE.bazel` + `MODULE.bazel.lock`, and `README.md` if it needs a host package |
| New contributor rule or workflow | this file |

## 5. Keep prose and comments concise

* Docs: one sentence beats two. Cut hedges and narration.
* Comments only when the why isn't derivable from names and structure. State units and frames (`_ns`, body/world, ENU/ECEF) for sensor quantities.

## 6. Destructive-action discipline

* No force-push, `git reset --hard`, branch deletion, or `--no-verify` without explicit confirmation.
* Never commit or overwrite `data/`.
* `third_party/` tracks upstream; don't reformat it as a drive-by.

## 7. Simplest design that solves the concrete problem

No speculative abstraction layers, plugin points, or config knobs. No compatibility shims — change every call site in the same PR. Prefer full English names; keep standard short forms (`imu`, `gps`, `ins`, `ecef`, `enu`, `_ns`).
