Mandatory rules for AI coding agents contributing to this repo. Direct user instructions win — flag the conflict when they do.

## Repository layout

TwoWheelINS (Bazel workspace `two_wheel_ins`) fuses IMU and GPS measurements recorded with MOKUKU
into a 6-DoF pose estimate. See `README.md` for the directory tree and build commands, and the
[GitHub wiki](https://github.com/MapMindAI/TwoWheelINS/wiki) for design notes.

| Path | Contents |
|---|---|
| `ikfom/` | C++17 iterated error-state Kalman filter on manifolds: `toolkit/` (MTK + esekfom, header-only), `backend/` (`ikf_ins`: IMU process model, pose/velocity measurements) |
| `common/` | Shared C++ helpers: `math/` (so3/se3 Jacobians, covariance, UTM projection), `sensor/` (IMU, GPS, wheel-speed data), `reader/` (MOKUKU text record reader), `simulation/` (simulated IMU data with ground truth) |
| `fusion/` | `InsFusion` (IMU + GPS on `ikf_ins`; wheel speed detects standstill), `ins_fusion_main` (runs it on a record, writes TUM poses in UTM), `trajectory_viewer.html` (TUM viewer: 3D via WebGL over satellite imagery, 2D map via Leaflet) |
| `MODULE.bazel` | All external deps (Bazel Central Registry); commit `MODULE.bazel.lock` with it |
| `benchmark/` | `run_kfgins.sh` / `run_inslib.sh`: fetch, build and run KF-GINS (GPL-3.0) / INSLIB (AGPL-3.0) on a record under `output/benchmark/`; `ins_benchmark.py` converts the record and writes TUM |
| `toolchain/` | aarch64 platform + cross-compile toolchain (`--config=aarch64`) |
| `.github/workflows/ci.yml` | CI: `bazel test //...` |
| `data/` | Local MOKUKU captures — git-ignored, never commit |

## 1. Language and portability

* New filter / fusion code that will ship to the embedded target is **C11** (`.c`/`.h`, `cc_library` with C sources only). No heap allocation in the per-sample path, no C++ or glog dependency, fixed-size arrays sized at compile time.
* `ikfom/` is the C++ reference. Use it to prototype and to cross-check C ports in tests; don't add embedded-only constraints to it.
* Every C module gets a `cc_test` (gtest is fine on the host side) that compares against the reference or against simulated ground truth.

## 2. Self-contained dependencies

* Bazel 9 (bzlmod only, no WORKSPACE). All external code comes through `bazel_dep` in `MODULE.bazel` at a pinned BCR version, or is vendored under `third_party/`. No git submodules, no private repos.
* Bazel 9 has no native C/C++ rules: every BUILD file loads `cc_library`/`cc_test` from `@rules_cc//cc:defs.bzl`.
* Need a helper from another codebase? Copy the minimal subset into `common/` (no protobuf or unrelated transitive deps) and give it its own `BUILD` target.
* Don't reference other private repositories by name in code, docs, or commit messages.
* The repo is open source under GPL-2.0. New MapMind files start with:
  `// Copyright <year> MapMind` and `// SPDX-License-Identifier: GPL-2.0-only`.
  Only add dependencies whose license is GPL-2.0 compatible (e.g. not Apache-2.0).
* `benchmark/` fetches KF-GINS and INSLIB at run time only. Their licenses are not GPL-2.0-compatible: never vendor, link or copy their code into the repo.
* Third-party code keeps its original copyright and license notice; add a "Modified by MapMind" line instead of replacing it (see `ikfom/toolkit/README.md`).

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

No speculative abstraction layers, plugin points, or config knobs. No compatibility shims — change every call site in the same PR. Prefer full English names; keep standard short forms (`imu`, `gps`, `ins`, `ikfom`, `ecef`, `enu`, `_ns`).
