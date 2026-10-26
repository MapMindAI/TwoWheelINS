# TwoWheelINS

IMU + GPS inertial navigation (INS) fusion for a two-wheel platform: estimate the 6-DoF pose from
data recorded with MOKUKU.

The fusion core is an iterated error-state Kalman filter on manifolds (`ikfom/`). Production code
is meant to be ported to an embedded target, so new filter code is written in C (C11); the C++
`ikfom/` reference implementation and tests are kept for development and validation.

## Layout

```
├── ikfom/            # iterated Kalman filter on manifolds (C++ reference)
│   ├── toolkit/      # MTK + esekfom (header-only), adapted from IKFoM; see its README for licensing
│   └── backend/      # ikf_ins: IMU/GPS state (pos, rot, vel, biases, gravity) and measurement models
├── fusion/           # InsFusion (IMU + GPS on ikf_ins), ins_fusion_main, WebGL trajectory viewer
├── benchmark/        # run KF-GINS / INSLIB on a record for comparison (fetched at run time)
├── common/           # shared helpers: so3/se3 Jacobians, covariance and UTM projection (math/), sensor data types (sensor/), record reader (reader/), IMU simulation (simulation/)
├── toolchain/        # aarch64 platform + cross-compile cc_toolchain
├── .github/workflows # CI: bazel test //...
├── MODULE.bazel      # external deps (Bazel Central Registry)
└── data/             # local MOKUKU captures (git-ignored)
```

Design notes (INS pipeline comparison, results, ESP32-S3 feasibility) are in the
[wiki](https://github.com/MapMindAI/TwoWheelINS/wiki).

## Build and test

Requires Bazel 9.3.1, pinned in `.bazelversion`. Install [bazelisk](https://github.com/bazelbuild/bazelisk)
as `bazel` and it fetches the pinned version; dependencies come from the Bazel Central Registry.

```bash
bazel test //...                       # host build + all tests
bazel build --config=aarch64 //...     # cross-compile for arm64 (needs gcc-aarch64-linux-gnu)
```

Run the fusion on a MOKUKU text record; it writes the IMU pose (UTM easting, northing, altitude)
for every IMU sample in TUM format (`t_s x y z qx qy qz qw`). Open `fusion/trajectory_viewer.html`
in a browser and pick or drop the file: it draws the trajectory in 3D (WebGL) over Esri World Imagery
and on a 2D map (Leaflet). The imagery needs network; without it the 3D view falls back to a grid.
The output also carries the record's GPS fixes as `# gps t_s easting northing altitude satellites`
comments; the viewer overlays them, coloured by satellite count.

```bash
bazel build //fusion:ins_fusion_main
bazel-bin/fusion/ins_fusion_main data/mobili_2.record output/mobili_2.tum
```

To compare against [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS) and
[INSLIB](https://github.com/jnz/INSLIB), run them on the same record. Each script fetches and
builds its library at a pinned commit under `output/benchmark/` (git-ignored; needs git, cmake, a
C/C++ compiler and python3) and writes `output/<record>_kfgins.tum` / `output/<record>_inslib.tum`
with the same GPS lines. Results are in the [wiki](https://github.com/MapMindAI/TwoWheelINS/wiki).

```bash
benchmark/run_kfgins.sh data/mobili_2.record --mount-yaw-deg 40
benchmark/run_inslib.sh data/mobili_2.record --mount-yaw-deg 40   # --outage 560:10 drops GPS
```

Personal flags go in `user.bazelrc` (git-ignored, auto-imported). CI runs `bazel test //...` on
every push to `main` and every pull request (`.github/workflows/ci.yml`).

## License

GPL-2.0 (`LICENSE`). `ikfom/toolkit/` is adapted from IKFoM and MTK and keeps their original
notices; see `ikfom/toolkit/README.md`.
