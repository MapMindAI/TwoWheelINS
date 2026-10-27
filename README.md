# TwoWheelINS

IMU + GPS inertial navigation (INS) fusion for a two-wheel platform: estimate the 6-DoF pose from
data recorded with MOKUKU.

The fusion core is a 15-state error-state INS after [INSLIB](https://github.com/jnz/INSLIB)'s
design on the float, static-memory UDU Kalman filter from [KFCore](https://github.com/jnz/KFCore)
(`third_party/kfcore/`). Production code is meant to be ported to an embedded target, so new filter
code is written in C (C11).

The previous iterated Kalman filter on manifolds (ikfom, GPL-2.0) is in release
[v0.1.0](https://github.com/MapMindAI/TwoWheelINS/releases/tag/v0.1.0).

## Layout

```
├── fusion/           # KfcoreIns (error-state INS on KFCore), InsFusion (IMU + GPS, wheel speed for standstill), ins_fusion_main, run_dataset_main
├── tools/            # WebGL/Leaflet trajectory viewer for TUM output, trajectory error and benchmark HTML report
├── third_party/      # vendored code: KFCore UDU Kalman filter (C)
├── benchmark/        # run KF-GINS / INSLIB on a record (fetched at run time); build datasets
├── common/           # shared helpers: so3/se3 Jacobians, covariance and UTM projection (math/), sensor data types (sensor/), record and benchmark dataset readers (reader/), IMU simulation and GPS noise (simulation/)
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
for every IMU sample in TUM format (`t_s x y z qx qy qz qw`). Open `tools/trajectory_viewer.html`
in a browser and pick or drop the file: it draws the trajectory in 3D (WebGL) over Esri World Imagery
and on a 2D map (Leaflet). The imagery needs network; without it the 3D view falls back to a grid.
The output also carries the record's GPS fixes as `# gps t_s easting northing altitude satellites`
comments; the viewer overlays them, coloured by satellite count.

```bash
bazel build //fusion:ins_fusion_main
bazel-bin/fusion/ins_fusion_main data/mobili_2.record output/mobili_2.tum
```

`--constrain_vertical_velocity` adds a level-road prior (near-zero vertical velocity while moving).
It holds the height when single-solution GPS altitude wanders tens of metres; leave it off for
hilly routes or accurate (RTK) altitude.

To compare against [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS) and
[INSLIB](https://github.com/jnz/INSLIB), run them on the same record. Each script fetches and
builds its library at a pinned commit under `output/benchmark/` (git-ignored; needs git, cmake, a
C/C++ compiler and python3) and writes `output/<record>_kfgins.tum` / `output/<record>_inslib.tum`
with the same GPS lines. Results are in the [wiki](https://github.com/MapMindAI/TwoWheelINS/wiki).

```bash
benchmark/run_kfgins.sh data/mobili_2.record --mount-yaw-deg 40
benchmark/run_inslib.sh data/mobili_2.record --mount-yaw-deg 40   # --outage 560:10 drops GPS
```

`benchmark/make_datasets.py` builds benchmark datasets under `benchmark/datasets/<name>/` from
the looper rosbags with their CoreMap global maps (camera IMU) and from the unilidar looper sessions
with their lidar SLAM trajectory (`/unilidar/imu`): `imu.txt` (100 Hz), `gps.txt` (IMU position at
1 Hz from the ground truth, no noise; noise is added at run time), `gt.txt` (IMU poses), plus one
top-down image per group of datasets (trajectories over their maps) under
`output/benchmark/dataset_maps/`. The
datasets are listed in the [wiki](https://github.com/MapMindAI/TwoWheelINS/wiki/Benchmark-Datasets).
It needs ROS 2 (`rosbag2_py`).

```bash
source /opt/ros/jazzy/setup.bash && /usr/bin/python3 benchmark/make_datasets.py
```

`fusion/run_dataset_main` runs `InsFusion` on one dataset: it adds GPS noise
(`common/simulation/gps_noise.h`: white noise plus a drift that random-walks in degraded stretches,
3x faster in height, and decays back in clear ones; `--seed`) with a satellite count per fix that
follows the drift (RTK fix in clear sky, float and then single as it grows), starts the yaw
selection at walking speed and bridges IMU gaps up to 1.1 s. `tools/trajectory_error.py` (python3,
no packages) compares the output with `gt.txt`; `benchmark/run_datasets.sh` does both for every
dataset, prints one line each and writes the estimates and an HTML report
(`tools/benchmark_report.py`: summary table, then per dataset the trajectory with ground truth and
the GPS fixes coloured by satellite count, and the horizontal, height and rotation error over time)
to `output/benchmark/runs/<YYYYmmdd_HHMMSS>/`.

```bash
bazel build //fusion:run_dataset_main
bazel-bin/fusion/run_dataset_main benchmark/datasets/dbot_20261023T043217Z output/run.tum
tools/trajectory_error.py output/run.tum benchmark/datasets/dbot_20261023T043217Z/gt.txt
benchmark/run_datasets.sh --seed 2
```

Personal flags go in `user.bazelrc` (git-ignored, auto-imported). CI runs `bazel test //...` on
every push to `main` and every pull request (`.github/workflows/ci.yml`).

## License

BSD-3-Clause (`LICENSE`). `third_party/kfcore/` is KFCore (BSD-3-Clause with an
attribution clause, `third_party/kfcore/LICENSE`): This product includes KFCore, developed by Jan
Zwiener.
