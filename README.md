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
├── common/           # shared helpers: so3/se3 Jacobians and pose covariance transformations (math/), IMU data types (sensor/), IMU simulation (simulation/)
├── toolchain/        # aarch64 platform + cross-compile cc_toolchain
├── .github/workflows # CI: bazel test //...
├── MODULE.bazel      # external deps (Bazel Central Registry)
└── data/             # local MOKUKU captures (git-ignored)
```

## Build and test

Requires Bazel 9.3.1, pinned in `.bazelversion`. Install [bazelisk](https://github.com/bazelbuild/bazelisk)
as `bazel` and it fetches the pinned version; dependencies come from the Bazel Central Registry.

```bash
bazel test //...                       # host build + all tests
bazel build --config=aarch64 //...     # cross-compile for arm64 (needs gcc-aarch64-linux-gnu)
```

Personal flags go in `user.bazelrc` (git-ignored, auto-imported). CI runs `bazel test //...` on
every push to `main` and every pull request (`.github/workflows/ci.yml`).

## License

GPL-2.0 (`LICENSE`). `ikfom/toolkit/` is adapted from IKFoM and MTK and keeps their original
notices; see `ikfom/toolkit/README.md`.
