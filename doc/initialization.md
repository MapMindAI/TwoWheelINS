# InsFusion initialization

How `fusion/InsFusion` gets from the first IMU sample to a running filter (`fusion/ins_fusion.cc`).
The filter state is position, velocity, attitude and the accelerometer and gyroscope biases
(`KfcoreIns`); the world frame is UTM east-north-up, relative to the first usable GPS fix.

```
IMU 0-1 s ──► levelling ──► first GPS fix ──► second GPS fix: filter starts
                                  ▲                                │
                                  └── IMU gap > max_imu_gap_seconds ◄┘
```

## 1. Levelling (first second of IMU)

- The mean specific force over the first `kStaticAlignmentSeconds` (1 s) is taken as "up" in the
  body frame; the attitude is the smallest rotation that turns it onto world +z. This fixes roll
  and pitch; yaw is arbitrary.
- The gyroscope bias starts at **zero**: the start need not be static. A non-static start only
  degrades roll and pitch through the motion in the mean.

## 2. Two GPS fixes

A fix is usable with ≥ 4 satellites and when it is not a repeat of the previous fix.

- At the first usable fix after levelling, its position and horizontal variance
  (`sensor::GpsCovariance`) are kept, and the integral `F` of the forward direction
  (`InsFusionOptions::forward`, IMU frame) starts: `F += R · forward dt`, with the levelled
  attitude `R ← R · exp((ω − b_g) dt)`.
- At the second, the filter starts. The yaw turns the heading of `F` onto the course of the step
  `s = p_2 − p_1`:

```
yaw = atan2(s_y, s_x) − atan2(F_y, F_x)
```

For benchmark datasets `forward` comes from `forward.txt` (the mean ground-truth walking direction
in the IMU frame, written by `benchmark/make_datasets.py`); it defaults to IMU +x.

## 3. Filter start

With `σ_s = sqrt(σ_1² + σ_2²)` the horizontal noise of the step and `dt` its duration:

| State | Initial value | Initial std |
|---|---|---|
| position | the second fix | `GpsCovariance` of that fix |
| velocity | `s / dt` (vertical 0) | `σ_s / dt`, at least 1 m/s |
| roll, pitch | from levelling, propagated by the gyroscope | 0.05 rad |
| yaw | `Rz(yaw) · R` | `σ_s / |s|`, clamped to [0.05, π] rad |
| accelerometer bias | 0 | 0.3 m/s² |
| gyroscope bias | 0 (or carried) | 0.02 rad/s |

From then on every IMU sample predicts the filter and every GPS fix updates its position (see
`InsFusion` for the velocity constraints and the altitude gate).

## 4. IMU gaps

- A gap up to `max_imu_gap_seconds` is predicted over with the first sample after it (a GPS fix
  inside the gap is applied after predicting to its time with the last sample).
- A longer gap restarts from the next two fixes: the attitude and gyroscope bias are carried from
  the filter.

## Known limitations

Benchmark numbers are from `benchmark/run_datasets.sh --seed 2`.

- **Start yaw.** In 1 s a walker moves 0.6–1.2 m while a single-solution fix is off by 1–2.5 m,
  and some start standing still, so the start yaw is 4–180° off. GPS position updates hardly
  observe yaw while walking, so the error stays: the yaw median is 4–166° on the outdoor datasets.
  A heading measurement (magnetometer) is needed.
- **IMU gaps.** Predicting over a 1 s gap with one sample is wrong while walking: that sample is
  2–5 m/s² and 0.7 rad/s away from the mean motion. The lens (`indoor-office_*`) datasets lose
  1 s every 2–4 s, and their height runs off by tens to hundreds of metres, partly into a wrong
  accelerometer bias.
- **Gyroscope bias.** Starting at zero, a vertical gyroscope bias is learnt from GPS only, which
  is slow in a steady turn.
