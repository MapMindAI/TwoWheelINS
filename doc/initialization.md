# InsFusion initialization

How `fusion/InsFusion` gets from the first IMU sample to a running filter (`fusion/ins_fusion.cc`).
The filter state is position, velocity, attitude and the accelerometer and gyroscope biases
(`KfcoreIns`); the world frame is UTM east-north-up, relative to the first usable GPS fix.

```
IMU 0-1 s ──► levelling ──► integrate in a levelled frame ──► yaw fit over GPS fixes ──► filter
                                       ▲                                                    │
                                       └────────────── IMU gap > max_imu_gap_seconds ◄─────┘
```

## 1. Levelling (first second of IMU)

- The mean specific force over the first `kStaticAlignmentSeconds` (1 s) is taken as "up" in the
  body frame; the attitude is the smallest rotation that turns it onto world +z. This fixes roll
  and pitch; yaw is arbitrary.
- The gyroscope bias starts at **zero**: the start need not be static. A non-static start only
  degrades roll and pitch through the motion in the mean.

## 2. Integration in the levelled frame

Until the filter starts, every IMU sample is integrated in the levelled, yaw-free frame:

- attitude: `R ← R · exp((ω − b_g) dt)` with `b_g` = 0 (or the carried estimate after a gap),
- acceleration `a = R f − (0, 0, g)`, velocity `v += a dt`, position `p += v dt + ½ a dt²`,
- with a known forward direction (`InsFusionOptions::forward`, IMU frame): `F += R · forward dt`.

At every usable GPS fix (≥ 4 satellites, not a repeat of the previous fix) these integrals are
stored with the fix position and its horizontal variance (`sensor::GpsCovariance`).

## 3. Yaw fit

The window of stored fixes grows from `kMinYawFitFixes` (5) and is tried at every fix. A fit is
accepted when its yaw standard deviation is small enough; otherwise the window keeps growing up to
`kMaxYawFitFixes` (30) and then slides. A singular fit (NaN) is rejected. Which fit runs depends on
whether the forward direction is known.

### With a known forward direction: GPS course

For benchmark datasets the forward direction comes from `forward.txt` (the mean ground-truth
walking direction in the IMU frame, written by `benchmark/make_datasets.py`).

Each GPS step `p_k − p_{k−1}` is turned back by the heading `h_k` of the integrated forward
direction over the same interval, and the steps are summed:

```
s = Σ_k R(−h_k) (p_k − p_{k−1}),    yaw = atan2(s_y, s_x)
```

`s` points along the yaw; the sum weighs steps by length (so standing still adds nothing) and
follows turns through `h_k`. Its covariance follows from the GPS variance of every fix; the fit is
accepted below `kMaxCourseYawStd` (0.1 rad). At a walking speed of 1.2 m/s and 0.5 m GPS noise
that takes about 6 s of walking.

### Without it: GPS track (mount-free)

The levelled IMU double integral `d_k` (from rest at the first fix of the window) is rotated onto
the GPS displacement:

```
p_k − p_0 = v_0 Δt_k + Rz(yaw) d_k + ½ b Δt_k²
```

with unknown start velocity `v_0` and a constant world acceleration `b` that absorbs gravity
leaking through the levelling error. It is linear in `(v_0, cos yaw, sin yaw, b)` and solved by
least squares; the fit is accepted below `kMaxTrackYawStd` (0.3 rad), and the filter refines the
yaw. Yaw is only observable when the track speeds up, slows down or turns: a constant acceleration
is indistinguishable from `b`. The precision grows with about the third power of the window
length.

## 4. Filter start

| State | Initial value | Initial std |
|---|---|---|
| position | the last fix of the window | `GpsCovariance` of that fix |
| velocity | course fit: last GPS step / dt; track fit: from the fitted `v_0`, `b` and integrals (vertical 0) | 1 m/s |
| roll, pitch | from levelling, propagated by the gyroscope | 0.05 rad |
| yaw | `Rz(yaw) · R` | the fit's std, at least 0.05 rad |
| accelerometer bias | 0 | 0.3 m/s² |
| gyroscope bias | 0 (or carried) | 0.02 rad/s |

From then on every IMU sample predicts the filter and every GPS fix updates its position (see
`InsFusion` for the velocity constraints and the altitude gate).

## 5. IMU gaps

- A gap up to `max_imu_gap_seconds` is predicted over with the first sample after it (a GPS fix
  inside the gap is applied after predicting to its time with the last sample).
- A longer gap restarts the yaw fit: the attitude and gyroscope bias are carried from the filter,
  the integrals and the fit window are cleared.

## Known limitations

Benchmark numbers are from `benchmark/run_datasets.sh --seed 2`.

- **IMU gaps.** Predicting over a 1 s gap with one sample is wrong while walking: that sample is
  2–5 m/s² and 0.7 rad/s away from the mean motion. The lens (`indoor-office_*`) datasets lose
  1 s every 2–4 s, and their height runs off by tens to hundreds of metres, partly into a wrong
  accelerometer bias. Filling the gaps with the mean IMU of the preceding second brings their
  height median to 0.7–1.3 m. Their yaw stays wrong (14–56°): the turning inside a gap is unknown.
- **Gyroscope bias.** Starting at zero, a vertical gyroscope bias is learnt from GPS only, which
  is slow in a steady turn (in simulation a 0.01 rad/s bias leaves ~0.2 rad of yaw error for
  ~40 s).
- **Start latency.** The course fit waits for enough walking relative to the GPS noise of the
  window's worst fix (up to 7.5 m for a single solution): InsFusion starts 29–103 s into the
  benchmark datasets at the default noise, INSLIB (which restarts from GPS) after 5–25 s.
- **Track fit precision.** Without a forward direction, walking accelerations are weak against
  IMU drift: even with exact GPS a single window is 12–28° off in median (offline prototype), so
  the start yaw relies on the filter to converge.
