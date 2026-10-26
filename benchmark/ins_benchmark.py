# Copyright 2026 MapMind
# SPDX-License-Identifier: GPL-2.0-only
"""Runs KF-GINS or INSLIB on a MOKUKU record and writes a TUM trajectory for
fusion/trajectory_viewer.html. Called by run_kfgins.sh / run_inslib.sh, which fetch and build the
library first.

  ins_benchmark.py kfgins <record> <work_dir> <out.tum> --binary <KF-GINS> [options]
  ins_benchmark.py inslib <record> <work_dir> <out.tum> --replay <tools/replay.py> [options]

Both get the same input as ins_fusion_main: IMU samples cleaned like RemoveCorruptImuSamples, GPS
fixes with >= 4 satellites that do not repeat the previous one, and the GPS covariance of
GpsCovariance (fusion/ins_fusion.cc). The IMU is rotated into a vehicle front-right-down (FRD)
frame: down from the mean specific force, forward from fitting the horizontal specific force to the
GPS along-track acceleration, turned by --mount-yaw-deg.

The output TUM is the vehicle pose (FRD in UTM east-north-up) in the UTM zone of the first fix,
followed by the record's GPS fixes as "# gps t_s easting_m northing_m altitude_m satellites".
"""
import argparse
import math
import os
import subprocess
import sys

import numpy as np

ACC_RANGE = 50.0  # m/s^2
ACC_SPIKE = 15.0  # m/s^2
GYR_SPIKE = 0.4  # rad/s
NED_FROM_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)

# IMU noise measured on mobili_2: 3.1 deg/s gyroscope, 1.4 m/s^2 accelerometer per sample at 29 Hz.
KFGINS_ARW_DEG_SQRTH = 34.0
KFGINS_VRW_MPS_SQRTH = 16.0
INSLIB_GYR_PSD = 1.0e-4  # (rad/s)^2/Hz
INSLIB_ACC_PSD = 0.07  # (m/s^2)^2/Hz
KFGINS_YAW_HYPOTHESES = 12
KFGINS_SCORE_DELAY_S = 30.0  # skip the convergence after the start when scoring a hypothesis
MIN_START_SPEED_MPS = 3.0


# ---- record ----------------------------------------------------------------------------------
def read_record(path):
    """imu rows: t_us acc_xyz gyr_xyz; gps rows: t_us status lat lon alt speed satellites."""
    imu, gps = [], []
    with open(path) as f:
        for line in f:
            fields = line.split()
            try:
                if fields[0] == "imu" and len(fields) == 8:
                    imu.append([float(x) for x in fields[1:]])
                elif fields[0] == "gps" and len(fields) == 8:
                    gps.append([float(x) for x in fields[1:]])
            except (ValueError, IndexError):
                pass
    return np.array(imu), np.array(gps)


def clean_imu(imu):
    """Same rules as RemoveCorruptImuSamples (common/reader/txt_record_reader.cc)."""
    imu = imu[np.abs(imu[:, 1:4]).max(axis=1) <= ACC_RANGE]
    keep = np.ones(len(imu), bool)
    for i in range(1, len(imu) - 1):
        for cols, threshold in ((slice(1, 4), ACC_SPIKE), (slice(4, 7), GYR_SPIKE)):
            to_previous = imu[i, cols] - imu[i - 1, cols]
            to_next = imu[i, cols] - imu[i + 1, cols]
            if np.any((np.abs(to_previous) > threshold) & (np.abs(to_next) > threshold) &
                      ((to_previous > 0) == (to_next > 0))):
                keep[i] = False
    return imu[keep]


def usable_gps(gps):
    """Fixes with >= 4 satellites not repeating the previous usable one (InsFusion::IsUsable)."""
    out, last = [], None
    for fix in gps:
        if fix[6] < 4:
            continue
        if last is None or fix[2] != last[2] or fix[3] != last[3]:
            out.append(fix)
        last = fix
    return np.array(out)


def gps_quality(satellites):
    """RTKLIB solution status from the satellite count: 1 fix, 2 float, 5 single, 0 none."""
    n = int(satellites)
    return 1 if n > 20 else 2 if n > 10 else 5 if n >= 4 else 0


def gps_std(satellites):
    """(horizontal, vertical) std in m; same rule as GpsCovariance in fusion/ins_fusion.cc."""
    n = np.asarray(satellites, float)
    redundancy = np.maximum(n - 3, 1)
    horizontal = np.where(n > 20, 0.05, np.where(n > 10, 0.5, 7.5 / redundancy))
    vertical = np.where(n > 20, 0.1, np.where(n > 10, 1.0, 50.0 / redundancy))
    return horizontal, vertical


# ---- geodesy ---------------------------------------------------------------------------------
def utm_zone(longitude_deg):
    zone = int(math.floor((longitude_deg + 180.0) / 6.0)) + 1
    return 1 if zone > 60 else zone


def geodetic_to_utm(latitude_deg, longitude_deg, zone):
    """(easting, northing) in m; Krueger series to third order, as common/math/geodesy.cc."""
    a, f = 6378137.0, 1.0 / 298.257223563
    n = f / (2.0 - f)
    rectifying_radius = a / (1.0 + n) * (1.0 + n * n / 4.0 + n ** 4 / 64.0)
    alpha = [n / 2 - 2 * n * n / 3 + 5 * n ** 3 / 16, 13 * n * n / 48 - 3 * n ** 3 / 5,
             61 * n ** 3 / 240]
    eccentricity_term = 2.0 * math.sqrt(n) / (1.0 + n)
    latitude = np.radians(latitude_deg)
    longitude_offset = np.radians(np.asarray(longitude_deg) - (6.0 * zone - 183.0))
    sin_latitude = np.sin(latitude)
    t = np.sinh(np.arctanh(sin_latitude) -
                eccentricity_term * np.arctanh(eccentricity_term * sin_latitude))
    xi = np.arctan2(t, np.cos(longitude_offset))
    eta = np.arctanh(np.sin(longitude_offset) / np.sqrt(1.0 + t * t))
    easting, northing = eta, xi
    for j in range(1, 4):
        easting = easting + alpha[j - 1] * np.cos(2 * j * xi) * np.sinh(2 * j * eta)
        northing = northing + alpha[j - 1] * np.sin(2 * j * xi) * np.cosh(2 * j * eta)
    false_northing = np.where(np.asarray(latitude_deg) < 0, 10000000.0, 0.0)
    return (500000.0 + 0.9996 * rectifying_radius * easting,
            false_northing + 0.9996 * rectifying_radius * northing)


# ---- rotations -------------------------------------------------------------------------------
def ned_from_euler(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                     [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                     [-sp, sr * cp, cr * cp]])


def quaternion(R):
    """(x, y, z, w) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


# ---- shared preprocessing --------------------------------------------------------------------
class Session:
    """Cleaned record in UTM plus the IMU in the vehicle FRD frame."""

    def __init__(self, record, mount_yaw_deg, outages):
        imu, gps_all = read_record(record)
        self.gps_all = gps_all[(gps_all[:, 2] != 0) | (gps_all[:, 3] != 0)]
        self.imu = clean_imu(imu)
        self.gps = usable_gps(gps_all)
        self.zone = utm_zone(self.gps[0, 3])
        east, north = geodetic_to_utm(self.gps[:, 2], self.gps[:, 3], self.zone)
        self.enu = np.column_stack([east, north, self.gps[:, 4]])
        self.t_gps = self.gps[:, 0] * 1e-6
        self.vehicle_from_imu = self.mount(mount_yaw_deg)
        self.acc = self.imu[:, 1:4] @ self.vehicle_from_imu.T
        self.gyr = self.imu[:, 4:7] @ self.vehicle_from_imu.T
        self.in_outage = np.zeros(len(self.gps), bool)
        for start, duration in outages:
            self.in_outage |= (self.t_gps >= start) & (self.t_gps < start + duration)
        print(f"imu {len(self.imu)}, usable gps {len(self.gps)}, "
              f"{self.in_outage.sum()} removed by {len(outages)} outages, UTM zone {self.zone}")

    def mount(self, yaw_deg, window_s=8):
        t, acc = self.imu[:, 0] * 1e-6, self.imu[:, 1:4]
        up = acc.mean(0) / np.linalg.norm(acc.mean(0))
        h1 = np.array([1.0, 0, 0]) - up * up[0]
        h1 /= np.linalg.norm(h1)
        h2 = np.cross(up, h1)
        grid = np.arange(np.ceil(self.t_gps[0]) + 2, self.t_gps[-1] - 2, 1.0)
        speed = np.hypot(np.gradient(np.interp(grid, self.t_gps, self.enu[:, 0])),
                         np.gradient(np.interp(grid, self.t_gps, self.enu[:, 1])))
        kernel = np.ones(window_s) / window_s
        along = np.gradient(np.convolve(speed, kernel, "same"))
        binned = np.array([acc[(t >= g - 0.5) & (t < g + 0.5)].mean(0) for g in grid])
        horizontal = np.column_stack([np.convolve(binned @ h1, kernel, "same"),
                                      np.convolve(binned @ h2, kernel, "same")])
        valid = (speed > MIN_START_SPEED_MPS) & ~np.isnan(horizontal).any(1)
        solution, *_ = np.linalg.lstsq(horizontal[valid], along[valid], rcond=None)
        forward = (solution[0] * h1 + solution[1] * h2) / np.linalg.norm(solution)
        print("mount fit: correlation %.2f, forward %.1f deg from IMU x, turned by %.1f deg" % (
            np.corrcoef(horizontal[valid] @ solution, along[valid])[0, 1],
            math.degrees(math.atan2(solution[1], solution[0])), yaw_deg))
        down = -up
        right = np.cross(down, forward)
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        forward, right = c * forward + s * right, c * right - s * forward
        return np.vstack([forward, right, down])

    def gps_velocity_ned(self):
        """Receiver ground speed along the central-difference GPS track; 0 where undefined."""
        velocity = np.zeros((len(self.gps), 3))
        valid = np.zeros(len(self.gps), int)
        for k in range(1, len(self.gps) - 1):
            dt = self.t_gps[k + 1] - self.t_gps[k - 1]
            chord = self.enu[k + 1, :2] - self.enu[k - 1, :2]
            if dt > 3.0 or np.linalg.norm(chord) < 1.0:
                continue
            east, north = chord / np.linalg.norm(chord) * self.gps[k, 5]
            velocity[k] = [north, east, 0.0]
            valid[k] = 1
        return velocity, valid

    def start_fix(self):
        """First fix moving >= MIN_START_SPEED_MPS (InsFusion's start condition)."""
        for k in range(1, len(self.gps) - 1):
            dt = self.t_gps[k] - self.t_gps[k - 1]
            speed = np.linalg.norm(self.enu[k, :2] - self.enu[k - 1, :2]) / dt
            if 0.5 <= dt <= 1.5 and speed >= MIN_START_SPEED_MPS:
                return k
        raise RuntimeError("vehicle never moves fast enough to start")

    def gps_comment_lines(self):
        east, north = geodetic_to_utm(self.gps_all[:, 2], self.gps_all[:, 3], self.zone)
        return ["# gps %.6f %.4f %.4f %.4f %d\n" % (fix[0] * 1e-6, e, n, fix[4], fix[6])
                for fix, e, n in zip(self.gps_all, east, north)]

    def write_tum(self, path, t, latitude, longitude, height, rpy_deg):
        east, north = geodetic_to_utm(latitude, longitude, self.zone)
        hemisphere = "S" if self.gps[0, 2] < 0 else "N"
        with open(path, "w") as f:
            f.write(f"# UTM zone {self.zone}{hemisphere} (WGS84), "
                    "t_s easting_m northing_m altitude_m qx qy qz qw\n")
            for row in zip(t, east, north, height, rpy_deg):
                rotation = NED_FROM_ENU @ ned_from_euler(*np.radians(row[4]))  # ENU <- FRD
                f.write("%.6f %.4f %.4f %.4f %.7f %.7f %.7f %.7f\n" %
                        (*row[:4], *quaternion(rotation)))
            f.writelines(self.gps_comment_lines())
        print(f"wrote {len(t)} poses to {path}")

    def horizontal_rms_to_gps(self, t, latitude, longitude, from_t):
        east, north = geodetic_to_utm(latitude, longitude, self.zone)
        scored = (self.t_gps >= from_t) & (self.t_gps <= t[-1])
        error = np.hypot(np.interp(self.t_gps[scored], t, east) - self.enu[scored, 0],
                         np.interp(self.t_gps[scored], t, north) - self.enu[scored, 1])
        return math.sqrt(np.mean(error ** 2))


# ---- KF-GINS ---------------------------------------------------------------------------------
def write_kfgins_input(session, directory, yaw_offset_deg):
    os.makedirs(directory, exist_ok=True)
    t_imu = session.imu[:, 0] * 1e-6
    dt = np.diff(t_imu)
    # increments over the interval ending at each sample
    dtheta = 0.5 * (session.gyr[1:] + session.gyr[:-1]) * dt[:, None]
    dvelocity = 0.5 * (session.acc[1:] + session.acc[:-1]) * dt[:, None]
    np.savetxt(f"{directory}/imu.txt", np.column_stack([t_imu[1:], dtheta, dvelocity]), fmt="%.9f")
    gps = session.gps[~session.in_outage]
    horizontal, vertical = gps_std(gps[:, 6])
    np.savetxt(f"{directory}/gnss.txt",
               np.column_stack([gps[:, 0] * 1e-6, gps[:, 2], gps[:, 3], gps[:, 4],
                                horizontal, horizontal, vertical]), fmt="%.10f")

    k = session.start_fix()
    velocity_enu = ((session.enu[k + 1] - session.enu[k - 1]) /
                    (session.t_gps[k + 1] - session.t_gps[k - 1]))
    velocity_enu[2] = 0.0
    yaw = math.degrees(math.atan2(velocity_enu[0], velocity_enu[1])) + yaw_offset_deg
    start_std = gps_std(session.gps[k, 6])[0]
    with open(f"{directory}/kf-gins.yaml", "w") as f:
        f.write(f"""imupath: "{directory}/imu.txt"
gnsspath: "{directory}/gnss.txt"
outputpath: "{directory}"
imudatalen: 7
imudatarate: {int(round(1.0 / np.median(dt)))}
starttime: {session.t_gps[k]:.6f}
endtime: -1
initpos: [ {session.gps[k, 2]:.9f}, {session.gps[k, 3]:.9f}, {session.gps[k, 4]:.3f} ]
initvel: [ {', '.join('%.3f' % x for x in NED_FROM_ENU @ velocity_enu)} ]
initatt: [ 0.0, 0.0, {yaw:.4f} ]
initgyrbias: [ 0, 0, 0 ]
initaccbias: [ 0, 0, 0 ]
initgyrscale: [ 0, 0, 0 ]
initaccscale: [ 0, 0, 0 ]
initposstd: [ {start_std:.3f}, {start_std:.3f}, 3.0 ]
initvelstd: [ 1.0, 1.0, 1.0 ]
initattstd: [ 3.0, 3.0, 10.0 ]
imunoise:
  arw: [{KFGINS_ARW_DEG_SQRTH}, {KFGINS_ARW_DEG_SQRTH}, {KFGINS_ARW_DEG_SQRTH}]  # deg/sqrt(h)
  vrw: [{KFGINS_VRW_MPS_SQRTH}, {KFGINS_VRW_MPS_SQRTH}, {KFGINS_VRW_MPS_SQRTH}]  # m/s/sqrt(h)
  gbstd: [1000, 1000, 1000]         # deg/h
  abstd: [50000, 50000, 50000]      # mGal
  gsstd: [10000, 10000, 10000]      # ppm
  asstd: [10000, 10000, 10000]      # ppm
  corrtime: 1.0                     # h
antlever: [ 0, 0, 0 ]
""")
    return session.t_gps[k]


def run_kfgins(args, session):
    """Starts KF-GINS at evenly spaced yaw offsets from the GPS course and keeps the run that fits
    the GPS best, like InsFusion's yaw hypotheses."""
    best = None
    for i in range(KFGINS_YAW_HYPOTHESES):
        offset = 360.0 * i / KFGINS_YAW_HYPOTHESES
        directory = os.path.abspath(f"{args.work_dir}/yaw_{int(offset):03d}")
        start = write_kfgins_input(session, directory, offset)
        subprocess.run([args.binary, f"{directory}/kf-gins.yaml"], check=True,
                       stdout=subprocess.DEVNULL)
        nav = np.loadtxt(f"{directory}/KF_GINS_Navresult.nav")
        rms = session.horizontal_rms_to_gps(nav[:, 1], nav[:, 2], nav[:, 3],
                                            start + KFGINS_SCORE_DELAY_S)
        print(f"yaw offset {offset:5.1f} deg: horizontal RMS to GPS {rms:.2f} m")
        if best is None or rms < best[0]:
            best = (rms, offset, nav)
    rms, offset, nav = best
    print(f"selected yaw offset {offset:.1f} deg (RMS {rms:.2f} m)")
    session.write_tum(args.output, nav[:, 1], nav[:, 2], nav[:, 3], nav[:, 4], nav[:, 8:11])


# ---- INSLIB ----------------------------------------------------------------------------------
INSLIB_CONFIG = f"""name: ins_benchmark
aiding: gnss
init: auto
chi2_disable: 0
automotive_mode: 1
automotive_lateral_constraint: 1
max_deadreckoning_sec: 120.0

imu:
  gyr_psd: {INSLIB_GYR_PSD}
  acc_psd: {INSLIB_ACC_PSD}
  gyr_bias_rw: 1.0e-04   # rad/s/sqrt(s)
  acc_bias_rw: 1.0e-02   # m/s^2/sqrt(s)

gnss:
  leverarm_frd: [0, 0, 0]
  delay_ms: 0
  # GpsCovariance reaches 7.5 m / 50 m at 4 satellites; GNSS velocity is 1 m/s
  max_horizontal_pos_stddev_m: 8.0
  max_vertical_pos_stddev_m: 60.0
  max_horizontal_vel_stddev_mps: 3.0
  max_vertical_vel_stddev_mps: 3.0
  start_max_horizontal_pos_stddev_m: 3.0
  start_max_vertical_pos_stddev_m: 15.0
  start_max_horizontal_vel_stddev_mps: 2.0
  start_max_vertical_vel_stddev_mps: 2.0
  stop_max_horizontal_pos_stddev_m: 8.0
  stop_max_vertical_pos_stddev_m: 60.0
  stop_max_horizontal_vel_stddev_mps: 3.0
  stop_max_vertical_vel_stddev_mps: 3.0

mag:
  enable: 0

baro:
  enable: 0

score:
  warmup_sec: 30.0
  min_epochs: 1
  attitude: 0
  lim_pos_rms_m: 0
  lim_ellipsoid_rms_m: 0
"""


def write_inslib_input(session, directory):
    os.makedirs(directory, exist_ok=True)
    with open(f"{directory}/imu.csv", "w") as f:
        f.write("# t_us, gyr_frd_xyz [rad/s], acc_frd_xyz [m/s^2]\n")
        for t, gyr, acc in zip(session.imu[:, 0], session.gyr, session.acc):
            f.write("%d,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f\n" % (t, *gyr, *acc))
    velocity, velocity_valid = session.gps_velocity_ned()
    keep = ~session.in_outage
    with open(f"{directory}/gnss.csv", "w") as gnss, open(f"{directory}/ref.csv", "w") as ref:
        gnss.write("# t_us, lat_deg, lon_deg, h_m, cov_pos_ned nn,ne,nd,ee,ed,dd, vn, ve, vd, "
                   "cov_vel_ned nn,ne,nd,ee,ed,dd, vel_ok, quality, gps_week, gps_tow_s, num_sv, "
                   "jamming, spoofing, sig_auth, nav_auth\n")
        gnss.write("# velocity: receiver ground speed along the GPS track; quality from the "
                   "satellite count (> 20 fix, 11-20 float, 4-10 single); GPS week/tow unknown\n")
        ref.write("# t_us, lat_deg, lon_deg, h_m, roll_deg, pitch_deg, yaw_deg, vn, ve, vd\n")
        for fix, v, valid in zip(session.gps[keep], velocity[keep], velocity_valid[keep]):
            horizontal, vertical = gps_std(fix[6])
            gnss.write("%d,%.10f,%.10f,%.3f,%g,0,0,%g,0,%g,%.3f,%.3f,%.3f,1,0,0,1,0,1,%d,%d,0,0,%d,"
                       "0,0,0,0\n" % (fix[0], fix[2], fix[3], fix[4], horizontal ** 2,
                                      horizontal ** 2, vertical ** 2, *v, valid,
                                      gps_quality(fix[6]), fix[6]))
            ref.write("%d,%.10f,%.10f,%.3f,0,0,0,%.3f,%.3f,%.3f\n" % (fix[0], *fix[2:5], *v))
    with open(f"{directory}/config.yaml", "w") as f:
        f.write(INSLIB_CONFIG)


def run_inslib(args, session):
    directory = os.path.abspath(args.work_dir)
    write_inslib_input(session, directory)
    solution = f"{directory}/solution.csv"
    with open(f"{directory}/log.txt", "w") as log:
        subprocess.run([sys.executable, args.replay, directory, "--dump-solution", solution,
                        "--dump-solution-hz", "0"],
                       check=True, stdout=log, stderr=subprocess.STDOUT)
    d = np.loadtxt(solution, delimiter=",", comments="#", usecols=range(7))
    session.write_tum(args.output, d[:, 0] * 1e-6, d[:, 1], d[:, 2], d[:, 3], d[:, 4:7])


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("library", choices=["kfgins", "inslib"])
    parser.add_argument("record")
    parser.add_argument("work_dir")
    parser.add_argument("output", help="TUM trajectory to write")
    parser.add_argument("--binary", help="KF-GINS executable")
    parser.add_argument("--replay", help="INSLIB tools/replay.py")
    parser.add_argument("--mount-yaw-deg", type=float, default=0.0,
                        help="turn the fitted IMU forward axis about down (40 for mobili_2)")
    parser.add_argument("--outage", action="append", default=[], metavar="START_S:DURATION_S",
                        help="drop GPS fixes in this window (repeatable)")
    args = parser.parse_args()
    outages = [tuple(map(float, outage.split(":"))) for outage in args.outage]
    session = Session(args.record, args.mount_yaw_deg, outages)
    if args.library == "kfgins":
        run_kfgins(args, session)
    else:
        run_inslib(args, session)


if __name__ == "__main__":
    main()
