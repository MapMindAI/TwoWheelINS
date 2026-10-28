# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
"""Runs INSLIB on a benchmark dataset with the noisy GPS of a run_dataset_main run, and writes its
trajectory as TUM in the dataset world frame (like gt.txt), followed by the same "# gps" lines, so
tools/trajectory_error.py and tools/benchmark_report.py compare it with InsFusion. Called by
run_inslib_datasets.sh, which fetches and builds INSLIB first.

  inslib_dataset.py <dataset_dir> <insfusion.tum> <work_dir> <out.tum> --replay <tools/replay.py>

INSLIB expects the IMU in an FRD body frame (x forward, y right, z down). The dataset IMU is turned
into one built from its forward.txt (the walking direction in the IMU frame) and the mean specific
force over the run (up): down = -up, forward = forward.txt without its down component, right =
down x forward. With that forward axis, automotive mode (yaw from the GNSS course over ground,
from 0.5 m/s) gives the heading. INSLIB also gets the noisy GPS positions with the GpsCovariance
std of their satellite count, a velocity from the step since the previous fix (the simulated
receiver reports none) with the matching covariance, InsFusion's IMU noise densities, and no
magnetometer. Its FRD attitude is turned back into the IMU frame for the output. ref.csv is the
ground truth (FRD attitude), so replay.py also scores it.
"""
import argparse
import math
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ins_benchmark import (NED_FROM_ENU, geodetic_to_utm, gps_quality, gps_std,  # noqa: E402
                           ned_from_euler, quaternion)

# run_dataset_main's kOrigin: world x, y, z are UTM easting, northing, altitude offsets from it
ORIGIN = (22.787839, 113.5210145, 13.0)
ZONE = 49
# InsFusion's kProcessNoise (fusion/ins_fusion.cc) as densities
GYR_PSD = 0.01 ** 2  # (rad/s)^2/Hz
ACC_PSD = 0.2 ** 2  # (m/s^2)^2/Hz
GYR_BIAS_RW = 2e-4  # rad/s/sqrt(s)
ACC_BIAS_RW = 2e-3  # m/s^2/sqrt(s)

CONFIG = f"""name: benchmark_dataset
aiding: gnss
init: auto
chi2_disable: 0
automotive_mode: 1
automotive_min_speed_mps: 0.5
max_deadreckoning_sec: 300.0

imu:
  gyr_psd: {GYR_PSD}
  acc_psd: {ACC_PSD}
  gyr_bias_rw: {GYR_BIAS_RW}
  acc_bias_rw: {ACC_BIAS_RW}

gnss:
  leverarm_frd: [0, 0, 0]
  delay_ms: 0
  # GpsCovariance reaches 7.5 m / 50 m at 4 satellites
  max_horizontal_pos_stddev_m: 8.0
  max_vertical_pos_stddev_m: 60.0
  start_max_horizontal_pos_stddev_m: 8.0
  start_max_vertical_pos_stddev_m: 60.0
  stop_max_horizontal_pos_stddev_m: 8.0
  stop_max_vertical_pos_stddev_m: 60.0
  max_horizontal_vel_stddev_mps: 20.0
  max_vertical_vel_stddev_mps: 20.0
  start_max_horizontal_vel_stddev_mps: 20.0
  start_max_vertical_vel_stddev_mps: 20.0
  stop_max_horizontal_vel_stddev_mps: 20.0
  stop_max_vertical_vel_stddev_mps: 20.0

mag:
  enable: 0

baro:
  enable: 0

score:
  warmup_sec: 15.0
  min_epochs: 1
  attitude: 1
  lim_pos_rms_m: 0
  lim_ellipsoid_rms_m: 0
"""


def origin_utm():
    east, north = geodetic_to_utm(ORIGIN[0], ORIGIN[1], ZONE)
    return np.array([east, north, ORIGIN[2]])


def world_to_geodetic(world):
    """(latitude_deg, longitude_deg, altitude_m) rows of world positions, by Newton iteration on
    geodetic_to_utm (exact to its own precision)."""
    target = origin_utm()[:2] + world[:, :2]
    latitude = np.full(len(world), ORIGIN[0])
    longitude = np.full(len(world), ORIGIN[1])
    step = 1e-6
    for _ in range(6):
        east, north = geodetic_to_utm(latitude, longitude, ZONE)
        east_lat, north_lat = geodetic_to_utm(latitude + step, longitude, ZONE)
        east_lon, north_lon = geodetic_to_utm(latitude, longitude + step, ZONE)
        a, b = (east_lat - east) / step, (east_lon - east) / step
        c, d = (north_lat - north) / step, (north_lon - north) / step
        de, dn = target[:, 0] - east, target[:, 1] - north
        determinant = a * d - b * c
        latitude = latitude + (d * de - b * dn) / determinant
        longitude = longitude + (a * dn - c * de) / determinant
    return np.column_stack([latitude, longitude, ORIGIN[2] + world[:, 2]])


def read_gps(path):
    rows = [line.split()[2:7] for line in open(path) if line.startswith("# gps ")]
    return np.array(rows, float)


def rotation_matrices(quaternions):
    x, y, z, w = quaternions.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], 1)


def frd_from_imu(dataset):
    """Rotation from the dataset IMU frame into INSLIB's FRD body frame."""
    imu = np.loadtxt(f"{dataset}/imu.txt", comments="#")
    down = -imu[:, 1:4].mean(axis=0)
    down /= np.linalg.norm(down)
    forward = np.loadtxt(f"{dataset}/forward.txt", comments="#")
    forward = forward - forward.dot(down) * down
    forward /= np.linalg.norm(forward)
    return np.vstack([forward, np.cross(down, forward), down])


def write_input(dataset, insfusion_tum, directory, frd_from_imu_matrix):
    os.makedirs(directory, exist_ok=True)
    imu = np.loadtxt(f"{dataset}/imu.txt", comments="#")
    gyr = imu[:, 4:7] @ frd_from_imu_matrix.T
    acc = imu[:, 1:4] @ frd_from_imu_matrix.T
    with open(f"{directory}/imu.csv", "w") as f:
        f.write("# t_us, gyr_frd_xyz [rad/s], acc_frd_xyz [m/s^2]\n")
        for t, g, a in zip(imu[:, 0], gyr, acc):
            f.write("%d,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f\n" % (round(t * 1e6), *g, *a))
    gps = read_gps(insfusion_tum)
    geodetic = world_to_geodetic(gps[:, 1:4])
    with open(f"{directory}/gnss.csv", "w") as f:
        f.write("# t_us, lat_deg, lon_deg, h_m, cov_pos_ned nn,ne,nd,ee,ed,dd, vn, ve, vd, "
                "cov_vel_ned nn,ne,nd,ee,ed,dd, vel_ok, quality, gps_week, gps_tow_s, num_sv\n")
        for k, (fix, position) in enumerate(zip(gps, geodetic)):
            horizontal, vertical = gps_std(fix[4])
            # velocity from the step since the previous fix (the receiver reports none)
            velocity, velocity_variance, velocity_ok = np.zeros(3), np.zeros(2), 0
            if k > 0 and gps[k, 0] - gps[k - 1, 0] <= 1.5:
                dt = gps[k, 0] - gps[k - 1, 0]
                velocity = (gps[k, 1:4] - gps[k - 1, 1:4]) / dt @ NED_FROM_ENU.T
                previous_horizontal, previous_vertical = gps_std(gps[k - 1, 4])
                velocity_variance = np.array([horizontal ** 2 + previous_horizontal ** 2,
                                              vertical ** 2 + previous_vertical ** 2]) / dt ** 2
                velocity_ok = 1
            f.write("%d,%.10f,%.10f,%.4f,%g,0,0,%g,0,%g,%.3f,%.3f,%.3f,%g,0,0,%g,0,%g,%d,%d,0,0,%d"
                    "\n" % (round(fix[0] * 1e6), *position, horizontal ** 2, horizontal ** 2,
                            vertical ** 2, *velocity, velocity_variance[0], velocity_variance[0],
                            velocity_variance[1], velocity_ok, gps_quality(fix[4]), fix[4]))
    truth = np.loadtxt(f"{dataset}/gt.txt", comments="#")
    truth_geodetic = world_to_geodetic(truth[:, 1:4])
    ned_from_frd = NED_FROM_ENU @ rotation_matrices(truth[:, 4:8]) @ frd_from_imu_matrix.T
    roll = np.degrees(np.arctan2(ned_from_frd[:, 2, 1], ned_from_frd[:, 2, 2]))
    pitch = np.degrees(-np.arcsin(np.clip(ned_from_frd[:, 2, 0], -1, 1)))
    yaw = np.degrees(np.arctan2(ned_from_frd[:, 1, 0], ned_from_frd[:, 0, 0]))
    velocity_ned = np.gradient(truth[:, 1:4], truth[:, 0], axis=0) @ NED_FROM_ENU.T
    with open(f"{directory}/ref.csv", "w") as f:
        f.write("# t_us, lat_deg, lon_deg, h_m, roll_deg, pitch_deg, yaw_deg, vn, ve, vd "
                "(ground truth, FRD body frame)\n")
        for row in zip(truth[:, 0], truth_geodetic, roll, pitch, yaw, velocity_ned):
            f.write("%d,%.10f,%.10f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f\n" % (
                round(row[0] * 1e6), *row[1], row[2], row[3], row[4], *row[5]))
    with open(f"{directory}/config.yaml", "w") as f:
        f.write(CONFIG)


def write_tum(solution, insfusion_tum, path, frd_from_imu_matrix):
    d = np.loadtxt(solution, delimiter=",", comments="#", usecols=range(7))
    east, north = geodetic_to_utm(d[:, 1], d[:, 2], ZONE)
    world = np.column_stack([east, north, d[:, 3]]) - origin_utm()
    with open(path, "w") as f:
        f.write("# INSLIB, world frame, t_s x_m y_m z_m qx qy qz qw (world_from_imu)\n")
        for t, position, rpy in zip(d[:, 0] * 1e-6, world, d[:, 4:7]):
            # ENU <- NED <- FRD <- IMU
            rotation = NED_FROM_ENU @ ned_from_euler(*np.radians(rpy)) @ frd_from_imu_matrix
            f.write("%.6f %.4f %.4f %.4f %.7f %.7f %.7f %.7f\n" % (t, *position,
                                                                 *quaternion(rotation)))
        f.writelines(line for line in open(insfusion_tum) if line.startswith("# gps "))
    print(f"wrote {len(d)} poses to {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dataset")
    parser.add_argument("insfusion_tum", help="run_dataset_main output with the noisy GPS")
    parser.add_argument("work_dir")
    parser.add_argument("output", help="TUM trajectory to write")
    parser.add_argument("--replay", required=True, help="INSLIB tools/replay.py")
    args = parser.parse_args()
    directory = os.path.abspath(args.work_dir)
    frd_from_imu_matrix = frd_from_imu(args.dataset)
    write_input(args.dataset, args.insfusion_tum, directory, frd_from_imu_matrix)
    solution = f"{directory}/solution.csv"
    with open(f"{directory}/log.txt", "w") as log:
        subprocess.run([sys.executable, args.replay, directory, "--dump-solution", solution,
                        "--dump-solution-hz", "0"], check=True, stdout=log,
                       stderr=subprocess.STDOUT)
    write_tum(solution, args.insfusion_tum, args.output, frd_from_imu_matrix)


if __name__ == "__main__":
    main()
