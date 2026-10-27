# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
"""Builds the benchmark datasets under benchmark/datasets/<name>/ from looper rosbags and their
CoreMap global maps. Needs the ROS 2 Python environment (source /opt/ros/<distro>/setup.bash; uses
rosbag2_py and numpy only):

  /usr/bin/python3 benchmark/make_datasets.py [--rosbags DIR] [--maps DIR] [--out DIR]
      [--images DIR] [names...]

Each dataset gets:
  imu.txt  t_s ax ay az (m/s^2) gx gy gz (rad/s) of /camera/camera/imu (front camera), in
           camera_camera_imu_optical (same axes as camera_camera_left), header stamps
  gps.txt  t_s x y z (m): IMU position in the ground-truth world frame (z up), sampled at 1 Hz
           by linear interpolation of gt.txt; no noise. Skipped where the two neighbouring
           ground-truth poses are more than MAX_GT_GAP_S apart.
  gt.txt   TUM world_from_camera_camera_imu: the map base_poses.txt (world_from_camera_camera_left)
           moved to the IMU with the /tf_static extrinsic, which is kept in the header
--images gets <name>.png for the wiki Benchmark-Datasets page: top-down view of point_cloud.ply
(points above the trajectory + CEILING_CLEARANCE_M cut away) with the ground-truth trajectory in
red, start green, end blue, 5 m scale bar.
"""
import argparse
import math
import os
import struct
import sys
import zlib

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

# lens bags that have the front camera IMU and a base_poses.txt
DATASETS = [
    "lens/indoor-office_20260811_133205",
    "lens/indoor-office_20260811_133749",
    "lens/indoor-office_20260811_134400",
    "lens/indoor-office_20260811_134658",
    "lens/indoor-office_20260811_135008",
]
IMU_TOPIC = "/camera/camera/imu"
IMU_FRAME = "camera_camera_imu"
CAMERA_FRAME = "camera_camera_left"
GPS_PERIOD_S = 1.0
MAX_GT_GAP_S = 2.0
METERS_PER_PIXEL = 0.03
CEILING_CLEARANCE_M = 1.0
SCALE_BAR_M = 5.0


def open_bag(bag_dir, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def read_extrinsic(bag_dir):
    """Returns (translation, quaternion xyzw) of the camera in the IMU frame from /tf_static."""
    reader = open_bag(bag_dir, ["/tf_static"])
    while reader.has_next():
        _, data, _ = reader.read_next()
        for transform in deserialize_message(data, TFMessage).transforms:
            if transform.header.frame_id == IMU_FRAME and transform.child_frame_id == CAMERA_FRAME:
                t, q = transform.transform.translation, transform.transform.rotation
                return np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w])
    sys.exit(f"{bag_dir}: no {IMU_FRAME} -> {CAMERA_FRAME} in /tf_static")


def write_imu(bag_dir, path):
    reader = open_bag(bag_dir, [IMU_TOPIC])
    rows, frames = [], set()
    while reader.has_next():
        _, data, _ = reader.read_next()
        imu = deserialize_message(data, Imu)
        frames.add(imu.header.frame_id)
        a, w = imu.linear_acceleration, imu.angular_velocity
        rows.append((imu.header.stamp.sec + 1e-9 * imu.header.stamp.nanosec,
                     a.x, a.y, a.z, w.x, w.y, w.z))
    if not rows:
        sys.exit(f"{bag_dir}: no {IMU_TOPIC}")
    rows.sort()
    with open(path, "w") as f:
        f.write(f"# {IMU_TOPIC} frame {','.join(sorted(frames))} (axes of {CAMERA_FRAME})\n"
                "# t_s ax ay az (m/s^2) gx gy gz (rad/s)\n")
        for row in rows:
            f.write(f"{row[0]:.9f} " + " ".join(f"{v:.9f}" for v in row[1:]) + "\n")
    return np.array(rows)


def rotation_matrix(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quaternion_product(a, b):
    """Hamilton product of xyzw quaternions."""
    (ax, ay, az, aw), (bx, by, bz, bw) = a, b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def imu_poses(camera_poses, translation, quaternion):
    """world_from_imu = world_from_camera * inverse(imu_from_camera), as TUM rows."""
    camera_from_imu = np.array([-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]])
    imu_in_camera = -rotation_matrix(quaternion).T @ translation
    rows = []
    for pose in camera_poses:
        q = quaternion_product(pose[4:8], camera_from_imu)
        q *= np.sign(q[3]) or 1.0
        rows.append(np.concatenate(([pose[0]], pose[1:4] + rotation_matrix(pose[4:8]) @ imu_in_camera,
                                    q / np.linalg.norm(q))))
    return np.array(rows)


def write_gps(gt, path):
    """1 Hz positions of the IMU poses `gt`; returns the number written and skipped."""
    times, imu_positions = gt[:, 0], gt[:, 1:4]
    written = skipped = 0
    with open(path, "w") as f:
        f.write("# IMU position of gt.txt at 1 Hz, world frame of gt.txt (z up), no noise\n"
                "# t_s x_m y_m z_m\n")
        for t in np.arange(math.ceil(times[0]), times[-1], GPS_PERIOD_S):
            i = min(np.searchsorted(times, t, side="right"), len(times) - 1)
            if times[i] - times[i - 1] > MAX_GT_GAP_S:
                skipped += 1
                continue
            alpha = (t - times[i - 1]) / (times[i] - times[i - 1])
            p = (1 - alpha) * imu_positions[i - 1] + alpha * imu_positions[i]
            f.write(f"{t:.6f} {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            written += 1
    return written, skipped


def write_gt(gt, translation, quaternion, path):
    with open(path, "w") as f:
        f.write(f"# world_from_{IMU_FRAME} (IMU data axes), from the map base_poses.txt "
                f"(world_from_{CAMERA_FRAME})\n"
                f"# {IMU_FRAME} -> {CAMERA_FRAME} (/tf_static): t {' '.join(map(str, translation))}"
                f" q_xyzw {' '.join(map(str, quaternion))}\n"
                "# t_s x_m y_m z_m qx qy qz qw\n")
        for pose in gt:
            f.write(f"{pose[0]:.9f} " + " ".join(f"{v:.9f}" for v in pose[1:]) + "\n")


def read_ascii_ply(path):
    """Returns (xyz, rgb) of an ASCII PLY point cloud with x y z ... red green blue properties."""
    with open(path, "rb") as f:
        properties, vertices = [], 0
        while True:
            line = f.readline().decode().strip()
            if line.startswith("format") and "ascii" not in line:
                sys.exit(f"{path}: only ASCII PLY is supported")
            if line.startswith("element vertex"):
                vertices = int(line.split()[2])
            elif line.startswith("property"):
                properties.append(line.split()[-1])
            elif line == "end_header":
                break
        values = np.loadtxt(f, max_rows=vertices, ndmin=2)
    column = {name: i for i, name in enumerate(properties)}
    xyz = values[:, [column["x"], column["y"], column["z"]]]
    rgb = values[:, [column["red"], column["green"], column["blue"]]].astype(np.uint8)
    return xyz, rgb


def write_png(path, image):
    height, width, _ = image.shape
    raw = b"".join(b"\x00" + image[row].tobytes() for row in range(height))

    def chunk(kind, data):
        return (struct.pack(">I", len(data)) + kind + data +
                struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2,
                                                                  0, 0, 0)) +
                chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def render_map(ply_path, gt, path):
    xyz, rgb = read_ascii_ply(ply_path)
    keep = xyz[:, 2] < gt[:, 3].max() + CEILING_CLEARANCE_M
    xyz, rgb = xyz[keep], rgb[keep]
    margin = 1.0
    low = np.minimum(xyz[:, :2].min(0), gt[:, 1:3].min(0)) - margin
    high = np.maximum(xyz[:, :2].max(0), gt[:, 1:3].max(0)) + margin
    width, height = np.ceil((high - low) / METERS_PER_PIXEL).astype(int) + 1

    def pixel(points):  # world x right, y up
        column = ((points[:, 0] - low[0]) / METERS_PER_PIXEL).astype(int)
        row = ((high[1] - points[:, 1]) / METERS_PER_PIXEL).astype(int)
        return np.clip(row, 0, height - 1), np.clip(column, 0, width - 1)

    image = np.full((height, width, 3), 255, np.uint8)
    # draw low to high so each pixel shows the highest point under the ceiling cut
    order = np.argsort(xyz[:, 2])
    rows, columns = pixel(xyz[order])
    image[rows, columns] = rgb[order]

    def disc(row, column, radius, color):
        r0, r1 = max(row - radius, 0), min(row + radius + 1, height)
        c0, c1 = max(column - radius, 0), min(column + radius + 1, width)
        rr, cc = np.mgrid[r0:r1, c0:c1]
        inside = (rr - row) ** 2 + (cc - column) ** 2 <= radius ** 2
        image[rr[inside], cc[inside]] = color

    # trajectory densified to one sample per pixel, drawn 2 px wide
    positions = gt[:, 1:3]
    steps = np.maximum(np.ceil(np.linalg.norm(np.diff(positions, axis=0), axis=1) /
                               METERS_PER_PIXEL), 1).astype(int)
    dense = np.vstack([np.linspace(a, b, n, endpoint=False)
                       for a, b, n in zip(positions[:-1], positions[1:], steps)] + [positions[-1:]])
    for row, column in zip(*pixel(dense)):
        disc(row, column, 2, (220, 30, 30))
    rows, columns = pixel(positions[[0, -1]])
    disc(rows[0], columns[0], 8, (30, 170, 30))
    disc(rows[1], columns[1], 8, (30, 60, 220))
    bar = int(SCALE_BAR_M / METERS_PER_PIXEL)
    image[height - 25:height - 19, 20:20 + bar] = 0
    write_png(path, image)
    return width, height


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rosbags", default="/mnt/car/raw/looper/rosbags")
    parser.add_argument("--maps", default="/mnt/car/raw/looper/benchmark/CoreMapGlobalMaps")
    parser.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "datasets"))
    parser.add_argument("--images", default="output/benchmark/dataset_maps")
    parser.add_argument("names", nargs="*", default=DATASETS,
                        help="bag directories relative to --rosbags")
    args = parser.parse_args()
    for name in args.names:
        bag_dir = os.path.join(args.rosbags, name)
        map_dir = os.path.join(args.maps, os.path.basename(name), "_create_space_request")
        out_dir = os.path.join(args.out, os.path.basename(name))
        os.makedirs(out_dir, exist_ok=True)
        os.makedirs(args.images, exist_ok=True)
        translation, quaternion = read_extrinsic(bag_dir)
        imu = write_imu(bag_dir, os.path.join(out_dir, "imu.txt"))
        gt = imu_poses(np.loadtxt(os.path.join(map_dir, "base_poses.txt"), ndmin=2), translation,
                       quaternion)
        write_gt(gt, translation, quaternion, os.path.join(out_dir, "gt.txt"))
        written, skipped = write_gps(gt, os.path.join(out_dir, "gps.txt"))
        width, height = render_map(os.path.join(map_dir, "point_cloud.ply"), gt,
                                   os.path.join(args.images, os.path.basename(name) + ".png"))
        print(f"{out_dir}: {len(imu)} IMU ({imu[-1, 0] - imu[0, 0]:.1f} s), {len(gt)} gt "
              f"({gt[-1, 0] - gt[0, 0]:.1f} s), {written} GPS ({skipped} skipped in gt gaps), "
              f"map {width}x{height}", flush=True)


if __name__ == "__main__":
    main()
