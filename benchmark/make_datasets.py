# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
"""Builds the benchmark datasets under benchmark/datasets/<name>/. Needs the ROS 2 Python
environment (source /opt/ros/<distro>/setup.bash; uses rosbag2_py and numpy only):

  /usr/bin/python3 benchmark/make_datasets.py [--rosbags DIR] [--maps DIR] [--unilidar DIR]
      [--out DIR] [--images DIR] [names...]

Two sources:
  CoreMap   looper rosbags (--rosbags) with a camera IMU, and the CoreMap global map (--maps)
            whose base_poses.txt is world_from_camera_camera_left. The IMU pose follows from the
            /tf_static chain: axes of the IMU's _optical frame, origin at its link frame (the
            _optical frame without the suffix).
  unilidar  unilidar_looper sessions (--unilidar) with /unilidar/imu and the lidar SLAM's
            unilidar_imu_trajectory.proto.bin (world_from_unilidar_imu at IMU stamps), less
            TRAJECTORY_TRIM_S at both ends. The SLAM world is not level, so it is rotated to put
            the mean world specific force on +z.

Each dataset gets:
  imu.txt  t_s ax ay az (m/s^2) gx gy gz (rad/s) at 100 Hz: the mean of the samples in each
           10 ms window, stamped at the window end (header stamps). Windows short of samples are
           dropped, so IMU dropouts stay gaps.
  gps.txt  t_s x y z (m): IMU position in the ground-truth world frame (z up), sampled at 1 Hz
           by linear interpolation of gt.txt; no noise. Skipped where the two neighbouring
           ground-truth poses are more than MAX_GT_GAP_S apart.
  gt.txt   TUM world_from_imu, at most 100 Hz.
--images gets <group>.png per group of datasets (the name up to the first underscore) for the wiki
Benchmark-Datasets page: one panel per dataset in name order, PANELS_PER_ROW per row at a common
scale, each a top-down view of its own map point cloud within MAP_MARGIN_M of the trajectory
(points above the trajectory + CEILING_CLEARANCE_M cut away; shaded by height if it has no colour)
with the ground-truth trajectory in red, start green, end blue and a 10 m scale bar. Each dataset
has its own map frame, so panels are not aligned with each other.

Each dataset is checked against its ground truth: gravity (resting specific force rotated to the
world, should be ~(0, 0, 9.8)) and the gyro integrated between ground-truth poses (relative
rotation error, should be a few percent).
"""
import argparse
import glob
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

FRONT_IMU = "/camera/camera/imu"
BACK_IMU = "/back/camera/camera/imu"
UNILIDAR_IMU = "/unilidar/imu"
# CoreMap datasets: (bag directory relative to --rosbags, IMU topic). The lens bags (the 3 longest)
# have the front camera IMU, with ~1 s dropouts every 2-4 s; scene0 only records the back camera
# IMU, complete.
COREMAP_DATASETS = [
    ("lens/indoor-office_20260811_133205", FRONT_IMU),
    ("lens/indoor-office_20260811_133749", FRONT_IMU),
    ("lens/indoor-office_20260811_134658", FRONT_IMU),
    ("scene0_outdoor/daytime/dbot_20261023T042109Z", BACK_IMU),
    ("scene0_outdoor/daytime/dbot_20261023T043217Z", BACK_IMU),
]
# unilidar_looper sessions in strong light or daylight
UNILIDAR_DATASETS = [
    "garden_cloudy_20260715_124158_strong_light",
    "garden_day_20260716_080819",
    "grass_cloudy_20260715_124004_strong_light",
    "playground_cloudy_20260715_124806_strong_light",
    "playground_day_20260701_031131_A",
    "playground_day_20260702_072203_color",
    "sideroad_day_20260701_032435",
]
CAMERA_FRAME = "camera_camera_left"
OUTPUT_PERIOD_S = 0.01  # 100 Hz
# the unilidar trajectory's first poses can be off by tens of degrees for up to 0.3 s
TRAJECTORY_TRIM_S = 0.5
GPS_PERIOD_S = 1.0
MAX_GT_GAP_S = 2.0
MAX_PANEL_PIXELS = 700
PANELS_PER_ROW = 3
MAP_MARGIN_M = 5.0
CEILING_CLEARANCE_M = 1.0
SCALE_BAR_M = 10.0


def open_bag(bag_dir, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=""),  # mcap or sqlite3
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def rotation_matrix(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quaternion(rotation):
    """xyzw quaternion with w >= 0 of a rotation matrix."""
    w = math.sqrt(max(0.0, 1.0 + np.trace(rotation))) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] -
                                    rotation[2, 2])) / 2.0, rotation[2, 1] - rotation[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - rotation[0, 0] + rotation[1, 1] -
                                    rotation[2, 2])) / 2.0, rotation[0, 2] - rotation[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - rotation[0, 0] - rotation[1, 1] +
                                    rotation[2, 2])) / 2.0, rotation[1, 0] - rotation[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def transform(translation, q):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation_matrix(np.asarray(q))
    matrix[:3, 3] = translation
    return matrix


def read_tf_static(bag_dir):
    """Returns {child: (parent, parent_from_child 4x4)}."""
    tree = {}
    reader = open_bag(bag_dir, ["/tf_static"])
    while reader.has_next():
        _, data, _ = reader.read_next()
        for t in deserialize_message(data, TFMessage).transforms:
            p, q = t.transform.translation, t.transform.rotation
            tree[t.child_frame_id] = (t.header.frame_id,
                                      transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
    return tree


def root_from(tree, frame):
    """(root frame, root_from_frame) following the parents in `tree`."""
    matrix = np.eye(4)
    while frame in tree:
        parent, parent_from_child = tree[frame]
        matrix = parent_from_child @ matrix
        frame = parent
    return frame, matrix


def camera_from_imu(tree, imu_frame):
    """camera_camera_left_from_imu: rotation of `imu_frame`, origin of its link frame."""
    link = imu_frame[:-len("_optical")] if imu_frame.endswith("_optical") else imu_frame
    roots = {root_from(tree, frame)[0] for frame in (CAMERA_FRAME, imu_frame, link)}
    if len(roots) != 1:
        sys.exit(f"/tf_static does not connect {CAMERA_FRAME}, {imu_frame} and {link}")
    camera_inverse = np.linalg.inv(root_from(tree, CAMERA_FRAME)[1])
    matrix = camera_inverse @ root_from(tree, imu_frame)[1]
    matrix[:3, 3] = (camera_inverse @ root_from(tree, link)[1])[:3, 3]
    return matrix


def read_imu(bag_dir, topic):
    """Returns (frame id, rows t_s ax ay az gx gy gz) of `topic` sorted by header stamp."""
    reader = open_bag(bag_dir, [topic])
    rows, frames = [], set()
    while reader.has_next():
        _, data, _ = reader.read_next()
        imu = deserialize_message(data, Imu)
        frames.add(imu.header.frame_id)
        a, w = imu.linear_acceleration, imu.angular_velocity
        rows.append((imu.header.stamp.sec + 1e-9 * imu.header.stamp.nanosec,
                     a.x, a.y, a.z, w.x, w.y, w.z))
    if len(frames) != 1:
        sys.exit(f"{bag_dir}: {topic} frames {frames or 'none'}")
    rows.sort()
    return frames.pop(), np.array(rows)


def downsample(imu):
    """Means of the samples in each OUTPUT_PERIOD_S window (start, end], stamped at the end.
    Windows with fewer samples than a full window holds at the median rate are dropped."""
    period = np.median(np.diff(imu[:, 0]))
    min_samples = int(math.floor(round(OUTPUT_PERIOD_S / period, 6)))
    window = np.ceil(np.round(imu[:, 0] / OUTPUT_PERIOD_S, 9)).astype(np.int64)
    starts = np.flatnonzero(np.diff(window, prepend=window[0] - 1))
    rows = []
    for start, end in zip(starts, np.append(starts[1:], len(imu))):
        if end - start >= min_samples:
            rows.append(np.concatenate(([window[start] * OUTPUT_PERIOD_S],
                                        imu[start:end, 1:].mean(axis=0))))
    return np.array(rows), 1.0 / period


def write_imu(imu, topic, frame, rate_hz, path):
    with open(path, "w") as f:
        f.write(f"# {topic} frame {frame}, 100 Hz: mean over each 10 ms of {rate_hz:.0f} Hz "
                "samples, stamped at the window end\n# t_s ax ay az (m/s^2) gx gy gz (rad/s)\n")
        for row in imu:
            f.write(f"{row[0]:.9f} " + " ".join(f"{v:.9f}" for v in row[1:]) + "\n")


def imu_poses(camera_poses, camera_from_imu_matrix):
    """world_from_imu TUM rows from world_from_camera TUM rows."""
    rows = []
    for pose in camera_poses:
        world_from_imu = transform(pose[1:4], pose[4:8]) @ camera_from_imu_matrix
        rows.append(np.concatenate(([pose[0]], world_from_imu[:3, 3],
                                    quaternion(world_from_imu[:3, :3]))))
    return np.array(rows)


def write_gt(gt, header, path):
    with open(path, "w") as f:
        f.write(header + "# t_s x_m y_m z_m qx qy qz qw\n")
        for pose in gt:
            f.write(f"{pose[0]:.9f} " + " ".join(f"{v:.9f}" for v in pose[1:]) + "\n")


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


def exp_so3(vector):
    angle = np.linalg.norm(vector)
    if angle < 1e-12:
        return np.eye(3)
    k = vector / angle
    skew = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew


def rotation_angle(rotation):
    return math.acos(max(-1.0, min(1.0, (np.trace(rotation) - 1.0) / 2.0)))


def thin(poses, period):
    """Poses at least `period` apart, starting with the first."""
    keep, last = [], -np.inf
    for i, t in enumerate(poses[:, 0]):
        if t - last >= period - 1e-6:
            keep.append(i)
            last = t
    return poses[keep]


def check(imu, gt):
    """(world gravity at the first pose, median relative error of the gyro rotation between
    ground-truth poses >= 0.1 s apart that turn > 3 deg with no IMU gap in between)."""
    resting = np.abs(imu[:, 0] - gt[0, 0]) < 0.25
    gravity = rotation_matrix(gt[0, 4:8]) @ imu[resting, 1:4].mean(axis=0)
    period = OUTPUT_PERIOD_S
    errors = []
    poses = thin(gt, 0.1)
    for before, after in zip(poses[:-1], poses[1:]):
        # each sample is the mean over the period ending at its stamp
        inside = np.flatnonzero((imu[:, 0] > before[0]) & (imu[:, 0] < after[0] + period))
        if len(inside) == 0 or after[0] - before[0] > 0.6 or \
                np.diff(imu[inside, 0], prepend=before[0]).max() > 1.5 * period:
            continue
        truth = rotation_matrix(before[4:8]).T @ rotation_matrix(after[4:8])
        if rotation_angle(truth) < math.radians(3.0):
            continue
        integrated, previous = np.eye(3), before[0]
        for k in inside:
            end = min(imu[k, 0], after[0])
            integrated = integrated @ exp_so3(imu[k, 4:7] * (end - previous))
            previous = end
        errors.append(rotation_angle(integrated.T @ truth) / rotation_angle(truth))
    return gravity, np.median(errors) if errors else float("nan")


def proto_fields(data):
    """Yields (field number, value) of a protobuf message: int, float, or bytes."""
    def varint(i):
        value = shift = 0
        while True:
            byte = data[i]
            i += 1
            value |= (byte & 0x7F) << shift
            shift += 7
            if byte < 0x80:
                return value, i

    i = 0
    while i < len(data):
        key, i = varint(i)
        wire_type = key & 7
        if wire_type == 0:
            value, i = varint(i)
        elif wire_type == 1:
            value, i = struct.unpack_from("<d", data, i)[0], i + 8
        elif wire_type == 2:
            length, i = varint(i)
            value, i = data[i:i + length], i + length
        elif wire_type == 5:
            value, i = struct.unpack_from("<f", data, i)[0], i + 4
        else:
            sys.exit(f"unsupported protobuf wire type {wire_type}")
        yield key >> 3, value


def proto_doubles(data, count):
    values = [0.0] * count
    for field, value in proto_fields(data):
        values[field - 1] = value
    return values


def read_trajectory(path):
    """TUM rows of a dm.internal.proto.output.Trajectory: repeated points {int64 timestamp (ns),
    Pose pose {Vec3d translation, Quaternion rotation (x y z w)}}."""
    rows = []
    with open(path, "rb") as f:
        data = f.read()
    for _, point in proto_fields(data):
        timestamp_ns, translation, rotation = 0, [0.0] * 3, [0.0, 0.0, 0.0, 1.0]
        for field, value in proto_fields(point):
            if field == 1:
                timestamp_ns = value
            elif field == 2:
                for pose_field, pose_value in proto_fields(value):
                    if pose_field == 1:
                        translation = proto_doubles(pose_value, 3)
                    elif pose_field == 2:
                        rotation = proto_doubles(pose_value, 4)
        rows.append([timestamp_ns * 1e-9] + translation + rotation)
    return np.array(sorted(rows))


def levelling(imu, gt):
    """Rotation that takes the mean world specific force over the run onto +z."""
    near = np.clip(np.searchsorted(gt[:, 0], imu[:, 0]), 0, len(gt) - 1)
    close = np.abs(gt[near, 0] - imu[:, 0]) < OUTPUT_PERIOD_S
    up = sum(rotation_matrix(gt[k, 4:8]) @ f for k, f in zip(near[close], imu[close, 1:4]))
    up /= np.linalg.norm(up)
    axis = np.cross(up, [0.0, 0.0, 1.0])
    angle = math.atan2(np.linalg.norm(axis), up[2])
    return exp_so3(angle * axis / max(np.linalg.norm(axis), 1e-12))


def read_ascii_ply(path):
    """Returns (xyz, rgb or None) of an ASCII PLY point cloud with x y z [... red green blue]."""
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
    if "red" not in column:
        return xyz, None
    return xyz, values[:, [column["red"], column["green"], column["blue"]]].astype(np.uint8)


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


def group_name(name):
    """Datasets are grouped by the name up to the first underscore (place or recorder)."""
    return name.split("_")[0]


def map_points(ply_path, world_from_map, gt):
    """World points of the map within MAP_MARGIN_M of the trajectory, below the ceiling cut, and
    their colours (shaded by height if the PLY has none)."""
    xyz, rgb = read_ascii_ply(ply_path)
    xyz = xyz @ world_from_map.T
    low = gt[:, 1:3].min(axis=0) - MAP_MARGIN_M
    high = gt[:, 1:3].max(axis=0) + MAP_MARGIN_M
    keep = np.all((xyz[:, :2] >= low) & (xyz[:, :2] <= high), axis=1) & \
        (xyz[:, 2] < gt[:, 3].max() + CEILING_CLEARANCE_M)
    xyz = xyz[keep]
    if rgb is None:
        low_z, high_z = np.percentile(xyz[:, 2], [2, 98])
        shade = 200 - 160 * np.clip((xyz[:, 2] - low_z) / max(high_z - low_z, 1e-3), 0, 1)
        rgb = np.repeat(shade[:, None], 3, axis=1).astype(np.uint8)
    else:
        rgb = rgb[keep]
    return xyz.astype(np.float32), rgb


def render_panel(xyz, rgb, gt, meters_per_pixel):
    """Top-down image of the map points with the trajectory: red, start green, end blue, and a
    SCALE_BAR_M bar at the bottom left."""
    low = gt[:, 1:3].min(axis=0) - MAP_MARGIN_M
    high = gt[:, 1:3].max(axis=0) + MAP_MARGIN_M
    width, height = np.ceil((high - low) / meters_per_pixel).astype(int) + 1

    def pixel(points):  # world x right, y up
        column = ((points[:, 0] - low[0]) / meters_per_pixel).astype(int)
        row = ((high[1] - points[:, 1]) / meters_per_pixel).astype(int)
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

    # trajectory densified to one sample per pixel
    positions = gt[:, 1:3]
    steps = np.maximum(np.ceil(np.linalg.norm(np.diff(positions, axis=0), axis=1) /
                               meters_per_pixel), 1).astype(int)
    dense = np.vstack([np.linspace(a, b, n, endpoint=False)
                       for a, b, n in zip(positions[:-1], positions[1:], steps)] + [positions[-1:]])
    for row, column in zip(*pixel(dense)):
        disc(row, column, 1, (220, 30, 30))
    rows, columns = pixel(positions[[0, -1]])
    disc(rows[0], columns[0], 5, (30, 170, 30))
    disc(rows[1], columns[1], 5, (30, 60, 220))
    bar = int(round(SCALE_BAR_M / meters_per_pixel))
    image[height - 14:height - 10, 10:10 + bar] = 0
    return image


def render_group(panels, path):
    """One image of the (xyz, rgb, gt) panels at a common scale, PANELS_PER_ROW per row, each
    framed in grey; the longest panel side is MAX_PANEL_PIXELS."""
    extent = max(np.ptp(gt[:, 1:3], axis=0).max() + 2 * MAP_MARGIN_M for _, _, gt in panels)
    meters_per_pixel = extent / MAX_PANEL_PIXELS
    images = []
    for xyz, rgb, gt in panels:
        image = render_panel(xyz, rgb, gt, meters_per_pixel)
        image[[0, -1], :] = 160
        image[:, [0, -1]] = 160
        images.append(image)
    gap = 16
    rows = [images[i:i + PANELS_PER_ROW] for i in range(0, len(images), PANELS_PER_ROW)]
    width = max(sum(image.shape[1] for image in row) + gap * (len(row) + 1) for row in rows)
    height = sum(max(image.shape[0] for image in row) for row in rows) + gap * (len(rows) + 1)
    canvas = np.full((height, width, 3), 255, np.uint8)
    top = gap
    for row in rows:
        left = gap
        for image in row:
            canvas[top:top + image.shape[0], left:left + image.shape[1]] = image
            left += image.shape[1] + gap
        top += max(image.shape[0] for image in row) + gap
    write_png(path, canvas)
    return width, height, meters_per_pixel


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rosbags", default="/mnt/car/raw/looper/rosbags")
    parser.add_argument("--maps", default="/mnt/car/raw/looper/benchmark/CoreMapGlobalMaps")
    parser.add_argument("--unilidar", default="/mnt/car/raw/looper/unilidar_looper")
    parser.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "datasets"))
    parser.add_argument("--images", default="output/benchmark/dataset_maps")
    parser.add_argument("names", nargs="*", help="subset of the dataset names to build")
    args = parser.parse_args()
    os.makedirs(args.images, exist_ok=True)
    jobs = [(os.path.basename(bag), "coremap", os.path.join(args.rosbags, bag), topic)
            for bag, topic in COREMAP_DATASETS]
    jobs += [(name, "unilidar", os.path.join(args.unilidar, name), UNILIDAR_IMU)
             for name in UNILIDAR_DATASETS]
    groups = {}
    for name, source, bag_dir, topic in jobs:
        if args.names and name not in args.names:
            continue
        out_dir = os.path.join(args.out, name)
        os.makedirs(out_dir, exist_ok=True)
        frame, raw_imu = read_imu(bag_dir, topic)
        imu, rate_hz = downsample(raw_imu)
        write_imu(imu, topic, frame, rate_hz, os.path.join(out_dir, "imu.txt"))
        if source == "coremap":
            base_poses = glob.glob(os.path.join(args.maps, name, "*", "base_poses.txt"))
            if len(base_poses) != 1:
                sys.exit(f"{name}: expected one base_poses.txt, found {base_poses}")
            camera_from_imu_matrix = camera_from_imu(read_tf_static(bag_dir), frame)
            gt = imu_poses(np.loadtxt(base_poses[0], ndmin=2), camera_from_imu_matrix)
            world_from_map = np.eye(3)
            ply = os.path.join(os.path.dirname(base_poses[0]), "point_cloud.ply")
            translation = " ".join(f"{v:.6f}" for v in camera_from_imu_matrix[:3, 3])
            q = " ".join(f"{v:.9f}" for v in quaternion(camera_from_imu_matrix[:3, :3]))
            header = (f"# world_from_imu ({frame} axes), from the map base_poses.txt "
                      f"(world_from_{CAMERA_FRAME})\n"
                      f"# {CAMERA_FRAME}_from_imu (/tf_static): t {translation} q_xyzw {q}\n")
        else:
            slam = read_trajectory(os.path.join(bag_dir, "unilidar_imu_trajectory.proto.bin"))
            inside = (slam[:, 0] >= slam[0, 0] + TRAJECTORY_TRIM_S) & \
                (slam[:, 0] <= slam[-1, 0] - TRAJECTORY_TRIM_S)
            slam = thin(slam[inside], OUTPUT_PERIOD_S)
            world_from_map = levelling(raw_imu, slam)
            gt = imu_poses(slam, np.eye(4))
            for row in gt:
                row[1:4] = world_from_map @ row[1:4]
                row[4:8] = quaternion(world_from_map @ rotation_matrix(row[4:8]))
            ply = os.path.join(bag_dir, "unilidar_map.ply")
            tilt = math.degrees(math.acos(min(1.0, (np.trace(world_from_map) - 1.0) / 2.0)))
            header = (f"# world_from_imu ({frame}), from unilidar_imu_trajectory.proto.bin with "
                      f"the SLAM world levelled by {tilt:.2f} deg, {TRAJECTORY_TRIM_S} s trimmed "
                      "at both ends, thinned to 100 Hz\n")
        write_gt(gt, header, os.path.join(out_dir, "gt.txt"))
        written, skipped = write_gps(gt, os.path.join(out_dir, "gps.txt"))
        xyz, rgb = map_points(ply, world_from_map, gt)
        groups.setdefault(group_name(name), []).append((name, (xyz, rgb, gt)))
        gravity, gyro_error = check(imu, gt)
        gaps = np.count_nonzero(np.diff(imu[:, 0]) > 0.5)
        print(f"{name}: {len(raw_imu)} IMU at {rate_hz:.0f} Hz -> {len(imu)} at 100 Hz "
              f"({imu[-1, 0] - imu[0, 0]:.1f} s, {gaps} gaps > 0.5 s), {len(gt)} gt "
              f"({gt[-1, 0] - gt[0, 0]:.1f} s), {written} GPS ({skipped} skipped in gt gaps)"
              f"\n  check: gravity in world {np.round(gravity, 2)} m/s^2, "
              f"gyro vs gt rotation error {100 * gyro_error:.1f} %", flush=True)

    every_name = [name for name, _, _, _ in jobs]
    for group, members in sorted(groups.items()):
        expected = [name for name in every_name if group_name(name) == group]
        if [name for name, _ in members] != expected:
            print(f"{group}: not all of {expected} built, group image not written")
            continue
        width, height, meters_per_pixel = render_group(
            [panel for _, panel in members], os.path.join(args.images, group + ".png"))
        print(f"{group}.png: {', '.join(expected)} ({width}x{height}, "
              f"{100 * meters_per_pixel:.1f} cm/pixel)")

if __name__ == "__main__":
    main()
