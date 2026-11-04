#!/usr/bin/env python3
# Copyright 2026 MapMind
# SPDX-License-Identifier: BSD-3-Clause
"""Plots the height change from barometric pressure in a binary record (doc/binary_record_format.md)
and, if the record has GPS, the GPS altitude coloured by satellite count.

Height is relative to the mean pressure of the first second. GPS altitude (repeated positions
dropped) is shifted to match the pressure height at the first fix.

usage: tools/barometer_height.py <record> <output.png>
"""
import struct
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HEADER = b"MOKUREC\x00\x01"
SYNC = b"\xa5\x5a"
GPS = 0x03
LONG_IMU = 0x04
RECORD_SIZES = {0x01: 37, 0x02: 17, GPS: 42, LONG_IMU: 57}

GAS_CONSTANT_DRY_AIR = 287.05  # J/(kg K)
GRAVITY = 9.80665  # m/s^2
KELVIN = 273.15
REFERENCE_SECONDS = 1.0
SMOOTHING_SECONDS = 1.0


def pressure_to_relative_height(pressure_pa, reference_pressure_pa, temperature_c):
    """Height above the reference pressure level (m), hypsometric equation for an isothermal
    column of dry air at temperature_c."""
    scale_height_m = GAS_CONSTANT_DRY_AIR * (temperature_c + KELVIN) / GRAVITY
    return scale_height_m * np.log(reference_pressure_pa / np.asarray(pressure_pa))


def crc16(data):
    """CRC-16/CCITT-FALSE."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def read_record(path):
    """Long IMU rows: t_s, acc_xyz (m/s^2), gyr_xyz (rad/s), mag_xyz (mG), temperature_c,
    pressure_pa. GPS rows: t_s, latitude_deg, longitude_deg, altitude_m, satellites; a fix that
    repeats the previous position is dropped."""
    with open(path, "rb") as f:
        data = f.read()
    if not data.startswith(HEADER):
        sys.exit(f"{path}: not a binary record")
    imu, gps = [], []
    offset = len(HEADER)
    while offset + 3 <= len(data):
        size = RECORD_SIZES.get(data[offset + 2], 0) if data[offset:offset + 2] == SYNC else 0
        record = data[offset:offset + size]
        if size == 0 or len(record) < size or \
                crc16(record[2:-2]) != struct.unpack("<H", record[-2:])[0]:
            offset += 1
            continue
        t_s = struct.unpack("<q", record[3:11])[0] * 1e-6
        if record[2] == LONG_IMU:
            imu.append((t_s,) + struct.unpack("<11f", record[11:55]))
        elif record[2] == GPS:
            _, latitude, longitude, altitude, _, satellites = struct.unpack("<IddffB",
                                                                            record[11:40])
            if not gps or (latitude, longitude) != tuple(gps[-1][1:3]):
                gps.append((t_s, latitude, longitude, altitude, satellites))
        offset += size
    return np.array(imu), np.array(gps)


def moving_average(values, window):
    kernel = np.ones(window) / window
    padded = np.pad(values, (window // 2, window - 1 - window // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    rows, gps = read_record(sys.argv[1])
    if len(rows) == 0:
        sys.exit("no long IMU records")
    start_s = rows[0, 0]
    t = rows[:, 0] - start_s
    temperature_c = rows[:, 10]
    pressure_pa = rows[:, 11]
    rate_hz = (len(t) - 1) / t[-1]

    height_m = pressure_to_relative_height(
        pressure_pa, pressure_pa[t < REFERENCE_SECONDS].mean(), temperature_c.mean())
    window = max(1, round(SMOOTHING_SECONDS * rate_hz))
    smooth_height_m = moving_average(height_m, window)
    print(f"{len(t)} samples, {t[-1]:.1f} s, {rate_hz:.1f} Hz; pressure "
          f"{pressure_pa.min():.0f}..{pressure_pa.max():.0f} Pa, temperature "
          f"{temperature_c.mean():.1f} C; height {smooth_height_m.min():+.2f}.."
          f"{smooth_height_m.max():+.2f} m, end {smooth_height_m[-window:].mean():+.2f} m")

    figure, axes = plt.subplots(figsize=(11, 4.5), layout="constrained")
    axes.plot(t, height_m, ".", markersize=2, color="0.7", label="pressure, per sample")
    axes.plot(t, smooth_height_m, label=f"pressure, {SMOOTHING_SECONDS:g} s mean")
    if len(gps):
        gps_t = gps[:, 0] - start_s
        barometer_at_gps_m = np.interp(gps_t, t, smooth_height_m)
        gps_height_m = gps[:, 3] - gps[0, 3] + barometer_at_gps_m[0]
        difference_m = gps_height_m - barometer_at_gps_m
        print(f"{len(gps)} GPS fixes, {gps[:, 4].min():.0f}..{gps[:, 4].max():.0f} satellites; "
              f"GPS minus pressure height: median {np.median(difference_m):+.2f} m, "
              f"std {difference_m.std():.2f} m, range {difference_m.min():+.2f}.."
              f"{difference_m.max():+.2f} m")
        points = axes.scatter(gps_t, gps_height_m, c=gps[:, 4], s=8, cmap="viridis",
                              label="GPS altitude")
        figure.colorbar(points, ax=axes, pad=0.01, label="GPS satellites")
    axes.set_xlabel("time (s)")
    axes.set_ylabel("height (m)")
    axes.legend()
    axes.grid(alpha=0.3)
    axes.set_title(sys.argv[1])
    figure.savefig(sys.argv[2], dpi=120)
    print(f"wrote {sys.argv[2]}")


if __name__ == "__main__":
    main()
