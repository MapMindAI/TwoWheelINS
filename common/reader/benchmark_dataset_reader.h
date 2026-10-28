// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_READER_BENCHMARK_DATASET_READER_H_
#define COMMON_READER_BENCHMARK_DATASET_READER_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "common/sensor/imu_data.h"

namespace mapmind::reader {

struct GpsPosition {
  int64_t timestamp_ns;
  // m, ground-truth world frame (z up)
  Eigen::Vector3d position;
};

struct BenchmarkDataset {
  std::vector<sensor::ImuData> imu;
  std::vector<GpsPosition> gps;
  // walking direction in the IMU frame (unit vector)
  Eigen::Vector3d forward;
};

// Reads imu.txt (t_s ax ay az gx gy gz), gps.txt (t_s x y z) and forward.txt (x y z) of a
// benchmark/datasets/<name> directory; '#' lines are comments. Returns nullopt if a file cannot be
// opened or a line does not parse.
std::optional<BenchmarkDataset> ReadBenchmarkDataset(const std::string& directory);

}  // namespace mapmind::reader

#endif  // COMMON_READER_BENCHMARK_DATASET_READER_H_
