// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/reader/benchmark_dataset_reader.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace mapmind::reader {

namespace {

// Reads every non-comment line of `path` as `columns` numbers; nullopt on any malformed line.
std::optional<std::vector<std::vector<double>>> ReadColumns(const std::string& path, int columns) {
  std::ifstream file(path);
  if (!file.is_open()) {
    return std::nullopt;
  }
  std::vector<std::vector<double>> rows;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream stream(line);
    std::vector<double>& row = rows.emplace_back(columns);
    for (double& value : row) {
      stream >> value;
    }
    std::string rest;
    if (stream.fail() || stream >> rest) {
      return std::nullopt;
    }
  }
  return rows;
}

int64_t Nanoseconds(double seconds) { return std::llround(seconds * 1e9); }

}  // namespace

std::optional<BenchmarkDataset> ReadBenchmarkDataset(const std::string& directory) {
  const std::optional<std::vector<std::vector<double>>> imu_rows =
      ReadColumns(directory + "/imu.txt", 7);
  const std::optional<std::vector<std::vector<double>>> gps_rows =
      ReadColumns(directory + "/gps.txt", 4);
  if (!imu_rows || !gps_rows) {
    return std::nullopt;
  }
  BenchmarkDataset dataset;
  for (const std::vector<double>& row : *imu_rows) {
    dataset.imu.push_back({Nanoseconds(row[0]), Eigen::Vector3d(row[1], row[2], row[3]),
                           Eigen::Vector3d(row[4], row[5], row[6])});
  }
  for (const std::vector<double>& row : *gps_rows) {
    dataset.gps.push_back({Nanoseconds(row[0]), Eigen::Vector3d(row[1], row[2], row[3])});
  }
  return dataset;
}

}  // namespace mapmind::reader
