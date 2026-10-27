// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/reader/benchmark_dataset_reader.h"

#include <cstdlib>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

namespace mapmind::reader {
namespace {

std::string WriteDataset(const std::string& imu, const std::string& gps) {
  const std::string directory = std::getenv("TEST_TMPDIR");
  std::ofstream(directory + "/imu.txt") << imu;
  std::ofstream(directory + "/gps.txt") << gps;
  return directory;
}

TEST(BenchmarkDatasetReader, ParsesImuAndGps) {
  const std::string directory = WriteDataset(
      "# /camera/camera/imu\n"
      "# t_s ax ay az (m/s^2) gx gy gz (rad/s)\n"
      "2985.874768848 0.128566876 -9.725795513 -1.216745513 0.005325774 -0.002131883 0.000008573\n"
      "2985.877268848 0.132379800 -9.607339534 -1.349668456 0.009590439 -0.000003339 "
      "-0.002116716\n",
      "# t_s x_m y_m z_m\n"
      "2993.000000 1.2903 -1.1750 -1.1886\n");
  const std::optional<BenchmarkDataset> dataset = ReadBenchmarkDataset(directory);
  ASSERT_TRUE(dataset.has_value());

  ASSERT_EQ(dataset->imu.size(), 2u);
  EXPECT_EQ(dataset->imu[0].timestamp_ns, 2985874768848);
  EXPECT_EQ(dataset->imu[1].timestamp_ns, 2985877268848);
  EXPECT_DOUBLE_EQ(dataset->imu[0].linear_acceleration.y(), -9.725795513);
  EXPECT_DOUBLE_EQ(dataset->imu[1].angular_velocity.z(), -0.002116716);

  ASSERT_EQ(dataset->gps.size(), 1u);
  EXPECT_EQ(dataset->gps[0].timestamp_ns, 2993000000000);
  EXPECT_DOUBLE_EQ(dataset->gps[0].position.x(), 1.2903);
  EXPECT_DOUBLE_EQ(dataset->gps[0].position.z(), -1.1886);
}

TEST(BenchmarkDatasetReader, RejectsMalformedLine) {
  EXPECT_FALSE(ReadBenchmarkDataset(WriteDataset("2985.87 0.1 -9.7\n", "")).has_value());
  EXPECT_FALSE(ReadBenchmarkDataset(WriteDataset("", "2993.0 1.0 2.0 3.0 4.0\n")).has_value());
}

TEST(BenchmarkDatasetReader, MissingDirectory) {
  EXPECT_FALSE(ReadBenchmarkDataset("/nonexistent").has_value());
}

}  // namespace
}  // namespace mapmind::reader
