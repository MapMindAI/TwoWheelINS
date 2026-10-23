// Copyright 2026 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#include "common/reader/txt_record_reader.h"

#include <cstdlib>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

namespace mapmind::reader {
namespace {

std::string WriteTempFile(const std::string& content) {
  const std::string path = std::string(std::getenv("TEST_TMPDIR")) + "/record.txt";
  std::ofstream(path) << content;
  return path;
}

TEST(TxtRecordReader, ParsesAllLineTypes) {
  const std::string path = WriteTempFile(
      "imu 484355349 -1.00830 -8.46400 -4.02124 -0.00017 -0.00412 0.00297\n"
      "speed 484355349 18\n"
      "gps 484747380 0 22.7878390000 113.5210145000 13.0000000000 0.093 8\n");
  const std::optional<TxtRecord> record = ReadTxtRecord(path);
  ASSERT_TRUE(record.has_value());
  EXPECT_EQ(record->skipped_lines, 0);

  ASSERT_EQ(record->imu.size(), 1u);
  EXPECT_EQ(record->imu[0].timestamp_ns, 484355349000);
  EXPECT_DOUBLE_EQ(record->imu[0].linear_acceleration.y(), -8.464);
  EXPECT_DOUBLE_EQ(record->imu[0].angular_velocity.z(), 0.00297);

  ASSERT_EQ(record->wheel_speed.size(), 1u);
  EXPECT_DOUBLE_EQ(record->wheel_speed[0].speed_mps, 5.0);

  ASSERT_EQ(record->gps.size(), 1u);
  EXPECT_EQ(record->gps[0].timestamp_ns, 484747380000);
  EXPECT_DOUBLE_EQ(record->gps[0].latitude_deg, 22.787839);
  EXPECT_DOUBLE_EQ(record->gps[0].longitude_deg, 113.5210145);
  EXPECT_DOUBLE_EQ(record->gps[0].altitude_m, 13.0);
  EXPECT_DOUBLE_EQ(record->gps[0].ground_speed_mps, 0.093);
  EXPECT_EQ(record->gps[0].satellite_count, 8);
}

TEST(TxtRecordReader, SkipsTruncatedUnknownAndInvalidLines) {
  const std::string path = WriteTempFile(
      "imu 1 0 0 9.8 0 0 0\n"
      "speed 2 255\n"
      "unknown 3 1 2\n"
      "gps 4 0 22.78 113.52 13.0\n"
      "imu 5 0 0 9.8 0 0 0 7\n"
      "imu 6 0.74485 -6.18394 -4.01405 -0.09524 0");
  const std::optional<TxtRecord> record = ReadTxtRecord(path);
  ASSERT_TRUE(record.has_value());
  EXPECT_EQ(record->imu.size(), 1u);
  EXPECT_TRUE(record->gps.empty());
  EXPECT_TRUE(record->wheel_speed.empty());
  EXPECT_EQ(record->skipped_lines, 5);
}

TEST(TxtRecordReader, MissingFile) {
  EXPECT_FALSE(ReadTxtRecord("/nonexistent/record.txt").has_value());
}

sensor::ImuData Imu(int64_t timestamp_ns, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
  return {timestamp_ns, acc, gyr};
}

TEST(RemoveCorruptImuSamples, DropsOutOfRangeAndSpikes) {
  const Eigen::Vector3d gravity(0, -8.5, -4.5);
  const Eigen::Vector3d still = Eigen::Vector3d::Zero();
  std::vector<sensor::ImuData> imu = {
      Imu(0, gravity, still),
      Imu(1, Eigen::Vector3d(66.5, 58.9, -4.1), still),  // out of range
      Imu(2, gravity, still),
      Imu(3, gravity, Eigen::Vector3d(0.94, 0.84, 0.91)),  // gyroscope spike
      Imu(4, gravity, still),
      Imu(5, gravity + Eigen::Vector3d(0, 20, 0), still),  // accelerometer spike
      Imu(6, gravity, still),
      Imu(7, gravity, Eigen::Vector3d(0, 1.0, 0)),  // sustained turn, kept
      Imu(8, gravity, Eigen::Vector3d(0, 1.1, 0)),
      Imu(9, gravity, Eigen::Vector3d(0, 1.1, 0)),
  };
  EXPECT_EQ(RemoveCorruptImuSamples(&imu), 3);
  std::vector<int64_t> kept;
  for (const sensor::ImuData& sample : imu) {
    kept.push_back(sample.timestamp_ns);
  }
  EXPECT_EQ(kept, (std::vector<int64_t>{0, 2, 4, 6, 7, 8, 9}));
}

}  // namespace
}  // namespace mapmind::reader
