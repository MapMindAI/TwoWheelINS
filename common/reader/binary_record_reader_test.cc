// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/reader/binary_record_reader.h"

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace mapmind::reader {
namespace {

using Bytes = std::vector<uint8_t>;

const Bytes kHeader = {'M', 'O', 'K', 'U', 'R', 'E', 'C', 0x00, 0x01};

// First record of a real capture: long IMU at 15479045 us.
const Bytes kLongImuRecord = {
    0xA5, 0x5A, 0x04, 0x05, 0x31, 0xEC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, 0x24, 0xD4, 0xBF,
    0xD7, 0x92, 0x88, 0xC0, 0x3E, 0x6F, 0x19, 0x41, 0x0B, 0x88, 0x43, 0xBF, 0xCD, 0xFC, 0x2D,
    0xBF, 0x31, 0x36, 0xD9, 0x3F, 0x03, 0xE7, 0xCC, 0xBF, 0x8D, 0x17, 0xB8, 0x41, 0xCB, 0x7F,
    0xC6, 0xC0, 0x85, 0xEB, 0xE1, 0x41, 0x00, 0x94, 0xC4, 0x47, 0xE7, 0x1F};

std::string WriteTempFile(const Bytes& bytes) {
  const std::string path = std::string(std::getenv("TEST_TMPDIR")) + "/record.bin";
  std::ofstream(path, std::ios::binary)
      .write(reinterpret_cast<const char*>(bytes.data()),
             static_cast<std::streamsize>(bytes.size()));
  return path;
}

void Append(Bytes* bytes, uint64_t value, int size) {
  for (int i = 0; i < size; ++i) {
    bytes->push_back(static_cast<uint8_t>(value >> (8 * i)));
  }
}

void AppendFloat(Bytes* bytes, float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  Append(bytes, bits, 4);
}

void AppendDouble(Bytes* bytes, double value) {
  uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  Append(bytes, bits, 8);
}

// CRC-16/CCITT-FALSE, written independently of the reader.
uint16_t Crc(const Bytes& bytes) {
  uint32_t crc = 0xFFFF;
  for (uint8_t byte : bytes) {
    for (int bit = 7; bit >= 0; --bit) {
      const bool top = ((crc >> 15) & 1) != (((byte >> bit) & 1) != 0);
      crc = ((crc << 1) & 0xFFFF) ^ (top ? 0x1021 : 0);
    }
  }
  return static_cast<uint16_t>(crc);
}

// Frames `type`, `timestamp_us` and `payload` with sync bytes and CRC.
Bytes Record(uint8_t type, int64_t timestamp_us, const Bytes& payload) {
  Bytes body = {type};
  Append(&body, static_cast<uint64_t>(timestamp_us), 8);
  body.insert(body.end(), payload.begin(), payload.end());
  Bytes record = {0xA5, 0x5A};
  record.insert(record.end(), body.begin(), body.end());
  Append(&record, Crc(body), 2);
  return record;
}

Bytes File(const std::vector<Bytes>& parts) {
  Bytes file = kHeader;
  for (const Bytes& part : parts) {
    file.insert(file.end(), part.begin(), part.end());
  }
  return file;
}

TEST(BinaryRecordReader, CrcCheckValue) {
  EXPECT_EQ(Crc({'1', '2', '3', '4', '5', '6', '7', '8', '9'}), 0x29B1);
}

TEST(BinaryRecordReader, ParsesCapturedLongImu) {
  const std::string path = WriteTempFile(File({kLongImuRecord}));
  ASSERT_TRUE(IsBinaryRecord(path));
  const std::optional<BinaryRecord> record = ReadBinaryRecord(path);
  ASSERT_TRUE(record.has_value());
  EXPECT_EQ(record->crc_errors, 0);
  EXPECT_EQ(record->skipped_bytes, 0);
  ASSERT_EQ(record->imu.size(), 1u);
  EXPECT_EQ(record->imu[0].timestamp_ns, 15479045000);
  EXPECT_FLOAT_EQ(record->imu[0].linear_acceleration.z(), 9.589658737f);
  EXPECT_FLOAT_EQ(record->imu[0].angular_velocity.z(), 1.696966290f);
  ASSERT_TRUE(record->imu[0].magnetic_field_mgauss.has_value());
  ASSERT_EQ(record->barometer.size(), 1u);
  EXPECT_FLOAT_EQ(record->imu[0].magnetic_field_mgauss->y(), 23.0114994f);
  EXPECT_FLOAT_EQ(record->barometer[0].temperature_c, 28.24f);
  EXPECT_DOUBLE_EQ(record->barometer[0].pressure_pa, 100648.0);
}

TEST(BinaryRecordReader, ParsesAllRecordTypes) {
  Bytes imu;
  for (float value : {0.5f, -1.0f, 9.75f, 0.125f, -0.25f, 0.0625f}) {
    AppendFloat(&imu, value);
  }
  Bytes speed;
  Append(&speed, 18, 4);
  Bytes invalid_speed;
  Append(&invalid_speed, 255, 4);
  Bytes gps;
  Append(&gps, 120, 4);
  AppendDouble(&gps, 22.7878390123);
  AppendDouble(&gps, 113.5210145456);
  AppendFloat(&gps, 13.5f);
  AppendFloat(&gps, 0.25f);
  gps.push_back(31);

  const std::optional<BinaryRecord> record = ReadBinaryRecord(
      WriteTempFile(File({Record(0x01, 100, imu), Record(0x02, 100, speed),
                          Record(0x02, 150, invalid_speed), Record(0x03, 200, gps)})));
  ASSERT_TRUE(record.has_value());
  EXPECT_EQ(record->crc_errors, 0);
  EXPECT_EQ(record->skipped_bytes, 0);

  ASSERT_EQ(record->imu.size(), 1u);
  EXPECT_FALSE(record->imu[0].magnetic_field_mgauss.has_value());
  EXPECT_TRUE(record->barometer.empty());
  EXPECT_EQ(record->imu[0].timestamp_ns, 100000);
  EXPECT_EQ(record->imu[0].linear_acceleration, Eigen::Vector3d(0.5, -1.0, 9.75));
  EXPECT_EQ(record->imu[0].angular_velocity, Eigen::Vector3d(0.125, -0.25, 0.0625));

  ASSERT_EQ(record->wheel_speed.size(), 1u);
  EXPECT_DOUBLE_EQ(record->wheel_speed[0].speed_mps, 5.0);

  ASSERT_EQ(record->gps.size(), 1u);
  EXPECT_EQ(record->gps[0].timestamp_ns, 200000);
  EXPECT_EQ(record->gps[0].fix_age_ms, 120);
  EXPECT_DOUBLE_EQ(record->gps[0].latitude_deg, 22.7878390123);
  EXPECT_DOUBLE_EQ(record->gps[0].longitude_deg, 113.5210145456);
  EXPECT_DOUBLE_EQ(record->gps[0].altitude_m, 13.5);
  EXPECT_DOUBLE_EQ(record->gps[0].ground_speed_mps, 0.25);
  EXPECT_EQ(record->gps[0].satellite_count, 31);
}

TEST(BinaryRecordReader, ResyncsAfterCorruption) {
  Bytes corrupted = kLongImuRecord;
  corrupted[20] ^= 0x01;
  const Bytes unknown_type = {0xA5, 0x5A, 0x07, 0x00};
  const Bytes truncated(kLongImuRecord.begin(), kLongImuRecord.begin() + 30);
  const std::optional<BinaryRecord> record =
      ReadBinaryRecord(WriteTempFile(File({corrupted, unknown_type, kLongImuRecord, truncated})));
  ASSERT_TRUE(record.has_value());
  EXPECT_EQ(record->imu.size(), 1u);
  EXPECT_EQ(record->crc_errors, 1);
  EXPECT_EQ(record->skipped_bytes,
            static_cast<int64_t>(corrupted.size() + unknown_type.size() + truncated.size()));
}

TEST(BinaryRecordReader, RejectsBadHeaderAndMissingFile) {
  Bytes wrong_version = File({kLongImuRecord});
  wrong_version[8] = 0x02;
  const std::string path = WriteTempFile(wrong_version);
  EXPECT_FALSE(IsBinaryRecord(path));
  EXPECT_FALSE(ReadBinaryRecord(path).has_value());
  EXPECT_FALSE(IsBinaryRecord("/nonexistent/record.bin"));
  EXPECT_FALSE(ReadBinaryRecord("/nonexistent/record.bin").has_value());
}

}  // namespace
}  // namespace mapmind::reader
