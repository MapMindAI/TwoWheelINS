// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_READER_BINARY_RECORD_READER_H_
#define COMMON_READER_BINARY_RECORD_READER_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "common/sensor/barometer_data.h"
#include "common/sensor/gps_data.h"
#include "common/sensor/imu_data.h"
#include "common/sensor/wheel_speed_data.h"

namespace mapmind::reader {

struct BinaryRecord {
  std::vector<sensor::ImuData> imu;
  // temperature and pressure of long IMU records
  std::vector<sensor::BarometerData> barometer;
  std::vector<sensor::GpsData> gps;
  std::vector<sensor::WheelSpeedData> wheel_speed;
  // records with a valid frame whose CRC does not match
  int crc_errors = 0;
  // bytes after the header that are not part of a valid record
  int64_t skipped_bytes = 0;
};

// True if `path` starts with the binary record header (magic and version).
bool IsBinaryRecord(const std::string& path);

// Reads a binary record (doc/binary_record_format.md). Returns nullopt if the file cannot be
// opened or its header does not match.
std::optional<BinaryRecord> ReadBinaryRecord(const std::string& path);

}  // namespace mapmind::reader

#endif  // COMMON_READER_BINARY_RECORD_READER_H_
