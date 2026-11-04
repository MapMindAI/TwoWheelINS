// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/reader/binary_record_reader.h"

#include <cstring>
#include <fstream>
#include <iterator>

namespace mapmind::reader {

namespace {

constexpr char kMagic[] = {'M', 'O', 'K', 'U', 'R', 'E', 'C', '\0'};
constexpr uint8_t kVersion = 1;
constexpr size_t kHeaderSize = sizeof(kMagic) + 1;
constexpr uint8_t kSync0 = 0xA5;
constexpr uint8_t kSync1 = 0x5A;
// sync (2) + type (1) + timestamp (8)
constexpr size_t kPayloadOffset = 11;
constexpr size_t kCrcSize = 2;

enum RecordType : uint8_t { kImu = 0x01, kSpeed = 0x02, kGps = 0x03, kLongImu = 0x04 };

constexpr int64_t kNanosecondsPerMicrosecond = 1000;
constexpr int32_t kInvalidWheelSpeedKmh = 255;
constexpr double kKmhToMps = 1.0 / 3.6;

// Total record size for `type`, 0 if unknown.
size_t RecordSize(uint8_t type) {
  switch (type) {
    case kImu:
      return 37;
    case kSpeed:
      return 17;
    case kGps:
      return 42;
    case kLongImu:
      return 57;
    default:
      return 0;
  }
}

uint64_t LittleEndian(const uint8_t* bytes, int size) {
  uint64_t value = 0;
  for (int i = size - 1; i >= 0; --i) {
    value = (value << 8) | bytes[i];
  }
  return value;
}

float Float32(const uint8_t* bytes) {
  const uint32_t bits = static_cast<uint32_t>(LittleEndian(bytes, 4));
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

double Float64(const uint8_t* bytes) {
  const uint64_t bits = LittleEndian(bytes, 8);
  double value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

// CRC-16/CCITT-FALSE.
uint16_t Crc16(const uint8_t* bytes, size_t size) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(bytes[i] << 8);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

bool HasHeader(const std::vector<uint8_t>& bytes) {
  return bytes.size() >= kHeaderSize && std::memcmp(bytes.data(), kMagic, sizeof(kMagic)) == 0 &&
         bytes[sizeof(kMagic)] == kVersion;
}

// Decodes one CRC-checked record starting at `record`.
void Decode(const uint8_t* record, BinaryRecord* out) {
  const uint8_t type = record[2];
  const int64_t timestamp_ns =
      static_cast<int64_t>(LittleEndian(record + 3, 8)) * kNanosecondsPerMicrosecond;
  const uint8_t* payload = record + kPayloadOffset;
  switch (type) {
    case kImu:
    case kLongImu: {
      sensor::ImuData imu{timestamp_ns, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
      for (int axis = 0; axis < 3; ++axis) {
        imu.linear_acceleration[axis] = Float32(payload + 4 * axis);
        imu.angular_velocity[axis] = Float32(payload + 12 + 4 * axis);
      }
      if (type == kLongImu) {
        imu.magnetic_field_mgauss =
            Eigen::Vector3d(Float32(payload + 24), Float32(payload + 28), Float32(payload + 32));
        out->barometer.push_back({timestamp_ns, Float32(payload + 36), Float32(payload + 40)});
      }
      out->imu.push_back(imu);
      break;
    }
    case kSpeed: {
      const int32_t speed_kmh = static_cast<int32_t>(LittleEndian(payload, 4));
      if (speed_kmh != kInvalidWheelSpeedKmh) {
        out->wheel_speed.push_back({timestamp_ns, speed_kmh * kKmhToMps});
      }
      break;
    }
    case kGps: {
      sensor::GpsData gps{timestamp_ns, 0, 0.0, 0.0, 0.0, 0.0, 0};
      gps.fix_age_ms = static_cast<int64_t>(LittleEndian(payload, 4));
      gps.latitude_deg = Float64(payload + 4);
      gps.longitude_deg = Float64(payload + 12);
      gps.altitude_m = Float32(payload + 20);
      gps.ground_speed_mps = Float32(payload + 24);
      gps.satellite_count = payload[28];
      out->gps.push_back(gps);
      break;
    }
    default:
      break;
  }
}

std::optional<std::vector<uint8_t>> ReadBytes(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    return std::nullopt;
  }
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(file),
                              std::istreambuf_iterator<char>());
}

}  // namespace

bool IsBinaryRecord(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  std::vector<uint8_t> header(kHeaderSize);
  file.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(kHeaderSize));
  return file.gcount() == static_cast<std::streamsize>(kHeaderSize) && HasHeader(header);
}

std::optional<BinaryRecord> ReadBinaryRecord(const std::string& path) {
  const std::optional<std::vector<uint8_t>> bytes = ReadBytes(path);
  if (!bytes || !HasHeader(*bytes)) {
    return std::nullopt;
  }
  BinaryRecord record;
  const uint8_t* data = bytes->data();
  const size_t size = bytes->size();
  size_t offset = kHeaderSize;
  while (offset < size) {
    const size_t record_size =
        offset + 2 < size && data[offset] == kSync0 && data[offset + 1] == kSync1
            ? RecordSize(data[offset + 2])
            : 0;
    if (record_size == 0 || offset + record_size > size) {
      ++offset;
      ++record.skipped_bytes;
      continue;
    }
    const uint8_t* frame = data + offset;
    const size_t crc_offset = record_size - kCrcSize;
    if (Crc16(frame + 2, crc_offset - 2) != LittleEndian(frame + crc_offset, kCrcSize)) {
      ++record.crc_errors;
      ++offset;
      ++record.skipped_bytes;
      continue;
    }
    Decode(frame, &record);
    offset += record_size;
  }
  return record;
}

}  // namespace mapmind::reader
