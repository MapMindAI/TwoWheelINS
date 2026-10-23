// Copyright 2026 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#include "common/reader/txt_record_reader.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace mapmind::reader {

namespace {

constexpr int64_t kNanosecondsPerMicrosecond = 1000;
constexpr double kInvalidWheelSpeedKmh = 255.0;
constexpr double kKmhToMps = 1.0 / 3.6;

constexpr double kAccelerometerRange = 50.0;  // m/s^2
constexpr double kAccelerometerSpike = 15.0;  // m/s^2
constexpr double kGyroscopeSpike = 0.4;       // rad/s

// True if every field parsed and nothing but whitespace follows.
bool FullyParsed(std::istringstream& stream) {
  if (stream.fail()) {
    return false;
  }
  std::string rest;
  return !(stream >> rest);
}

bool IsSpike(double previous, double current, double next, double threshold) {
  const double to_previous = current - previous;
  const double to_next = current - next;
  return std::abs(to_previous) > threshold && std::abs(to_next) > threshold &&
         (to_previous > 0) == (to_next > 0);
}

bool IsSpike(const Eigen::Vector3d& previous, const Eigen::Vector3d& current,
             const Eigen::Vector3d& next, double threshold) {
  for (int axis = 0; axis < 3; ++axis) {
    if (IsSpike(previous[axis], current[axis], next[axis], threshold)) {
      return true;
    }
  }
  return false;
}

}  // namespace

std::optional<TxtRecord> ReadTxtRecord(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    return std::nullopt;
  }
  TxtRecord record;
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream stream(line);
    std::string type;
    int64_t timestamp_us = 0;
    stream >> type >> timestamp_us;
    const int64_t timestamp_ns = timestamp_us * kNanosecondsPerMicrosecond;
    if (type == "imu") {
      sensor::ImuData imu{timestamp_ns, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
      stream >> imu.linear_acceleration.x() >> imu.linear_acceleration.y() >>
          imu.linear_acceleration.z() >> imu.angular_velocity.x() >> imu.angular_velocity.y() >>
          imu.angular_velocity.z();
      if (FullyParsed(stream)) {
        record.imu.push_back(imu);
        continue;
      }
    } else if (type == "gps") {
      sensor::GpsData gps{timestamp_ns, 0, 0.0, 0.0, 0.0, 0.0, 0};
      stream >> gps.status >> gps.latitude_deg >> gps.longitude_deg >> gps.altitude_m >>
          gps.ground_speed_mps >> gps.satellite_count;
      if (FullyParsed(stream)) {
        record.gps.push_back(gps);
        continue;
      }
    } else if (type == "speed") {
      double speed_kmh = 0.0;
      stream >> speed_kmh;
      if (FullyParsed(stream) && speed_kmh != kInvalidWheelSpeedKmh) {
        record.wheel_speed.push_back({timestamp_ns, speed_kmh * kKmhToMps});
        continue;
      }
    }
    ++record.skipped_lines;
  }
  return record;
}

int RemoveCorruptImuSamples(std::vector<sensor::ImuData>* imu) {
  const size_t original_size = imu->size();
  std::vector<sensor::ImuData> in_range;
  in_range.reserve(imu->size());
  for (const sensor::ImuData& sample : *imu) {
    if (sample.linear_acceleration.cwiseAbs().maxCoeff() <= kAccelerometerRange) {
      in_range.push_back(sample);
    }
  }

  imu->clear();
  for (size_t i = 0; i < in_range.size(); ++i) {
    if (i > 0 && i + 1 < in_range.size()) {
      const sensor::ImuData& previous = in_range[i - 1];
      const sensor::ImuData& current = in_range[i];
      const sensor::ImuData& next = in_range[i + 1];
      if (IsSpike(previous.linear_acceleration, current.linear_acceleration,
                  next.linear_acceleration, kAccelerometerSpike) ||
          IsSpike(previous.angular_velocity, current.angular_velocity, next.angular_velocity,
                  kGyroscopeSpike)) {
        continue;
      }
    }
    imu->push_back(in_range[i]);
  }
  return static_cast<int>(original_size - imu->size());
}

}  // namespace mapmind::reader
