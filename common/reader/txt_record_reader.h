// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_READER_TXT_RECORD_READER_H_
#define COMMON_READER_TXT_RECORD_READER_H_

#include <optional>
#include <string>
#include <vector>

#include "common/sensor/gps_data.h"
#include "common/sensor/imu_data.h"
#include "common/sensor/wheel_speed_data.h"

namespace mapmind::reader {

struct TxtRecord {
  std::vector<sensor::ImuData> imu;
  std::vector<sensor::GpsData> gps;
  std::vector<sensor::WheelSpeedData> wheel_speed;
  // lines that are truncated, unknown, or carry an invalid value
  int skipped_lines = 0;
};

// Reads a MOKUKU text record (one sample per line, timestamps in microseconds):
//   imu <t_us> <acc_x> <acc_y> <acc_z> <gyr_x> <gyr_y> <gyr_z>   (m/s^2, rad/s, body frame)
//   gps <t_us> <status> <lat_deg> <lon_deg> <alt_m> <ground_speed_mps> <satellites>
//   speed <t_us> <wheel_speed_kmh>                               (255 = invalid)
// Returns nullopt if the file cannot be opened.
std::optional<TxtRecord> ReadTxtRecord(const std::string& path);

// Removes corrupted IMU samples: any accelerometer axis beyond the sensor range, or a channel that
// spikes against both neighbours (the recorder occasionally writes garbage values for one sample).
// Returns the number of removed samples.
int RemoveCorruptImuSamples(std::vector<sensor::ImuData>* imu);

}  // namespace mapmind::reader

#endif  // COMMON_READER_TXT_RECORD_READER_H_
