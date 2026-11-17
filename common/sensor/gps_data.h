// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_SENSOR_GPS_DATA_H_
#define COMMON_SENSOR_GPS_DATA_H_

#include <cstdint>
#include <limits>

namespace mapmind::sensor {

struct GpsData {
  int64_t timestamp_ns;
  // receiver-specific status flag, passed through unmodified
  int status;
  double latitude_deg;
  double longitude_deg;
  double altitude_m;
  double ground_speed_mps;
  int satellite_count;
  // age of the fix at timestamp_ns (binary records); 0 in text records
  int64_t fix_age_ms = 0;
  // long GPS records only, else NaN; course is clockwise from true north
  double hdop = std::numeric_limits<double>::quiet_NaN();
  double course_deg = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace mapmind::sensor

#endif  // COMMON_SENSOR_GPS_DATA_H_
