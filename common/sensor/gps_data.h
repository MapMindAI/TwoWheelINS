// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_SENSOR_GPS_DATA_H_
#define COMMON_SENSOR_GPS_DATA_H_

#include <cstdint>

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
};

}  // namespace mapmind::sensor

#endif  // COMMON_SENSOR_GPS_DATA_H_
