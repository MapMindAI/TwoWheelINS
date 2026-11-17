// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_SENSOR_GPS_ACCURACY_H_
#define COMMON_SENSOR_GPS_ACCURACY_H_

#ifdef __cplusplus
extern "C" {
#endif

// GPS position standard deviation (m) along each horizontal UTM axis and in altitude.
typedef struct {
  double horizontal;
  double vertical;
} GpsPositionStd;

// For a fix from `satellite_count` satellites: > 20 is a fix solution, 11-20 float, fewer single.
GpsPositionStd gps_position_std(int satellite_count);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_SENSOR_GPS_ACCURACY_H_
