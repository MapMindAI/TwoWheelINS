// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/sensor/gps_accuracy.h"

// GPS fix status from the satellite count (the receiver's own status flag is not used).
enum { kMinFixSatellites = 21, kMinFloatSatellites = 11 };
// RTK-style accuracy for fix and float solutions (m).
static const GpsPositionStd kFixStd = {0.05, 0.1};
static const GpsPositionStd kFloatStd = {0.5, 1.0};
// Single solutions: std = scale / (satellites - 3). Fix-to-fix scatter in mobili_2 falls this way
// with the satellite count; the scale keeps 1.5 m / 10 m at the median of 8 satellites.
static const GpsPositionStd kSingleStdScale = {7.5, 50.0};

GpsPositionStd gps_position_std(int satellite_count) {
  if (satellite_count >= kMinFixSatellites) {
    return kFixStd;
  }
  if (satellite_count >= kMinFloatSatellites) {
    return kFloatStd;
  }
  const double redundancy = satellite_count > 4 ? satellite_count - 3 : 1;
  const GpsPositionStd std = {kSingleStdScale.horizontal / redundancy,
                              kSingleStdScale.vertical / redundancy};
  return std;
}
