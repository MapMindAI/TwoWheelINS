// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/sensor/gps_data.h"

#include <algorithm>

namespace mapmind::sensor {

namespace {

// GPS fix status from the satellite count (the receiver's own status flag is not used).
constexpr int kMinFixSatellites = 21;
constexpr int kMinFloatSatellites = 11;
// RTK-style accuracy for fix and float solutions (m).
constexpr double kGpsFixHorizontalStd = 0.05;
constexpr double kGpsFixVerticalStd = 0.1;
constexpr double kGpsFloatHorizontalStd = 0.5;
constexpr double kGpsFloatVerticalStd = 1.0;
// Single solutions: std = scale / (satellites - 3). Fix-to-fix scatter in mobili_2 falls this way
// with the satellite count; the scale keeps 1.5 m / 10 m at the median of 8 satellites.
constexpr double kGpsSingleHorizontalStdScale = 7.5;  // m
constexpr double kGpsSingleVerticalStdScale = 50.0;   // m

}  // namespace

Eigen::Matrix3d GpsCovariance(int satellite_count) {
  double horizontal = kGpsFixHorizontalStd;
  double vertical = kGpsFixVerticalStd;
  if (satellite_count < kMinFloatSatellites) {
    const double redundancy = std::max(satellite_count - 3, 1);
    horizontal = kGpsSingleHorizontalStdScale / redundancy;
    vertical = kGpsSingleVerticalStdScale / redundancy;
  } else if (satellite_count < kMinFixSatellites) {
    horizontal = kGpsFloatHorizontalStd;
    vertical = kGpsFloatVerticalStd;
  }
  return Eigen::Vector3d(horizontal, horizontal, vertical).cwiseAbs2().asDiagonal();
}

}  // namespace mapmind::sensor
