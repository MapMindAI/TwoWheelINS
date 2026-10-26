// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FUSION_GPS_COVARIANCE_H_
#define FUSION_GPS_COVARIANCE_H_

#include <Eigen/Core>

namespace mapmind::fusion {

// GPS position covariance (UTM easting, northing, altitude; m^2) for a fix from
// `satellite_count` satellites: > 20 is a fix solution, 11-20 float, fewer single.
Eigen::Matrix3d GpsCovariance(int satellite_count);

}  // namespace mapmind::fusion

#endif  // FUSION_GPS_COVARIANCE_H_
