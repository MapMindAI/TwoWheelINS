// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_MATH_GEODESY_H_
#define COMMON_MATH_GEODESY_H_

#include <Eigen/Core>

namespace mapmind::math {

// UTM zone number (1-60) containing `longitude_deg`; ignores the Norway / Svalbard exceptions.
int UtmZone(double longitude_deg);

// WGS84 geodetic (latitude_deg, longitude_deg, altitude_m) to UTM (easting_m, northing_m,
// altitude_m) in `zone`. Southern latitudes use the 10000 km false northing.
Eigen::Vector3d GeodeticToUtm(const Eigen::Vector3d& geodetic, int zone);

// Inverse of GeodeticToUtm; `southern` selects the 10000 km false northing.
Eigen::Vector3d UtmToGeodetic(const Eigen::Vector3d& utm, int zone, bool southern);

}  // namespace mapmind::math

#endif  // COMMON_MATH_GEODESY_H_
