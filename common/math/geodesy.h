// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_MATH_GEODESY_H_
#define COMMON_MATH_GEODESY_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// UTM zone number (1-60) containing `longitude_deg`; ignores the Norway / Svalbard exceptions.
int utm_zone(double longitude_deg);

// WGS84 geodetic (latitude_deg, longitude_deg, altitude_m) to UTM (easting_m, northing_m,
// altitude_m) in `zone`. Southern latitudes use the 10000 km false northing.
void geodetic_to_utm(const double geodetic[3], int zone, double utm[3]);

// Inverse of geodetic_to_utm; `southern` selects the 10000 km false northing.
void utm_to_geodetic(const double utm[3], int zone, bool southern, double geodetic[3]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_MATH_GEODESY_H_
