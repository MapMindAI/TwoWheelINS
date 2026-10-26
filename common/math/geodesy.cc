// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/math/geodesy.h"

#include <cmath>

namespace mapmind::math {

namespace {

constexpr double kSemiMajorAxis = 6378137.0;
constexpr double kFlattening = 1.0 / 298.257223563;
constexpr double kScaleFactor = 0.9996;
constexpr double kFalseEasting = 500000.0;
constexpr double kSouthernFalseNorthing = 10000000.0;

double ToRadians(double degrees) { return degrees * M_PI / 180.0; }

}  // namespace

int UtmZone(double longitude_deg) {
  const int zone = static_cast<int>(std::floor((longitude_deg + 180.0) / 6.0)) + 1;
  return zone > 60 ? 1 : zone;
}

// Krueger series to third order in n, as in
// https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
Eigen::Vector3d GeodeticToUtm(const Eigen::Vector3d& geodetic, int zone) {
  constexpr double n = kFlattening / (2.0 - kFlattening);
  constexpr double n2 = n * n;
  constexpr double n3 = n2 * n;
  constexpr double rectifying_radius =
      kSemiMajorAxis / (1.0 + n) * (1.0 + n2 / 4.0 + n2 * n2 / 64.0);
  constexpr double alpha[3] = {n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0,
                               13.0 * n2 / 48.0 - 3.0 * n3 / 5.0, 61.0 * n3 / 240.0};
  const double eccentricity_term = 2.0 * std::sqrt(n) / (1.0 + n);

  const double latitude = ToRadians(geodetic.x());
  const double longitude_offset = ToRadians(geodetic.y() - (6.0 * zone - 183.0));
  const double sin_latitude = std::sin(latitude);
  const double t = std::sinh(std::atanh(sin_latitude) -
                             eccentricity_term * std::atanh(eccentricity_term * sin_latitude));
  const double xi = std::atan2(t, std::cos(longitude_offset));
  const double eta = std::atanh(std::sin(longitude_offset) / std::sqrt(1.0 + t * t));

  double easting = eta;
  double northing = xi;
  for (int j = 1; j <= 3; ++j) {
    easting += alpha[j - 1] * std::cos(2.0 * j * xi) * std::sinh(2.0 * j * eta);
    northing += alpha[j - 1] * std::sin(2.0 * j * xi) * std::cosh(2.0 * j * eta);
  }
  const double false_northing = geodetic.x() < 0.0 ? kSouthernFalseNorthing : 0.0;
  return {kFalseEasting + kScaleFactor * rectifying_radius * easting,
          false_northing + kScaleFactor * rectifying_radius * northing, geodetic.z()};
}

}  // namespace mapmind::math
