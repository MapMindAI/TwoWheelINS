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

constexpr double kN = kFlattening / (2.0 - kFlattening);
constexpr double kN2 = kN * kN;
constexpr double kN3 = kN2 * kN;
constexpr double kRectifyingRadius =
    kSemiMajorAxis / (1.0 + kN) * (1.0 + kN2 / 4.0 + kN2 * kN2 / 64.0);

double ToRadians(double degrees) { return degrees * M_PI / 180.0; }
double ToDegrees(double radians) { return radians * 180.0 / M_PI; }

}  // namespace

int UtmZone(double longitude_deg) {
  const int zone = static_cast<int>(std::floor((longitude_deg + 180.0) / 6.0)) + 1;
  return zone > 60 ? 1 : zone;
}

// Krueger series to third order in n, as in
// https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
Eigen::Vector3d GeodeticToUtm(const Eigen::Vector3d& geodetic, int zone) {
  constexpr double alpha[3] = {kN / 2.0 - 2.0 * kN2 / 3.0 + 5.0 * kN3 / 16.0,
                               13.0 * kN2 / 48.0 - 3.0 * kN3 / 5.0, 61.0 * kN3 / 240.0};
  const double eccentricity_term = 2.0 * std::sqrt(kN) / (1.0 + kN);

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
  return {kFalseEasting + kScaleFactor * kRectifyingRadius * easting,
          false_northing + kScaleFactor * kRectifyingRadius * northing, geodetic.z()};
}

Eigen::Vector3d UtmToGeodetic(const Eigen::Vector3d& utm, int zone, bool southern) {
  constexpr double beta[3] = {kN / 2.0 - 2.0 * kN2 / 3.0 + 37.0 * kN3 / 96.0,
                              kN2 / 48.0 + kN3 / 15.0, 17.0 * kN3 / 480.0};
  constexpr double delta[3] = {2.0 * kN - 2.0 * kN2 / 3.0 - 2.0 * kN3,
                               7.0 * kN2 / 3.0 - 8.0 * kN3 / 5.0, 56.0 * kN3 / 15.0};
  const double false_northing = southern ? kSouthernFalseNorthing : 0.0;
  const double xi = (utm.y() - false_northing) / (kScaleFactor * kRectifyingRadius);
  const double eta = (utm.x() - kFalseEasting) / (kScaleFactor * kRectifyingRadius);

  double xi_prime = xi;
  double eta_prime = eta;
  for (int j = 1; j <= 3; ++j) {
    xi_prime -= beta[j - 1] * std::sin(2.0 * j * xi) * std::cosh(2.0 * j * eta);
    eta_prime -= beta[j - 1] * std::cos(2.0 * j * xi) * std::sinh(2.0 * j * eta);
  }
  const double chi = std::asin(std::sin(xi_prime) / std::cosh(eta_prime));
  double latitude = chi;
  for (int j = 1; j <= 3; ++j) {
    latitude += delta[j - 1] * std::sin(2.0 * j * chi);
  }
  const double longitude_offset = std::atan2(std::sinh(eta_prime), std::cos(xi_prime));
  return {ToDegrees(latitude), 6.0 * zone - 183.0 + ToDegrees(longitude_offset), utm.z()};
}

}  // namespace mapmind::math
