// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/math/geodesy.h"

#include <math.h>

#define SEMI_MAJOR_AXIS 6378137.0
#define FLATTENING (1.0 / 298.257223563)
#define SCALE_FACTOR 0.9996
#define FALSE_EASTING 500000.0
#define SOUTHERN_FALSE_NORTHING 10000000.0
#define N (FLATTENING / (2.0 - FLATTENING))
#define RECTIFYING_RADIUS (SEMI_MAJOR_AXIS / (1.0 + N) * (1.0 + N * N / 4.0 + N * N * N * N / 64.0))
#define PI 3.14159265358979323846

static double ToRadians(double degrees) { return degrees * PI / 180.0; }
static double ToDegrees(double radians) { return radians * 180.0 / PI; }

int utm_zone(double longitude_deg) {
  const int zone = (int)floor((longitude_deg + 180.0) / 6.0) + 1;
  return zone > 60 ? 1 : zone;
}

// Krueger series to third order in n, as in
// https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
void geodetic_to_utm(const double geodetic[3], int zone, double utm[3]) {
  const double alpha[3] = {N / 2.0 - 2.0 * N * N / 3.0 + 5.0 * N * N * N / 16.0,
                           13.0 * N * N / 48.0 - 3.0 * N * N * N / 5.0, 61.0 * N * N * N / 240.0};
  const double eccentricity_term = 2.0 * sqrt(N) / (1.0 + N);

  const double latitude = ToRadians(geodetic[0]);
  const double longitude_offset = ToRadians(geodetic[1] - (6.0 * zone - 183.0));
  const double sin_latitude = sin(latitude);
  const double t =
      sinh(atanh(sin_latitude) - eccentricity_term * atanh(eccentricity_term * sin_latitude));
  const double xi = atan2(t, cos(longitude_offset));
  const double eta = atanh(sin(longitude_offset) / sqrt(1.0 + t * t));

  double easting = eta;
  double northing = xi;
  for (int j = 1; j <= 3; ++j) {
    easting += alpha[j - 1] * cos(2.0 * j * xi) * sinh(2.0 * j * eta);
    northing += alpha[j - 1] * sin(2.0 * j * xi) * cosh(2.0 * j * eta);
  }
  const double false_northing = geodetic[0] < 0.0 ? SOUTHERN_FALSE_NORTHING : 0.0;
  const double altitude = geodetic[2];
  utm[0] = FALSE_EASTING + SCALE_FACTOR * RECTIFYING_RADIUS * easting;
  utm[1] = false_northing + SCALE_FACTOR * RECTIFYING_RADIUS * northing;
  utm[2] = altitude;
}

void utm_to_geodetic(const double utm[3], int zone, bool southern, double geodetic[3]) {
  const double beta[3] = {N / 2.0 - 2.0 * N * N / 3.0 + 37.0 * N * N * N / 96.0,
                          N * N / 48.0 + N * N * N / 15.0, 17.0 * N * N * N / 480.0};
  const double delta[3] = {2.0 * N - 2.0 * N * N / 3.0 - 2.0 * N * N * N,
                           7.0 * N * N / 3.0 - 8.0 * N * N * N / 5.0, 56.0 * N * N * N / 15.0};
  const double false_northing = southern ? SOUTHERN_FALSE_NORTHING : 0.0;
  const double xi = (utm[1] - false_northing) / (SCALE_FACTOR * RECTIFYING_RADIUS);
  const double eta = (utm[0] - FALSE_EASTING) / (SCALE_FACTOR * RECTIFYING_RADIUS);

  double xi_prime = xi;
  double eta_prime = eta;
  for (int j = 1; j <= 3; ++j) {
    xi_prime -= beta[j - 1] * sin(2.0 * j * xi) * cosh(2.0 * j * eta);
    eta_prime -= beta[j - 1] * cos(2.0 * j * xi) * sinh(2.0 * j * eta);
  }
  const double chi = asin(sin(xi_prime) / cosh(eta_prime));
  double latitude = chi;
  for (int j = 1; j <= 3; ++j) {
    latitude += delta[j - 1] * sin(2.0 * j * chi);
  }
  const double longitude_offset = atan2(sinh(eta_prime), cos(xi_prime));
  const double altitude = utm[2];
  geodetic[0] = ToDegrees(latitude);
  geodetic[1] = 6.0 * zone - 183.0 + ToDegrees(longitude_offset);
  geodetic[2] = altitude;
}
