// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_SIMULATION_GPS_NOISE_H_
#define COMMON_SIMULATION_GPS_NOISE_H_

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "common/reader/benchmark_dataset_reader.h"
#include "common/sensor/gps_data.h"

namespace mapmind::simulation {

// GPS error per world axis: white noise plus a drift. The receiver alternates between clear and
// degraded stretches of random length; while degraded the drift accumulates as a random walk,
// while clear it decays back to zero. The satellite count follows the drift.
struct GpsNoiseOptions {
  // s, mean length of the clear and degraded stretches (exponentially distributed)
  double mean_clear_s = 30.0;
  double mean_degraded_s = 20.0;
  // m/sqrt(s), random-walk rate of the drift while degraded
  double horizontal_walk = 0.2;
  double vertical_walk = 0.3;
  // s, time constant of the drift decay while clear
  double recovery_s = 10.0;
  // m, std of the white noise (RTK fix level)
  double horizontal_white_std = 0.03;
  double vertical_white_std = 0.06;
};

// Returns `gps` with the noise of `options` added, as WGS84 fixes: world x, y, z are UTM easting,
// northing and altitude offsets from `origin` (latitude_deg, longitude_deg, altitude_m; northern
// hemisphere). Each fix gets the highest satellite count whose sensor::GpsCovariance horizontal std
// covers the horizontal drift. The same `seed` gives the same noise.
std::vector<sensor::GpsData> AddGpsNoise(const std::vector<reader::GpsPosition>& gps,
                                         const Eigen::Vector3d& origin,
                                         const GpsNoiseOptions& options, uint32_t seed);

}  // namespace mapmind::simulation

#endif  // COMMON_SIMULATION_GPS_NOISE_H_
