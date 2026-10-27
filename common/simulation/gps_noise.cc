// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/simulation/gps_noise.h"

#include <cmath>
#include <random>

#include "common/math/geodesy.h"

namespace mapmind::simulation {

namespace {

constexpr int kMaxSatellites = 30;
constexpr int kMinSatellites = 4;

int SatelliteCount(double horizontal_error) {
  int count = kMaxSatellites;
  while (count > kMinSatellites &&
         std::sqrt(sensor::GpsCovariance(count)(0, 0)) < horizontal_error) {
    --count;
  }
  return count;
}

}  // namespace

std::vector<sensor::GpsData> AddGpsNoise(const std::vector<reader::GpsPosition>& gps,
                                         const Eigen::Vector3d& origin,
                                         const GpsNoiseOptions& options, uint32_t seed) {
  const int zone = math::UtmZone(origin.y());
  const Eigen::Vector3d origin_utm = math::GeodeticToUtm(origin, zone);
  const Eigen::Vector3d walk(options.horizontal_walk, options.horizontal_walk,
                             options.vertical_walk);
  const Eigen::Vector3d white_std(options.horizontal_white_std, options.horizontal_white_std,
                                  options.vertical_white_std);
  std::mt19937 random(seed);
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform;
  const auto draw = [&] { return Eigen::Vector3d(normal(random), normal(random), normal(random)); };

  std::vector<sensor::GpsData> noisy;
  bool degraded =
      uniform(random) < options.mean_degraded_s / (options.mean_clear_s + options.mean_degraded_s);
  Eigen::Vector3d drift = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < gps.size(); ++i) {
    if (i > 0) {
      const double dt = 1e-9 * static_cast<double>(gps[i].timestamp_ns - gps[i - 1].timestamp_ns);
      const double mean_stay_s = degraded ? options.mean_degraded_s : options.mean_clear_s;
      if (uniform(random) > std::exp(-dt / mean_stay_s)) {
        degraded = !degraded;
      }
      if (degraded) {
        drift += std::sqrt(dt) * walk.cwiseProduct(draw());
      } else {
        drift *= std::exp(-dt / options.recovery_s);
      }
    }
    const Eigen::Vector3d geodetic = math::UtmToGeodetic(
        origin_utm + gps[i].position + drift + white_std.cwiseProduct(draw()), zone,
        /*southern=*/false);
    noisy.push_back({gps[i].timestamp_ns, 0, geodetic.x(), geodetic.y(), geodetic.z(), 0.0,
                     SatelliteCount(drift.head<2>().norm())});
  }
  return noisy;
}

}  // namespace mapmind::simulation
