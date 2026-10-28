// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/simulation/gps_noise.h"

#include <cmath>

#include <gtest/gtest.h>
#include "common/math/geodesy.h"
#include "common/sensor/gps_data.h"

namespace mapmind::simulation {
namespace {

const Eigen::Vector3d kOrigin(22.787839, 113.5210145, 13.0);

// m, world-frame (UTM offset from kOrigin) error of the fixes of a static receiver at the origin
std::vector<Eigen::Vector3d> Errors(const std::vector<sensor::GpsData>& gps) {
  const int zone = math::UtmZone(kOrigin.y());
  const Eigen::Vector3d origin_utm = math::GeodeticToUtm(kOrigin, zone);
  std::vector<Eigen::Vector3d> errors;
  for (const sensor::GpsData& fix : gps) {
    errors.push_back(
        math::GeodeticToUtm({fix.latitude_deg, fix.longitude_deg, fix.altitude_m}, zone) -
        origin_utm);
  }
  return errors;
}

// 1 Hz fixes at the origin for a day
std::vector<reader::GpsPosition> StaticGps() {
  std::vector<reader::GpsPosition> gps;
  for (int64_t i = 0; i < 86400; ++i) {
    gps.push_back({i * 1'000'000'000, Eigen::Vector3d::Zero()});
  }
  return gps;
}

TEST(GpsNoise, DriftAccumulatesAndRecovers) {
  const std::vector<Eigen::Vector3d> errors =
      Errors(AddGpsNoise(StaticGps(), kOrigin, GpsNoiseOptions{}, 1));
  int small_count = 0;
  int large_count = 0;
  // stretches that pass 1 m horizontal error and later drop below 0.15 m again
  int recovery_count = 0;
  bool large = false;
  Eigen::Vector3d sum_squares = Eigen::Vector3d::Zero();
  Eigen::Vector3d step_sum_squares = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < errors.size(); ++i) {
    const Eigen::Vector3d& error = errors[i];
    const double horizontal = error.head<2>().norm();
    small_count += horizontal < 0.15;
    large_count += horizontal > 1.0;
    if (horizontal > 1.0) {
      large = true;
    } else if (large && horizontal < 0.15) {
      large = false;
      ++recovery_count;
    }
    sum_squares += error.cwiseAbs2();
    if (i > 0) {
      step_sum_squares += (error - errors[i - 1]).cwiseAbs2();
    }
  }
  const Eigen::Vector3d error_std = (sum_squares / errors.size()).cwiseSqrt();
  const Eigen::Vector3d step_std = (step_sum_squares / (errors.size() - 1)).cwiseSqrt();
  // not everywhere large: GPS is good part of the time, bad part of the time, and recovers
  EXPECT_GT(small_count, errors.size() / 3);
  EXPECT_GT(large_count, errors.size() / 20);
  EXPECT_GT(recovery_count, 50);
  // accumulated: 1 s apart the errors differ far less than independent errors would
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_LT(step_std[axis], 0.5 * error_std[axis]) << axis;
  }
  EXPECT_GT(error_std.z(), 1.3 * error_std.head<2>().maxCoeff());
}

TEST(GpsNoise, SatelliteCountFollowsError) {
  const std::vector<sensor::GpsData> noisy =
      AddGpsNoise(StaticGps(), kOrigin, GpsNoiseOptions{}, 1);
  const std::vector<Eigen::Vector3d> errors = Errors(noisy);
  int fix_count = 0;
  int single_count = 0;
  int outside_count = 0;
  for (size_t i = 0; i < noisy.size(); ++i) {
    fix_count += noisy[i].satellite_count >= 21;
    single_count += noisy[i].satellite_count <= 10;
    const double horizontal_std = std::sqrt(sensor::GpsCovariance(noisy[i].satellite_count)(0, 0));
    outside_count += errors[i].head<2>().norm() > 3.0 * horizontal_std;
  }
  EXPECT_GT(fix_count, noisy.size() / 5);
  EXPECT_GT(single_count, noisy.size() / 20);
  // the covariance of the reported count covers the error
  EXPECT_LT(outside_count, noisy.size() / 50);
}

TEST(GpsNoise, KeepsTimestampsAndRepeatsWithSeed) {
  std::vector<reader::GpsPosition> gps = StaticGps();
  gps[10].position = Eigen::Vector3d(100.0, -200.0, 5.0);
  const std::vector<sensor::GpsData> first = AddGpsNoise(gps, kOrigin, GpsNoiseOptions{}, 7);
  const std::vector<sensor::GpsData> second = AddGpsNoise(gps, kOrigin, GpsNoiseOptions{}, 7);
  const std::vector<sensor::GpsData> other = AddGpsNoise(gps, kOrigin, GpsNoiseOptions{}, 8);
  ASSERT_EQ(first.size(), gps.size());
  EXPECT_EQ(first[10].timestamp_ns, gps[10].timestamp_ns);
  // the world frame is the UTM grid at kOrigin
  EXPECT_LT((Errors(first)[10] - gps[10].position).norm(), 1.0);
  EXPECT_EQ(first[10].latitude_deg, second[10].latitude_deg);
  EXPECT_NE(first[10].latitude_deg, other[10].latitude_deg);
}

}  // namespace
}  // namespace mapmind::simulation
