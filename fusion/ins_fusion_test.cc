// Copyright 2026 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#include "fusion/ins_fusion.h"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include "common/math/geodesy.h"

namespace mapmind::fusion {
namespace {

constexpr double kGravity = 9.8099;
constexpr int64_t kImuPeriodNs = 10'000'000;  // 100 Hz
constexpr int kWheelEvery = 10;               // 10 Hz
constexpr int kGpsEvery = 100;                // 1 Hz
// Yaw hypotheses start at 6 s (two fixes at >= 3 m/s) and one is selected 10 fixes later.
constexpr double kSettledSeconds = 16.0;

// Planar ride: 2 s static, accelerate at 1 m/s^2 for 5 s, then turn at 0.2 rad/s.
struct Motion {
  double speed;
  double acceleration;
  double yaw_rate;
};

Motion MotionAt(double t) {
  if (t < 2.0) {
    return {0.0, 0.0, 0.0};
  }
  if (t < 7.0) {
    return {t - 2.0, 1.0, 0.0};
  }
  return {5.0, 0.0, 0.2};
}

// Geodetic position of a UTM offset from `origin`, by inverting the GeodeticToUtm Jacobian
// (mm-level over the test's few hundred metres).
Eigen::Vector3d UtmOffsetToGeodetic(const Eigen::Vector3d& offset, const Eigen::Vector3d& origin,
                                    int zone) {
  constexpr double kStep = 1e-4;
  const Eigen::Vector3d origin_utm = math::GeodeticToUtm(origin, zone);
  Eigen::Matrix3d jacobian;
  for (int i = 0; i < 3; ++i) {
    jacobian.col(i) =
        (math::GeodeticToUtm(origin + kStep * Eigen::Vector3d::Unit(i), zone) - origin_utm) / kStep;
  }
  return origin + jacobian.inverse() * offset;
}

TEST(GpsCovariance, ShrinksWithSatelliteCount) {
  // single solutions
  EXPECT_DOUBLE_EQ(GpsCovariance(8)(0, 0), 1.5 * 1.5);
  EXPECT_DOUBLE_EQ(GpsCovariance(8)(1, 1), 1.5 * 1.5);
  EXPECT_DOUBLE_EQ(GpsCovariance(8)(2, 2), 10.0 * 10.0);
  EXPECT_DOUBLE_EQ(GpsCovariance(4)(0, 0), 7.5 * 7.5);
  EXPECT_DOUBLE_EQ(GpsCovariance(10)(2, 2), (50.0 / 7) * (50.0 / 7));
  EXPECT_TRUE(GpsCovariance(0).allFinite());
  // float solutions
  EXPECT_DOUBLE_EQ(GpsCovariance(11)(0, 0), 0.5 * 0.5);
  EXPECT_DOUBLE_EQ(GpsCovariance(20)(2, 2), 1.0 * 1.0);
  // fix solutions
  EXPECT_DOUBLE_EQ(GpsCovariance(21)(1, 1), 0.05 * 0.05);
  EXPECT_DOUBLE_EQ(GpsCovariance(30)(2, 2), 0.1 * 0.1);
}

TEST(InsFusion, TracksSimulatedRide) {
  // arbitrary tilted IMU mount (unknown to the filter): body to vehicle (x forward, y left, z up)
  const Sophus::SO3d vehicle_from_body =
      Sophus::SO3d::rotX(1.1) * Sophus::SO3d::rotZ(0.4) * Sophus::SO3d::rotY(0.2);
  // gyroscope bias about the vehicle vertical: only GPS makes it (and heading) observable
  const Eigen::Vector3d gyroscope_bias = vehicle_from_body.inverse() * Eigen::Vector3d(0, 0, 0.01);
  // the simulated world frame is the UTM grid, offset to the first fix
  const Eigen::Vector3d origin(22.787839, 113.5210145, 13.0);
  const int zone = math::UtmZone(origin.y());
  const Eigen::Vector3d origin_utm = math::GeodeticToUtm(origin, zone);
  const Eigen::Vector3d gravity_world(0.0, 0.0, -kGravity);

  InsFusion fusion;
  double yaw = 0.7;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double max_position_error = 0.0;
  double max_rotation_error = 0.0;
  int pose_count = 0;
  for (int i = 0; i <= 40 * 100; ++i) {
    const int64_t timestamp_ns = 1'000'000'000 + i * kImuPeriodNs;
    const double t = 1e-9 * static_cast<double>(i * kImuPeriodNs);
    const double dt = 1e-9 * static_cast<double>(kImuPeriodNs);
    if (i > 0) {
      // exact integration over the previous period (motion is piecewise constant per sample)
      const Motion motion = MotionAt(t);
      const double previous_speed = motion.speed - motion.acceleration * dt;
      constexpr int kSubsteps = 100;
      for (int step = 0; step < kSubsteps; ++step) {
        const double fraction = (step + 0.5) / kSubsteps;
        const double speed = previous_speed + fraction * motion.acceleration * dt;
        const double heading = yaw + fraction * motion.yaw_rate * dt;
        position +=
            (dt / kSubsteps) * speed * Eigen::Vector3d(std::cos(heading), std::sin(heading), 0);
      }
      yaw += motion.yaw_rate * dt;
    }
    const Motion motion = MotionAt(t);
    const Sophus::SO3d world_from_body = Sophus::SO3d::rotZ(yaw) * vehicle_from_body;
    const Eigen::Vector3d acceleration_world =
        motion.acceleration * Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0) +
        motion.speed * motion.yaw_rate * Eigen::Vector3d(-std::sin(yaw), std::cos(yaw), 0);

    if (i % kGpsEvery == 0) {
      const Eigen::Vector3d geodetic = UtmOffsetToGeodetic(position, origin, zone);
      fusion.AddGps({timestamp_ns, 0, geodetic.x(), geodetic.y(), geodetic.z(), motion.speed, 8});
    }
    if (i % kWheelEvery == 0) {
      fusion.AddWheelSpeed({timestamp_ns, motion.speed});
    }
    fusion.AddImu(
        {timestamp_ns, world_from_body.inverse() * (acceleration_world - gravity_world),
         world_from_body.inverse() * Eigen::Vector3d(0, 0, motion.yaw_rate) + gyroscope_bias});

    if (fusion.initialized()) {
      ++pose_count;
      const Sophus::SE3d pose = fusion.pose();
      max_position_error =
          std::max(max_position_error, (pose.translation() - origin_utm - position).norm());
      if (t >= kSettledSeconds) {
        max_rotation_error =
            std::max(max_rotation_error, (pose.so3().inverse() * world_from_body).log().norm());
      }
    }
  }

  EXPECT_EQ(fusion.utm_zone(), zone);
  EXPECT_GT(pose_count, 20 * 100);
  EXPECT_LT(max_position_error, 0.5);
  EXPECT_LT(max_rotation_error, 0.1);
}

}  // namespace
}  // namespace mapmind::fusion
