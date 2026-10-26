// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "fusion/ins_fusion.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>

#include <gtest/gtest.h>
#include "common/math/geodesy.h"
#include "fusion/gps_covariance.h"

namespace mapmind::fusion {
namespace {

constexpr double kGravity = 9.8099;
constexpr int64_t kImuPeriodNs = 10'000'000;  // 100 Hz
constexpr int kWheelEvery = 10;               // 10 Hz
constexpr int kGpsEvery = 100;                // 1 Hz
// Yaw hypotheses start at 6 s (two fixes at >= 3 m/s) and one is selected 10 fixes later.
constexpr double kSettledSeconds = 16.0;

struct Motion {
  double speed;
  double acceleration;
  double yaw_rate;
};

// Planar ride: 2 s static, accelerate at 1 m/s^2 for 5 s, then turn at 0.2 rad/s.
Motion Ride(double t) {
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

struct Simulation {
  std::function<Motion(double)> motion = Ride;
  double duration_s = 40.0;
  int satellite_count = 8;
  // Returns the GPS position error (UTM, m) at `t`, or nullopt to drop the fix.
  std::function<std::optional<Eigen::Vector3d>(double)> gps_error = [](double) {
    return Eigen::Vector3d::Zero();
  };
};

struct Errors {
  int pose_count = 0;
  std::optional<int> utm_zone;
  // after kSettledSeconds, in [error_from_s, error_to_s)
  double max_position = 0.0;
  double max_height = 0.0;
  double max_rotation = 0.0;
};

Errors Simulate(const Simulation& simulation, double error_from_s, double error_to_s) {
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
  Errors errors;
  double yaw = 0.7;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  for (int i = 0; i <= simulation.duration_s * 100; ++i) {
    const int64_t timestamp_ns = 1'000'000'000 + i * kImuPeriodNs;
    const double t = 1e-9 * static_cast<double>(i * kImuPeriodNs);
    const double dt = 1e-9 * static_cast<double>(kImuPeriodNs);
    const Motion motion = simulation.motion(t);
    if (i > 0) {
      // exact integration over the previous period (motion is piecewise constant per sample)
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
    const Sophus::SO3d world_from_body = Sophus::SO3d::rotZ(yaw) * vehicle_from_body;
    const Eigen::Vector3d acceleration_world =
        motion.acceleration * Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0) +
        motion.speed * motion.yaw_rate * Eigen::Vector3d(-std::sin(yaw), std::cos(yaw), 0);

    if (i % kGpsEvery == 0) {
      if (const std::optional<Eigen::Vector3d> error = simulation.gps_error(t)) {
        const Eigen::Vector3d geodetic = UtmOffsetToGeodetic(position + *error, origin, zone);
        fusion.AddGps({timestamp_ns, 0, geodetic.x(), geodetic.y(), geodetic.z(), motion.speed,
                       simulation.satellite_count});
      }
    }
    if (i % kWheelEvery == 0) {
      fusion.AddWheelSpeed({timestamp_ns, motion.speed});
    }
    fusion.AddImu(
        {timestamp_ns, world_from_body.inverse() * (acceleration_world - gravity_world),
         world_from_body.inverse() * Eigen::Vector3d(0, 0, motion.yaw_rate) + gyroscope_bias});

    if (!fusion.initialized()) {
      continue;
    }
    ++errors.pose_count;
    if (t < kSettledSeconds || t < error_from_s || t >= error_to_s) {
      continue;
    }
    const Sophus::SE3d pose = fusion.pose();
    const Eigen::Vector3d position_error = pose.translation() - origin_utm - position;
    errors.max_position = std::max(errors.max_position, position_error.norm());
    errors.max_height = std::max(errors.max_height, std::abs(position_error.z()));
    errors.max_rotation =
        std::max(errors.max_rotation, (pose.so3().inverse() * world_from_body).log().norm());
  }
  errors.utm_zone = fusion.utm_zone();
  return errors;
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
  const Errors errors = Simulate(Simulation{}, 0.0, 40.0);
  EXPECT_EQ(errors.utm_zone, math::UtmZone(113.5210145));
  EXPECT_GT(errors.pose_count, 20 * 100);
  EXPECT_LT(errors.max_position, 0.5);
  // Yaw is observable only while the speed changes: in a steady turn a yaw error looks like a
  // body-fixed accelerometer bias, so the error left after yaw selection decays slowly.
  EXPECT_LT(errors.max_rotation, 0.15);
}

TEST(InsFusion, HoldsStillWhileParkedGpsDrifts) {
  Simulation simulation;
  // ride, brake to a stop at 30 s and park while the GPS walks off at 1 m/s
  simulation.motion = [](double t) {
    if (t < 25.0) {
      return Ride(t);
    }
    if (t < 30.0) {
      return Motion{30.0 - t, -1.0, 0.0};
    }
    return Motion{0.0, 0.0, 0.0};
  };
  simulation.duration_s = 60.0;
  simulation.gps_error = [](double t) -> std::optional<Eigen::Vector3d> {
    return Eigen::Vector3d(0.0, std::max(t - 31.0, 0.0), 0.0);
  };
  EXPECT_LT(Simulate(simulation, 31.0, 60.0).max_position, 0.5);
}

TEST(InsFusion, IgnoresGpsAltitudeDriftOnLevelRoad) {
  Simulation simulation;
  // the GPS altitude climbs at 1 m/s on a level road
  simulation.gps_error = [](double t) -> std::optional<Eigen::Vector3d> {
    return Eigen::Vector3d(0.0, 0.0, std::max(t - 20.0, 0.0));
  };
  EXPECT_LT(Simulate(simulation, 20.0, 40.0).max_height, 8.0);
}

TEST(InsFusion, DropsAltitudeOutliers) {
  Simulation simulation;
  simulation.satellite_count = 30;  // fix solution: 0.1 m vertical std
  simulation.gps_error = [](double t) -> std::optional<Eigen::Vector3d> {
    return Eigen::Vector3d(0.0, 0.0, t >= 25.0 && t < 28.0 ? 50.0 : 0.0);
  };
  EXPECT_LT(Simulate(simulation, 20.0, 40.0).max_height, 0.5);
}

}  // namespace
}  // namespace mapmind::fusion
