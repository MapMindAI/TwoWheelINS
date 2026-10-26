// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "fusion/kfcore_ins.h"

#include <gtest/gtest.h>

namespace mapmind::fusion {
namespace {

constexpr double kGravity = 9.81;
constexpr KfcoreIns::ProcessNoise kProcessNoise = {0.2, 0.01, 2e-3, 2e-4};

KfcoreIns MakeFilter() {
  KfcoreIns filter(kProcessNoise, kGravity);
  KfcoreIns::StateVector initial_std;
  initial_std << Eigen::Vector3d(1.5, 1.5, 10.0), Eigen::Vector3d::Constant(1.0),
      Eigen::Vector3d(0.05, 0.05, 0.3), Eigen::Vector3d::Constant(0.3),
      Eigen::Vector3d::Constant(0.02);
  filter.Reset(Eigen::Vector3d(3e5, 2.5e6, 10.0), Sophus::SO3d::rotZ(0.7) * Sophus::SO3d::rotX(0.2),
               Eigen::Vector3d(4.0, -2.0, 0.0), Eigen::Vector3d::Zero(), initial_std);
  return filter;
}

TEST(KfcoreIns, PredictsCovarianceLikeDenseFilter) {
  KfcoreIns filter = MakeFilter();
  // correlate the states first
  filter.Update<3>(Eigen::Vector3d(0.5, -0.2, 1.0), Eigen::Matrix<double, 3, 15>::Random(),
                   Eigen::Vector3d(0.5, 0.5, 2.0));
  const KfcoreIns::Covariance covariance = filter.covariance();
  const Eigen::Matrix3d rotation = filter.rotation().matrix();
  const Eigen::Vector3d acceleration(0.3, -0.1, 9.9);
  const Eigen::Vector3d specific_force_world =
      rotation * (acceleration - filter.accelerometer_bias());
  constexpr double kDt = 0.01;

  KfcoreIns::Covariance transition = KfcoreIns::Covariance::Identity();
  transition.block<3, 3>(KfcoreIns::kPosition, KfcoreIns::kVelocity).diagonal().setConstant(kDt);
  transition.block<3, 3>(KfcoreIns::kVelocity, KfcoreIns::kAttitude) =
      -Sophus::SO3d::hat(specific_force_world) * kDt;
  transition.block<3, 3>(KfcoreIns::kVelocity, KfcoreIns::kAccelerometerBias) = -rotation * kDt;
  transition.block<3, 3>(KfcoreIns::kAttitude, KfcoreIns::kGyroscopeBias) = -rotation * kDt;
  KfcoreIns::Covariance noise = KfcoreIns::Covariance::Zero();
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  noise.block<3, 3>(KfcoreIns::kVelocity, KfcoreIns::kVelocity) =
      kProcessNoise.accelerometer * kProcessNoise.accelerometer * kDt * identity;
  noise.block<3, 3>(KfcoreIns::kAttitude, KfcoreIns::kAttitude) =
      kProcessNoise.gyroscope * kProcessNoise.gyroscope * kDt * identity;
  noise.block<3, 3>(KfcoreIns::kAccelerometerBias, KfcoreIns::kAccelerometerBias) =
      kProcessNoise.accelerometer_bias * kProcessNoise.accelerometer_bias * kDt * identity;
  noise.block<3, 3>(KfcoreIns::kGyroscopeBias, KfcoreIns::kGyroscopeBias) =
      kProcessNoise.gyroscope_bias * kProcessNoise.gyroscope_bias * kDt * identity;
  const KfcoreIns::Covariance expected = transition * covariance * transition.transpose() + noise;

  filter.Predict(acceleration, Eigen::Vector3d(0.01, 0.02, 0.3), kDt);
  // float U-D factors: relative accuracy ~1e-5 of the largest variance
  EXPECT_LT((filter.covariance() - expected).cwiseAbs().maxCoeff(), 1e-4 * expected.maxCoeff());
}

TEST(KfcoreIns, UpdatesLikeDenseFilter) {
  KfcoreIns filter = MakeFilter();
  const KfcoreIns::Covariance covariance = filter.covariance();
  const Eigen::Matrix<double, 3, 15> jacobian = Eigen::Matrix<double, 3, 15>::Random();
  const Eigen::Vector3d residual(0.5, -0.2, 1.0);
  const Eigen::Vector3d variance(0.5, 0.5, 2.0);
  const Eigen::Matrix<double, 15, 3> gain =
      covariance * jacobian.transpose() *
      (jacobian * covariance * jacobian.transpose() + Eigen::Matrix3d(variance.asDiagonal()))
          .inverse();
  const KfcoreIns::StateVector error = gain * residual;
  const KfcoreIns::Covariance expected = covariance - gain * jacobian * covariance;
  const Eigen::Vector3d position = filter.position();
  const Eigen::Vector3d velocity = filter.velocity();
  const Sophus::SO3d rotation = filter.rotation();

  ASSERT_TRUE(filter.Update<3>(residual, jacobian, variance));
  EXPECT_LT((filter.covariance() - expected).cwiseAbs().maxCoeff(), 1e-4 * expected.maxCoeff());
  EXPECT_TRUE(
      (filter.position() - position).isApprox(error.segment<3>(KfcoreIns::kPosition), 1e-4));
  EXPECT_TRUE(
      (filter.velocity() - velocity).isApprox(error.segment<3>(KfcoreIns::kVelocity), 1e-4));
  EXPECT_TRUE((filter.rotation() * rotation.inverse())
                  .log()
                  .isApprox(error.segment<3>(KfcoreIns::kAttitude), 1e-4));
  EXPECT_TRUE(
      filter.accelerometer_bias().isApprox(error.segment<3>(KfcoreIns::kAccelerometerBias), 1e-4));
  EXPECT_TRUE(filter.gyroscope_bias().isApprox(error.segment<3>(KfcoreIns::kGyroscopeBias), 1e-4));
}

TEST(KfcoreIns, IntegratesConstantAcceleration) {
  KfcoreIns filter = MakeFilter();
  const Eigen::Vector3d position = filter.position();
  const Eigen::Vector3d velocity = filter.velocity();
  const Sophus::SO3d rotation = filter.rotation();
  const Eigen::Vector3d acceleration_world(0.5, 1.0, 0.0);
  const Eigen::Vector3d specific_force =
      rotation.inverse() * (acceleration_world + Eigen::Vector3d(0.0, 0.0, kGravity));
  for (int i = 0; i < 100; ++i) {
    filter.Predict(specific_force, Eigen::Vector3d::Zero(), 0.01);
  }
  EXPECT_TRUE(filter.velocity().isApprox(velocity + acceleration_world, 1e-9));
  EXPECT_LT((filter.position() - (position + velocity + 0.5 * acceleration_world)).norm(), 1e-6);
  EXPECT_LT((filter.rotation().inverse() * rotation).log().norm(), 1e-12);
}

}  // namespace
}  // namespace mapmind::fusion
