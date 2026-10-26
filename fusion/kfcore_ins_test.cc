// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "fusion/kfcore_ins.h"

#include <gtest/gtest.h>

namespace mapmind::fusion {
namespace {

constexpr double kGravity = 9.81;
constexpr KfcoreIns::ProcessNoise kProcessNoise = {0.2, 0.01, 2e-3, 2e-4, 0.01, 5e-4, 1e-3};

KfcoreIns MakeFilter(double covariance_period = 0.0) {
  KfcoreIns filter(kProcessNoise, kGravity, covariance_period);
  KfcoreIns::StateVector initial_std;
  initial_std << Eigen::Vector3d(1.5, 1.5, 10.0), Eigen::Vector3d::Constant(1.0),
      Eigen::Vector3d(0.05, 0.05, 0.3), Eigen::Vector3d::Constant(0.3),
      Eigen::Vector3d::Constant(0.02);
  filter.Reset(Eigen::Vector3d(3e5, 2.5e6, 10.0), Sophus::SO3d::rotZ(0.7) * Sophus::SO3d::rotX(0.2),
               Eigen::Vector3d(4.0, -2.0, 0.0), Eigen::Vector3d::Zero(), initial_std);
  return filter;
}

// Dense covariance prediction over `dt` at the filter's current state.
KfcoreIns::Covariance DensePrediction(const KfcoreIns& filter, const Eigen::Vector3d& acceleration,
                                      double dt) {
  const Eigen::Matrix3d rotation = filter.rotation().matrix();
  const Eigen::Vector3d specific_force_world =
      rotation * (acceleration - filter.accelerometer_bias());
  KfcoreIns::Covariance transition = KfcoreIns::Covariance::Identity();
  transition.block<3, 3>(KfcoreIns::kPosition, KfcoreIns::kVelocity).diagonal().setConstant(dt);
  transition.block<3, 3>(KfcoreIns::kVelocity, KfcoreIns::kAttitude) =
      -Sophus::SO3d::hat(specific_force_world) * dt;
  transition.block<3, 3>(KfcoreIns::kVelocity, KfcoreIns::kAccelerometerBias) = -rotation * dt;
  transition.block<3, 3>(KfcoreIns::kAttitude, KfcoreIns::kGyroscopeBias) = -rotation * dt;
  const auto variance = [dt](double density) { return density * density * dt; };
  KfcoreIns::StateVector noise;
  noise << Eigen::Vector3d::Constant(variance(kProcessNoise.position)),
      Eigen::Vector3d::Constant(variance(kProcessNoise.accelerometer) +
                                variance(kProcessNoise.velocity)),
      Eigen::Vector3d::Constant(variance(kProcessNoise.gyroscope) +
                                variance(kProcessNoise.attitude)),
      Eigen::Vector3d::Constant(variance(kProcessNoise.accelerometer_bias)),
      Eigen::Vector3d::Constant(variance(kProcessNoise.gyroscope_bias));
  return transition * filter.covariance() * transition.transpose() +
         KfcoreIns::Covariance(noise.asDiagonal());
}

// float U-D factors: relative accuracy ~1e-5 of the largest variance
void ExpectCovarianceNear(const KfcoreIns::Covariance& actual,
                          const KfcoreIns::Covariance& expected) {
  EXPECT_LT((actual - expected).cwiseAbs().maxCoeff(), 1e-4 * expected.maxCoeff());
}

TEST(KfcoreIns, PredictsCovarianceLikeDenseFilter) {
  KfcoreIns filter = MakeFilter();
  // correlate the states first
  filter.Update<3>(Eigen::Vector3d(0.5, -0.2, 1.0), Eigen::Matrix<double, 3, 15>::Random(),
                   Eigen::Vector3d(0.5, 0.5, 2.0));
  const Eigen::Vector3d acceleration(0.3, -0.1, 9.9);
  constexpr double kDt = 0.01;
  const KfcoreIns::Covariance expected = DensePrediction(filter, acceleration, kDt);

  filter.Predict(acceleration, Eigen::Vector3d(0.01, 0.02, 0.3), kDt);
  ExpectCovarianceNear(filter.covariance(), expected);
}

TEST(KfcoreIns, AddsProcessNoiseLikeDenseFilter) {
  // tight initial covariance, so the process noise dominates the prediction
  KfcoreIns filter(kProcessNoise, kGravity, /*covariance_period=*/0.0);
  filter.Reset(Eigen::Vector3d::Zero(), Sophus::SO3d::rotX(0.2), Eigen::Vector3d(4.0, -2.0, 0.0),
               Eigen::Vector3d::Zero(), KfcoreIns::StateVector::Constant(1e-4));
  const Eigen::Vector3d acceleration(0.3, -0.1, 9.9);
  constexpr double kDt = 0.1;
  const KfcoreIns::Covariance expected = DensePrediction(filter, acceleration, kDt);

  filter.Predict(acceleration, Eigen::Vector3d::Zero(), kDt);
  const KfcoreIns::StateVector variance = filter.covariance().diagonal();
  EXPECT_LT(
      ((variance - expected.diagonal()).array() / expected.diagonal().array()).abs().maxCoeff(),
      1e-3);
}

TEST(KfcoreIns, PredictsCovarianceOncePerPeriod) {
  KfcoreIns filter = MakeFilter(/*covariance_period=*/0.05);
  filter.Update<3>(Eigen::Vector3d(0.5, -0.2, 1.0), Eigen::Matrix<double, 3, 15>::Random(),
                   Eigen::Vector3d(0.5, 0.5, 2.0));
  const KfcoreIns::Covariance covariance = filter.covariance();
  const Eigen::Vector3d acceleration(0.3, -0.1, 9.9);
  // no rotation, so the transition is the same at every sample
  const Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  constexpr double kDt = 0.02;

  filter.Predict(acceleration, angular_velocity, kDt);
  filter.Predict(acceleration, angular_velocity, kDt);
  EXPECT_EQ(filter.covariance(), covariance);
  // the third sample completes the period: one prediction over all three
  const KfcoreIns::Covariance expected = DensePrediction(filter, acceleration, 3 * kDt);
  filter.Predict(acceleration, angular_velocity, kDt);
  ExpectCovarianceNear(filter.covariance(), expected);
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
  ExpectCovarianceNear(filter.covariance(), expected);
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
