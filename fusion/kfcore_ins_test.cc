// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "fusion/kfcore_ins.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "sophus/so3.hpp"

namespace mapmind::fusion {
namespace {

constexpr int kStateSize = KFCORE_INS_STATE_SIZE;
using StateVector = Eigen::Matrix<double, kStateSize, 1>;
using Covariance = Eigen::Matrix<double, kStateSize, kStateSize>;
template <int kRows>
using Jacobian = Eigen::Matrix<double, kRows, kStateSize, Eigen::RowMajor>;

constexpr double kGravity = 9.81;
constexpr KfcoreInsProcessNoise kProcessNoise = {0.2, 0.01, 2e-3, 2e-4, 0.01, 5e-4, 1e-3};

Eigen::Vector3d Position(const KfcoreIns& filter) {
  return Eigen::Map<const Eigen::Vector3d>(filter.position);
}
Eigen::Vector3d Velocity(const KfcoreIns& filter) {
  return Eigen::Map<const Eigen::Vector3d>(filter.velocity);
}
Eigen::Vector3d AccelerometerBias(const KfcoreIns& filter) {
  return Eigen::Map<const Eigen::Vector3d>(filter.accelerometer_bias);
}
Eigen::Vector3d GyroscopeBias(const KfcoreIns& filter) {
  return Eigen::Map<const Eigen::Vector3d>(filter.gyroscope_bias);
}
Sophus::SO3d Rotation(const KfcoreIns& filter) {
  const double* q = filter.rotation;
  return Sophus::SO3d(Eigen::Quaterniond(q[0], q[1], q[2], q[3]));
}
Covariance CovarianceOf(const KfcoreIns& filter) {
  Covariance covariance;
  kfcore_ins_covariance(&filter, covariance.data());
  return covariance;
}

void Reset(KfcoreIns* filter, const Eigen::Vector3d& position, const Sophus::SO3d& rotation,
           const Eigen::Vector3d& velocity, const StateVector& initial_std) {
  const Eigen::Quaterniond& q = rotation.unit_quaternion();
  const double rotation_wxyz[4] = {q.w(), q.x(), q.y(), q.z()};
  const Eigen::Vector3d gyroscope_bias = Eigen::Vector3d::Zero();
  kfcore_ins_reset(filter, position.data(), rotation_wxyz, velocity.data(), gyroscope_bias.data(),
                   initial_std.data());
}

void Predict(KfcoreIns* filter, const Eigen::Vector3d& acceleration,
             const Eigen::Vector3d& angular_velocity, double dt) {
  kfcore_ins_predict(filter, acceleration.data(), angular_velocity.data(), dt);
}

template <int kRows>
bool Update(KfcoreIns* filter, const Eigen::Matrix<double, kRows, 1>& residual,
            const Jacobian<kRows>& jacobian, const Eigen::Matrix<double, kRows, 1>& variance) {
  return kfcore_ins_update(filter, residual.data(), jacobian.data(), variance.data(), kRows);
}

KfcoreIns MakeFilter(double covariance_period = 0.0) {
  KfcoreIns filter;
  kfcore_ins_init(&filter, &kProcessNoise, kGravity, covariance_period);
  StateVector initial_std;
  initial_std << Eigen::Vector3d(1.5, 1.5, 10.0), Eigen::Vector3d::Constant(1.0),
      Eigen::Vector3d(0.05, 0.05, 0.3), Eigen::Vector3d::Constant(0.3),
      Eigen::Vector3d::Constant(0.02);
  Reset(&filter, Eigen::Vector3d(3e5, 2.5e6, 10.0),
        Sophus::SO3d::rotZ(0.7) * Sophus::SO3d::rotX(0.2), Eigen::Vector3d(4.0, -2.0, 0.0),
        initial_std);
  return filter;
}

// Dense covariance prediction over `dt` at the filter's current state.
Covariance DensePrediction(const KfcoreIns& filter, const Eigen::Vector3d& acceleration,
                           double dt) {
  const Eigen::Matrix3d rotation = Rotation(filter).matrix();
  const Eigen::Vector3d specific_force_world =
      rotation * (acceleration - AccelerometerBias(filter));
  Covariance transition = Covariance::Identity();
  transition.block<3, 3>(KFCORE_INS_POSITION, KFCORE_INS_VELOCITY).diagonal().setConstant(dt);
  transition.block<3, 3>(KFCORE_INS_VELOCITY, KFCORE_INS_ATTITUDE) =
      -Sophus::SO3d::hat(specific_force_world) * dt;
  transition.block<3, 3>(KFCORE_INS_VELOCITY, KFCORE_INS_ACCELEROMETER_BIAS) = -rotation * dt;
  transition.block<3, 3>(KFCORE_INS_ATTITUDE, KFCORE_INS_GYROSCOPE_BIAS) = -rotation * dt;
  const auto variance = [dt](double density) { return density * density * dt; };
  StateVector noise;
  noise << Eigen::Vector3d::Constant(variance(kProcessNoise.position)),
      Eigen::Vector3d::Constant(variance(kProcessNoise.accelerometer) +
                                variance(kProcessNoise.velocity)),
      Eigen::Vector3d::Constant(variance(kProcessNoise.gyroscope) +
                                variance(kProcessNoise.attitude)),
      Eigen::Vector3d::Constant(variance(kProcessNoise.accelerometer_bias)),
      Eigen::Vector3d::Constant(variance(kProcessNoise.gyroscope_bias));
  return transition * CovarianceOf(filter) * transition.transpose() +
         Covariance(noise.asDiagonal());
}

// float U-D factors: relative accuracy ~1e-5 of the largest variance
void ExpectCovarianceNear(const Covariance& actual, const Covariance& expected) {
  EXPECT_LT((actual - expected).cwiseAbs().maxCoeff(), 1e-4 * expected.maxCoeff());
}

TEST(KfcoreIns, PredictsCovarianceLikeDenseFilter) {
  KfcoreIns filter = MakeFilter();
  // correlate the states first
  Update<3>(&filter, Eigen::Vector3d(0.5, -0.2, 1.0), Jacobian<3>::Random(),
            Eigen::Vector3d(0.5, 0.5, 2.0));
  const Eigen::Vector3d acceleration(0.3, -0.1, 9.9);
  constexpr double kDt = 0.01;
  const Covariance expected = DensePrediction(filter, acceleration, kDt);

  Predict(&filter, acceleration, Eigen::Vector3d(0.01, 0.02, 0.3), kDt);
  ExpectCovarianceNear(CovarianceOf(filter), expected);
}

TEST(KfcoreIns, AddsProcessNoiseLikeDenseFilter) {
  // tight initial covariance, so the process noise dominates the prediction
  KfcoreIns filter;
  kfcore_ins_init(&filter, &kProcessNoise, kGravity, /*covariance_period=*/0.0);
  Reset(&filter, Eigen::Vector3d::Zero(), Sophus::SO3d::rotX(0.2), Eigen::Vector3d(4.0, -2.0, 0.0),
        StateVector::Constant(1e-4));
  const Eigen::Vector3d acceleration(0.3, -0.1, 9.9);
  constexpr double kDt = 0.1;
  const Covariance expected = DensePrediction(filter, acceleration, kDt);

  Predict(&filter, acceleration, Eigen::Vector3d::Zero(), kDt);
  const StateVector variance = CovarianceOf(filter).diagonal();
  EXPECT_LT(
      ((variance - expected.diagonal()).array() / expected.diagonal().array()).abs().maxCoeff(),
      1e-3);
}

TEST(KfcoreIns, PredictsCovarianceOncePerPeriod) {
  KfcoreIns filter = MakeFilter(/*covariance_period=*/0.05);
  Update<3>(&filter, Eigen::Vector3d(0.5, -0.2, 1.0), Jacobian<3>::Random(),
            Eigen::Vector3d(0.5, 0.5, 2.0));
  const Covariance covariance = CovarianceOf(filter);
  const Eigen::Vector3d acceleration(0.3, -0.1, 9.9);
  // no rotation, so the transition is the same at every sample
  const Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  constexpr double kDt = 0.02;

  Predict(&filter, acceleration, angular_velocity, kDt);
  Predict(&filter, acceleration, angular_velocity, kDt);
  EXPECT_EQ(CovarianceOf(filter), covariance);
  // the third sample completes the period: one prediction over all three
  const Covariance expected = DensePrediction(filter, acceleration, 3 * kDt);
  Predict(&filter, acceleration, angular_velocity, kDt);
  ExpectCovarianceNear(CovarianceOf(filter), expected);
}

TEST(KfcoreIns, UpdatesLikeDenseFilter) {
  KfcoreIns filter = MakeFilter();
  const Covariance covariance = CovarianceOf(filter);
  const Jacobian<3> jacobian = Jacobian<3>::Random();
  const Eigen::Vector3d residual(0.5, -0.2, 1.0);
  const Eigen::Vector3d variance(0.5, 0.5, 2.0);
  const Eigen::Matrix<double, kStateSize, 3> gain =
      covariance * jacobian.transpose() *
      (jacobian * covariance * jacobian.transpose() + Eigen::Matrix3d(variance.asDiagonal()))
          .inverse();
  const StateVector error = gain * residual;
  const Covariance expected = covariance - gain * jacobian * covariance;
  const Eigen::Vector3d position = Position(filter);
  const Eigen::Vector3d velocity = Velocity(filter);
  const Sophus::SO3d rotation = Rotation(filter);

  ASSERT_TRUE(Update<3>(&filter, residual, jacobian, variance));
  ExpectCovarianceNear(CovarianceOf(filter), expected);
  EXPECT_TRUE((Position(filter) - position).isApprox(error.segment<3>(KFCORE_INS_POSITION), 1e-4));
  EXPECT_TRUE((Velocity(filter) - velocity).isApprox(error.segment<3>(KFCORE_INS_VELOCITY), 1e-4));
  EXPECT_TRUE((Rotation(filter) * rotation.inverse())
                  .log()
                  .isApprox(error.segment<3>(KFCORE_INS_ATTITUDE), 1e-4));
  EXPECT_TRUE(
      AccelerometerBias(filter).isApprox(error.segment<3>(KFCORE_INS_ACCELEROMETER_BIAS), 1e-4));
  EXPECT_TRUE(GyroscopeBias(filter).isApprox(error.segment<3>(KFCORE_INS_GYROSCOPE_BIAS), 1e-4));
}

TEST(KfcoreIns, RejectsUnsupportedRowCount) {
  KfcoreIns filter = MakeFilter();
  const double values[1] = {0.0};
  EXPECT_FALSE(kfcore_ins_update(&filter, values, values, values, 0));
  EXPECT_FALSE(kfcore_ins_update(&filter, values, values, values, KFCORE_INS_MAX_UPDATE_ROWS + 1));
}

TEST(KfcoreIns, IntegratesConstantAccelerationAndRotation) {
  KfcoreIns filter = MakeFilter();
  const Eigen::Vector3d position = Position(filter);
  const Eigen::Vector3d velocity = Velocity(filter);
  const Sophus::SO3d rotation = Rotation(filter);
  const Eigen::Vector3d acceleration_world(0.5, 1.0, 0.0);
  const Eigen::Vector3d specific_force =
      rotation.inverse() * (acceleration_world + Eigen::Vector3d(0.0, 0.0, kGravity));
  for (int i = 0; i < 100; ++i) {
    Predict(&filter, specific_force, Eigen::Vector3d::Zero(), 0.01);
  }
  EXPECT_TRUE(Velocity(filter).isApprox(velocity + acceleration_world, 1e-9));
  EXPECT_LT((Position(filter) - (position + velocity + 0.5 * acceleration_world)).norm(), 1e-6);
  EXPECT_LT((Rotation(filter).inverse() * rotation).log().norm(), 1e-12);

  // constant body rate: rotation * exp(rate * t)
  const Eigen::Vector3d angular_velocity(0.3, -0.2, 0.5);
  for (int i = 0; i < 100; ++i) {
    Predict(&filter, specific_force, angular_velocity, 0.01);
  }
  EXPECT_LT(
      (Rotation(filter).inverse() * rotation * Sophus::SO3d::exp(angular_velocity)).log().norm(),
      1e-12);
}

}  // namespace
}  // namespace mapmind::fusion
