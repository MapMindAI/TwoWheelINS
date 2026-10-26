// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "fusion/kfcore_ins.h"

#include "third_party/kfcore/kalman_udu.h"

namespace mapmind::fusion {

namespace {

// IMU noise inputs: accelerometer, gyroscope, accelerometer bias walk, gyroscope bias walk.
constexpr int kNoiseSize = 12;

}  // namespace

KfcoreIns::KfcoreIns(const ProcessNoise& process_noise, double gravity)
    : process_noise_(process_noise), gravity_(0.0, 0.0, -gravity) {}

void KfcoreIns::Reset(const Eigen::Vector3d& position, const Sophus::SO3d& rotation,
                      const Eigen::Vector3d& velocity, const Eigen::Vector3d& gyroscope_bias,
                      const StateVector& initial_std) {
  position_ = position;
  rotation_ = rotation;
  velocity_ = velocity;
  accelerometer_bias_.setZero();
  gyroscope_bias_ = gyroscope_bias;
  u_.setIdentity();
  d_ = initial_std.cwiseAbs2().cast<float>();
}

void KfcoreIns::Predict(const Eigen::Vector3d& acceleration,
                        const Eigen::Vector3d& angular_velocity, double dt) {
  if (dt <= 0.0) {
    return;
  }
  const Eigen::Matrix3d rotation = rotation_.matrix();
  const Eigen::Vector3d specific_force = acceleration - accelerometer_bias_;
  const Eigen::Vector3d specific_force_world = rotation * specific_force;

  // error transition, first order: Phi = I + F dt
  Eigen::Matrix<float, kStateSize, kStateSize> transition;
  transition.setIdentity();
  transition.block<3, 3>(kPosition, kVelocity) = Eigen::Matrix3f::Identity() * dt;
  transition.block<3, 3>(kVelocity, kAttitude) =
      (-Sophus::SO3d::hat(specific_force_world) * dt).cast<float>();
  transition.block<3, 3>(kVelocity, kAccelerometerBias) = (-rotation * dt).cast<float>();
  transition.block<3, 3>(kAttitude, kGyroscopeBias) = (-rotation * dt).cast<float>();

  Eigen::Matrix<float, kStateSize, kNoiseSize> noise_input;
  noise_input.setZero();
  noise_input.block<3, 3>(kVelocity, 0) = rotation.cast<float>();
  noise_input.block<3, 3>(kAttitude, 3) = rotation.cast<float>();
  noise_input.block<3, 3>(kAccelerometerBias, 6).setIdentity();
  noise_input.block<3, 3>(kGyroscopeBias, 9).setIdentity();
  Eigen::Matrix<float, kNoiseSize, 1> noise_variance;
  noise_variance << Eigen::Vector3f::Constant(process_noise_.accelerometer *
                                              process_noise_.accelerometer * dt),
      Eigen::Vector3f::Constant(process_noise_.gyroscope * process_noise_.gyroscope * dt),
      Eigen::Vector3f::Constant(process_noise_.accelerometer_bias *
                                process_noise_.accelerometer_bias * dt),
      Eigen::Vector3f::Constant(process_noise_.gyroscope_bias * process_noise_.gyroscope_bias * dt);
  // The error state is zero after every correction, so only the covariance is predicted.
  kalman_udu_predict(nullptr, u_.data(), d_.data(), transition.data(), noise_input.data(),
                     noise_variance.data(), kStateSize, kNoiseSize);

  // nominal state: trapezoidal position
  const Eigen::Vector3d previous_velocity = velocity_;
  velocity_ += (specific_force_world + gravity_) * dt;
  position_ += 0.5 * dt * (previous_velocity + velocity_);
  rotation_ = rotation_ * Sophus::SO3d::exp((angular_velocity - gyroscope_bias_) * dt);
}

template <int kRows>
bool KfcoreIns::Update(const Eigen::Matrix<double, kRows, 1>& residual,
                       const Eigen::Matrix<double, kRows, kStateSize>& jacobian,
                       const Eigen::Matrix<double, kRows, 1>& variance) {
  Eigen::Matrix<float, kRows, 1> measurement = residual.template cast<float>();
  Eigen::Matrix<float, kStateSize, kRows> jacobian_transpose =
      jacobian.transpose().template cast<float>();
  Eigen::Matrix<float, kRows, kRows> measurement_covariance =
      variance.template cast<float>().asDiagonal();
  Eigen::Matrix<float, kStateSize, 1> error = Eigen::Matrix<float, kStateSize, 1>::Zero();
  const int status = kalman_udu(error.data(), u_.data(), d_.data(), measurement.data(),
                                measurement_covariance.data(), jacobian_transpose.data(),
                                kStateSize, kRows, /*chi2_threshold=*/0.0f,
                                /*downweight_outlier=*/0);
  Correct(error);
  return status == 0;
}

template bool KfcoreIns::Update<1>(const Eigen::Matrix<double, 1, 1>&,
                                   const Eigen::Matrix<double, 1, kStateSize>&,
                                   const Eigen::Matrix<double, 1, 1>&);
template bool KfcoreIns::Update<2>(const Eigen::Matrix<double, 2, 1>&,
                                   const Eigen::Matrix<double, 2, kStateSize>&,
                                   const Eigen::Matrix<double, 2, 1>&);
template bool KfcoreIns::Update<3>(const Eigen::Matrix<double, 3, 1>&,
                                   const Eigen::Matrix<double, 3, kStateSize>&,
                                   const Eigen::Matrix<double, 3, 1>&);

void KfcoreIns::Correct(const Eigen::Matrix<float, kStateSize, 1>& error) {
  const StateVector delta = error.cast<double>();
  position_ += delta.segment<3>(kPosition);
  velocity_ += delta.segment<3>(kVelocity);
  rotation_ = Sophus::SO3d::exp(delta.segment<3>(kAttitude)) * rotation_;
  accelerometer_bias_ += delta.segment<3>(kAccelerometerBias);
  gyroscope_bias_ += delta.segment<3>(kGyroscopeBias);
}

KfcoreIns::Covariance KfcoreIns::covariance() const {
  const Eigen::Matrix<double, kStateSize, kStateSize> u = u_.cast<double>();
  return u * d_.cast<double>().asDiagonal() * u.transpose();
}

}  // namespace mapmind::fusion
