// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FUSION_KFCORE_INS_H_
#define FUSION_KFCORE_INS_H_

#include <Eigen/Core>

#include "sophus/so3.hpp"

namespace mapmind::fusion {

// 15-state error-state INS on the KFCore UDU filter, after INSLIB's design: the nominal state is
// integrated in double, the error covariance is kept as float U-D factors (P = U diag(d) U^T),
// predicted with Thornton's and updated with Bierman's algorithm. Gravity is constant along world
// -z; Earth rotation is ignored. Truth = nominal + error, with the attitude error in the world
// frame (rotation_true = exp(error) * rotation). Fixed-size storage only, no heap.
class KfcoreIns {
 public:
  enum StateIndex {
    kPosition = 0,
    kVelocity = 3,
    kAttitude = 6,
    kAccelerometerBias = 9,
    kGyroscopeBias = 12,
    kStateSize = 15,
  };
  using StateVector = Eigen::Matrix<double, kStateSize, 1>;
  using Covariance = Eigen::Matrix<double, kStateSize, kStateSize>;

  // Continuous-time noise densities.
  struct ProcessNoise {
    double accelerometer;       // m/s^2/sqrt(Hz)
    double gyroscope;           // rad/s/sqrt(Hz)
    double accelerometer_bias;  // m/s^3/sqrt(Hz)
    double gyroscope_bias;      // rad/s^2/sqrt(Hz)
  };

  KfcoreIns(const ProcessNoise& process_noise, double gravity);

  // Sets the nominal state (zero accelerometer bias) and a diagonal covariance from `initial_std`.
  void Reset(const Eigen::Vector3d& position, const Sophus::SO3d& rotation,
             const Eigen::Vector3d& velocity, const Eigen::Vector3d& gyroscope_bias,
             const StateVector& initial_std);

  // Strapdown step over `dt` (s) with body-frame specific force (m/s^2) and angular rate (rad/s).
  void Predict(const Eigen::Vector3d& acceleration, const Eigen::Vector3d& angular_velocity,
               double dt);

  // Fuses `residual` = jacobian * error + noise with independent noise of `variance`, then
  // applies the error to the nominal state. Returns false if KFCore rejected a row.
  template <int kRows>
  bool Update(const Eigen::Matrix<double, kRows, 1>& residual,
              const Eigen::Matrix<double, kRows, kStateSize>& jacobian,
              const Eigen::Matrix<double, kRows, 1>& variance);

  Covariance covariance() const;
  // body to world
  const Sophus::SO3d& rotation() const { return rotation_; }
  const Eigen::Vector3d& position() const { return position_; }
  const Eigen::Vector3d& velocity() const { return velocity_; }
  const Eigen::Vector3d& accelerometer_bias() const { return accelerometer_bias_; }
  const Eigen::Vector3d& gyroscope_bias() const { return gyroscope_bias_; }

 private:
  void Correct(const Eigen::Matrix<float, kStateSize, 1>& error);

  ProcessNoise process_noise_;
  Eigen::Vector3d gravity_;

  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  Sophus::SO3d rotation_;
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerometer_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyroscope_bias_ = Eigen::Vector3d::Zero();

  // U-D factors, column-major as KFCore expects
  Eigen::Matrix<float, kStateSize, kStateSize> u_ =
      Eigen::Matrix<float, kStateSize, kStateSize>::Identity();
  Eigen::Matrix<float, kStateSize, 1> d_ = Eigen::Matrix<float, kStateSize, 1>::Ones();
};

}  // namespace mapmind::fusion

#endif  // FUSION_KFCORE_INS_H_
