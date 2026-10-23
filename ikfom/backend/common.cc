// Copyright 2022 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#include "ikfom/backend/common.h"

namespace mapmind::ikfom {

MTK::get_cov<ImuNoiseIkfom>::type GetProcessNoiseCov(double noise_acc, double noise_gyr,
                                                     double noise_bias_acc, double noise_bias_gyr) {
  MTK::get_cov<ImuNoiseIkfom>::type cov = MTK::get_cov<ImuNoiseIkfom>::type::Zero();
  // set imu noise
  MTK::setDiagonal<ImuNoiseIkfom, MTK::vect3, 0>(cov, &ImuNoiseIkfom::ng, noise_gyr);
  MTK::setDiagonal<ImuNoiseIkfom, MTK::vect3, 3>(cov, &ImuNoiseIkfom::na, noise_acc);
  // set imu bias
  MTK::setDiagonal<ImuNoiseIkfom, MTK::vect3, 6>(cov, &ImuNoiseIkfom::nbg, noise_bias_gyr);
  MTK::setDiagonal<ImuNoiseIkfom, MTK::vect3, 9>(cov, &ImuNoiseIkfom::nba, noise_bias_acc);
  return cov;
}

namespace {
Eigen::Matrix3d SquareDiagonal(const Eigen::Vector3d& vec) {
  Eigen::Array3d sqr_vec = vec.array() * vec.array();
  return sqr_vec.matrix().asDiagonal();
}
}  // namespace

MTK::get_cov<ImuNoiseIkfom>::type GetProcessNoiseCov(
    const mapmind::sensor::ImuIntrinsics& intrinsics) {
  MTK::get_cov<ImuNoiseIkfom>::type cov = MTK::get_cov<ImuNoiseIkfom>::type::Zero();
  cov.block(0, 0, 3, 3) = SquareDiagonal(intrinsics.gyroscope_noise);
  cov.block(3, 3, 3, 3) = SquareDiagonal(intrinsics.accelerometer_noise);
  cov.block(6, 6, 3, 3) = SquareDiagonal(intrinsics.gyroscope_bias_random_walk);
  cov.block(9, 9, 3, 3) = SquareDiagonal(intrinsics.accelerometer_bias_random_walk);
  return cov;
}

Eigen::VectorXd PoseMeasurementResidual(const Eigen::VectorXd& obs, const Eigen::VectorXd& raw) {
  Eigen::Matrix<double, 6, 1> result;
  result.segment<3>(0) = obs.segment<3>(0) - raw.segment<3>(0);
  result.segment<3>(3) =
      (Sophus::SO3d::exp(obs.segment<3>(3)) * Sophus::SO3d::exp(-raw.segment<3>(3))).log();
  return result;
}

Eigen::VectorXd RotationMeasurementResidual(const Eigen::VectorXd& obs,
                                            const Eigen::VectorXd& raw) {
  return (Sophus::SO3d::exp(obs) * Sophus::SO3d::exp(-raw)).log();
}

namespace {
constexpr double kFreeFallGravitySquared = 9.81 * 9.81 * 0.01;
constexpr double kThresholdPole = 0.1;
}  // namespace
// https://developer.android.com/reference/android/hardware/SensorManager#getRotationMatrix(float[],%20float[],%20float[],%20float[])
// https://android.googlesource.com/platform/frameworks/base/+/master/core/java/android/hardware/SensorManager.java
bool RotationFromMagneticGravity(const Eigen::Vector3d& gravity, const Eigen::Vector3d& magnetic,
                                 Eigen::Matrix3d* rotation, Eigen::Matrix3d* inclination) {
  double squared_norm_g = gravity.squaredNorm();
  if (squared_norm_g < kFreeFallGravitySquared) {  // gravity less than 10% of normal value
    return false;
  }
  Eigen::Vector3d vec_h = magnetic.cross(gravity);
  double vec_h_norm = vec_h.norm();
  if (vec_h_norm < kThresholdPole) {
    // device is close to free fall (or in space?), or close to
    // magnetic north pole. Typical values are > 100.
    return false;
  }
  Eigen::Vector3d un_h = (1.0 / vec_h_norm) * vec_h;
  Eigen::Vector3d un_gravity = (1.0 / sqrt(squared_norm_g)) * gravity;

  Eigen::Vector3d mag = un_gravity.cross(un_h);
  if (rotation != nullptr) {
    rotation->block(0, 0, 1, 3) = un_h.transpose();
    rotation->block(1, 0, 1, 3) = mag.transpose();
    rotation->block(2, 0, 1, 3) = un_gravity.transpose();
  }

  if (inclination != nullptr) {
    // compute the inclination matrix by projecting the geomagnetic
    // vector onto the Z (gravity) and X (horizontal component
    // of geomagnetic vector) axes.
    Eigen::Vector3d un_magnetic = magnetic.normalized();
    float c = un_magnetic.dot(mag);
    float s = un_magnetic.dot(un_gravity);
    *inclination = Eigen::Matrix3d::Identity();
    (*inclination)(1, 1) = c;
    (*inclination)(2, 2) = c;
    (*inclination)(1, 2) = s;
    (*inclination)(2, 1) = -s;
  }

  return true;
}

double GetInclinationAngle(const Eigen::Matrix3d& inclination) {
  return atan2(inclination(1, 2), inclination(1, 1));
}

}  // namespace mapmind::ikfom
