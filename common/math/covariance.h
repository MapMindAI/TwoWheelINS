// Copyright 2022 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#ifndef COMMON_MATH_COVARIANCE_H_
#define COMMON_MATH_COVARIANCE_H_

#include <Eigen/Dense>
#include <random>
#include <vector>
#include "common/math/sophus_utils.h"
#include "glog/logging.h"

namespace mapmind::math {

// given data, compute its covariance matrix
// https://en.wikipedia.org/wiki/Covariance_matrix
template <typename Scalar, size_t DIM>
Eigen::Matrix<Scalar, DIM, DIM> ComputeCovarianceMatrix(
    const std::vector<Eigen::Matrix<Scalar, DIM, 1>>& data) {
  CHECK(!data.empty());
  if (data.size() == 1) {
    // we cannot compute properly the covariance without enough data
    return Eigen::Matrix<Scalar, DIM, DIM>::Zero();
  }
  const auto data_matrix = Eigen::Map<const Eigen::Matrix<Scalar, DIM, Eigen::Dynamic>>(
      data[0].data(), DIM, data.size());
  Eigen::Matrix<Scalar, DIM, 1> average = data_matrix.rowwise().mean();
  auto recentered_matrix = data_matrix.colwise() - average;
  return (1.0 / (data.size() - 1)) * (recentered_matrix * recentered_matrix.transpose());
}

// measurement of relative pose : T_odom = pose0.inverse() * pose1
// usage : for some odometry with pose covariance
void GetRelativePoseMeasurementCovariance(const Sophus::SE3d& se3_w_b0,
                                          const Eigen::Matrix<double, 6, 6>& cov_w_b0,
                                          const Sophus::SE3d& se3_w_b1,
                                          const Eigen::Matrix<double, 6, 6>& cov_w_b1,
                                          Sophus::SE3d* measurement,
                                          Eigen::Matrix<double, 6, 6>* covariance);

// measurement of relative pose : T_addition = pose0 * pose1
// usage : for some odometry with pose covariance
void GetPoseAdditionMeasurementCovariance(const Sophus::SE3d& se3_w_b0,
                                          const Eigen::Matrix<double, 6, 6>& cov_w_b0,
                                          const Sophus::SE3d& se3_b0_b1,
                                          const Eigen::Matrix<double, 6, 6>& cov_b0_b1,
                                          Sophus::SE3d* measurement,
                                          Eigen::Matrix<double, 6, 6>* covariance);

// measurement of a extrinsic transformed pose
// T_w_b = T_w_c * T_c_b
void GetPoseWithExtrinsicMeasurementCovariance(const Sophus::SE3d& se3_w_c,
                                               const Eigen::Matrix<double, 6, 6>& cov_w_c,
                                               const Sophus::SE3d& se3_c_b,
                                               Sophus::SE3d* measurement,
                                               Eigen::Matrix<double, 6, 6>* covariance);

// T_w2_b = T_w2_w1 * T_w1_b
void GetPoseWorldChangeMeasurementCovariance(const Sophus::SE3d& se3_w1_b,
                                             const Eigen::Matrix<double, 6, 6>& cov_w1_b,
                                             const Sophus::SE3d& se3_w2_w1,
                                             Sophus::SE3d* measurement,
                                             Eigen::Matrix<double, 6, 6>* covariance);

// measurement of transformation from the local coordinate to global map world
// T_wl = T_wc * T_cb * T_lb^T (l:local, w:global map world)
// use PoseMeasurementResidual along with the following functions
void GetGlobalLocalizationMeasurementCovariance(
    const Sophus::SE3d& se3_wc, const Sophus::SE3d& se3_cb, const Sophus::SE3d& se3_lb,
    const Eigen::Matrix<double, 6, 6>& cov_wc, const Eigen::Matrix<double, 6, 6>& cov_lb,
    Sophus::SE3d* measure_wl, Eigen::Matrix<double, 6, 6>* covariance_wl);

// measurement of the delayed global localization pose to get current pose
// transform the old global localization covariance (T_w_c0) to the current pose (T_w_b1_global)
// T_w_b1_global = T_w_c0 * T_c_b * (T_w_b1^T * T_w_b0)^T
void GetGlobalLocalizationMeasurementCovariance(
    const Sophus::SE3d& se3_w_c0, const Eigen::Matrix<double, 6, 6>& cov_w_c0,
    const Sophus::SE3d& se3_w_b0, const Eigen::Matrix<double, 6, 6>& cov_w_b0,
    const Sophus::SE3d& se3_w_b1, const Eigen::Matrix<double, 6, 6>& cov_w_b1,
    const Sophus::SE3d& se3_c_b, Sophus::SE3d* measurement,
    Eigen::Matrix<double, 6, 6>* covariance);

}  // namespace mapmind::math

#endif  // COMMON_MATH_COVARIANCE_H_
