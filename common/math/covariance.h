// Copyright 2022 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#ifndef COMMON_MATH_COVARIANCE_H_
#define COMMON_MATH_COVARIANCE_H_

#include <Eigen/Dense>
#include <random>
#include <vector>
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

}  // namespace mapmind::math

#endif  // COMMON_MATH_COVARIANCE_H_
