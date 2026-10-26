// Copyright 2022 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_MATH_COVARIANCE_TEST_UTILS_H_
#define COMMON_MATH_COVARIANCE_TEST_UTILS_H_

#include <functional>
#include <vector>

#include "common/math/covariance.h"

namespace mapmind::math::testing_utils {

// given a random matrix, make it positive definite
// (i.e. make its eigenvalues positive) so it will be a proper covariance matrix
template <typename Scalar, size_t DIM>
Eigen::Matrix<Scalar, DIM, DIM> MakeMatrixPositiveDefinite(
    const Eigen::Matrix<Scalar, DIM, DIM>& matrix) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, DIM, DIM>> eigen_solver(matrix);
  Eigen::Matrix<Scalar, DIM, 1> abs_eigen_values = eigen_solver.eigenvalues().cwiseAbs();
  Eigen::Matrix<Scalar, DIM, DIM> transformation = eigen_solver.eigenvectors();
  return transformation * abs_eigen_values.asDiagonal() * transformation.transpose();
}

template <typename Scalar, size_t DIM>
std::vector<Eigen::Matrix<Scalar, DIM, 1>> SimulateDataWithCovariance(
    const Eigen::Matrix<Scalar, DIM, 1>& average, const Eigen::Matrix<Scalar, DIM, DIM>& covariance,
    size_t num = 100000) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, DIM, DIM>> eigen_solver(covariance);
  Eigen::Matrix<Scalar, DIM, 1> eigen_values = eigen_solver.eigenvalues();
  Eigen::Matrix<Scalar, DIM, DIM> transformation = eigen_solver.eigenvectors();

  static std::default_random_engine engine(2022);
  std::vector<std::normal_distribution<Scalar>> distributions;
  for (size_t i = 0; i < DIM; i++) {
    CHECK_GT(eigen_values(i, 0), 0.0);
    distributions.emplace_back(0, sqrt(eigen_values(i, 0)));
  }

  std::vector<Eigen::Matrix<Scalar, DIM, 1>> result;
  result.reserve(num);
  Eigen::Matrix<Scalar, DIM, 1> data;
  for (size_t i = 0; i < num; i++) {
    for (size_t k = 0; k < DIM; k++) {
      data(k, 0) = (distributions[k])(engine);
    }
    result.emplace_back(average + transformation * data);
  }
  return result;
}

template <typename Scalar, int DIM_FROM, int DIM_TO>
using StateTransitionFunction =
    std::function<Eigen::Matrix<Scalar, DIM_TO, 1>(const Eigen::Matrix<Scalar, DIM_FROM, 1>& data)>;

template <typename Scalar, int DIM_FROM, int DIM_TO>
Eigen::Matrix<Scalar, DIM_TO, DIM_TO> StateTransitionCovariance(
    const Eigen::Matrix<Scalar, DIM_FROM, 1>& average,
    const Eigen::Matrix<Scalar, DIM_FROM, DIM_FROM>& covariance,
    const StateTransitionFunction<Scalar, DIM_FROM, DIM_TO>& transition_fcn, size_t num = 100000) {
  // simulate from data
  std::vector<Eigen::Matrix<Scalar, DIM_FROM, 1>> from_data =
      SimulateDataWithCovariance<Scalar, DIM_FROM>(average, covariance, num);
  // make the new data
  std::vector<Eigen::Matrix<Scalar, DIM_TO, 1>> to_data;
  to_data.reserve(from_data.size());
  for (const auto& data : from_data) {
    to_data.emplace_back(transition_fcn(data));
  }
  return ComputeCovarianceMatrix<Scalar, DIM_TO>(to_data);
}

}  // namespace mapmind::math::testing_utils

#endif  // COMMON_MATH_COVARIANCE_TEST_UTILS_H_
