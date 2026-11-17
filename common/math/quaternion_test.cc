// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/math/quaternion.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "sophus/so3.hpp"

namespace mapmind::math {
namespace {

Eigen::Quaterniond ToEigen(const double q[4]) { return {q[0], q[1], q[2], q[3]}; }

TEST(Quaternion, MatchesSophus) {
  const Eigen::Vector3d vectors[] = {
      {0.3, -1.2, 2.0}, {1e-9, 2e-9, -1e-9}, {0.0, 0.0, 0.0}, {-0.5, 0.1, 0.05}};
  for (const Eigen::Vector3d& a_vector : vectors) {
    for (const Eigen::Vector3d& b_vector : vectors) {
      const Sophus::SO3d a = Sophus::SO3d::exp(a_vector);
      const Sophus::SO3d b = Sophus::SO3d::exp(b_vector);
      double qa[4];
      double qb[4];
      quaternion_exp(a_vector.data(), qa);
      quaternion_exp(b_vector.data(), qb);
      EXPECT_TRUE(ToEigen(qa).coeffs().isApprox(a.unit_quaternion().coeffs(), 1e-15));

      double product[4];
      quaternion_multiply(qa, qb, product);
      EXPECT_LT((Sophus::SO3d(ToEigen(product)).inverse() * a * b).log().norm(), 1e-14);
      // aliased output
      quaternion_multiply(qa, qb, qa);
      EXPECT_TRUE(ToEigen(qa).coeffs().isApprox(ToEigen(product).coeffs(), 1e-15));

      double matrix[9];
      quaternion_to_matrix(product, matrix);
      const Eigen::Matrix3d expected = (a * b).matrix();
      EXPECT_TRUE((Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(matrix) - expected)
                      .isZero(1e-14));

      double rotated[3];
      quaternion_rotate(product, b_vector.data(), rotated);
      EXPECT_TRUE((Eigen::Map<Eigen::Vector3d>(rotated) - (a * b) * b_vector).isZero(1e-14));
    }
  }
}

TEST(Quaternion, Normalizes) {
  double q[4] = {2.0, 0.0, -2.0, 1.0};
  quaternion_normalize(q);
  EXPECT_DOUBLE_EQ(ToEigen(q).norm(), 1.0);
  EXPECT_DOUBLE_EQ(q[0], 2.0 / 3.0);
  EXPECT_DOUBLE_EQ(q[2], -2.0 / 3.0);
}

}  // namespace
}  // namespace mapmind::math
