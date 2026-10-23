// Copyright 2026 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#ifndef COMMON_MATH_SOPHUS_UTILS_H_
#define COMMON_MATH_SOPHUS_UTILS_H_

#include <ostream>

#include <Eigen/Core>

#include "sophus/se3.hpp"

template <typename Scalar>
std::ostream& operator<<(std::ostream& os, const Sophus::SE3<Scalar>& se3) {
  return os << "{ t: [" << se3.translation().transpose() << "], q: ["
            << se3.so3().unit_quaternion().coeffs().transpose() << "] }";
}

namespace mapmind::math::sophus {

// Jacobians on so3, as defined in section 7.1.5 of
// http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf
Eigen::Matrix3d RightJacobian(const Eigen::Vector3d& phi);
Eigen::Matrix3d InverseRightJacobian(const Eigen::Vector3d& phi);
Eigen::Matrix3d LeftJacobian(const Eigen::Vector3d& phi);
Eigen::Matrix3d InverseLeftJacobian(const Eigen::Vector3d& phi);

// xi is [translation, phi] (phi is the so3 of the rotation), which differs from
// Sophus::SE3d::log() / Sophus::SE3d::exp().
Eigen::Matrix<double, 6, 1> Log(const Sophus::SE3d& pose);
Sophus::SE3d Exp(const Eigen::Matrix<double, 6, 1>& xi);

}  // namespace mapmind::math::sophus

#endif  // COMMON_MATH_SOPHUS_UTILS_H_
