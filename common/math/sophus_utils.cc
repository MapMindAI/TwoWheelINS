// Copyright 2026 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#include "common/math/sophus_utils.h"

namespace mapmind::math::sophus {

Eigen::Matrix3d RightJacobian(const Eigen::Vector3d& phi) { return LeftJacobian(-phi); }

Eigen::Matrix3d InverseRightJacobian(const Eigen::Vector3d& phi) {
  return InverseLeftJacobian(-phi);
}

Eigen::Matrix3d LeftJacobian(const Eigen::Vector3d& phi) { return Sophus::SO3d::leftJacobian(phi); }

Eigen::Matrix3d InverseLeftJacobian(const Eigen::Vector3d& phi) {
  return Sophus::SO3d::leftJacobianInverse(phi);
}

Eigen::Matrix<double, 6, 1> Log(const Sophus::SE3d& pose) {
  Eigen::Matrix<double, 6, 1> xi;
  xi.head<3>() = pose.translation();
  xi.tail<3>() = pose.so3().log();
  return xi;
}

Sophus::SE3d Exp(const Eigen::Matrix<double, 6, 1>& xi) {
  return {Sophus::SO3d::exp(xi.tail<3>()), xi.head<3>()};
}

}  // namespace mapmind::math::sophus
