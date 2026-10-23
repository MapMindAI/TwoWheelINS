// Copyright 2022 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#include "ikfom/common/covariance_transformation.h"

namespace mapmind::ikfom {

void GetRelativePoseMeasurementCovariance(const Sophus::SE3d& se3_w_b0,
                                          const Eigen::Matrix<double, 6, 6>& cov_w_b0,
                                          const Sophus::SE3d& se3_w_b1,
                                          const Eigen::Matrix<double, 6, 6>& cov_w_b1,
                                          Sophus::SE3d* measurement,
                                          Eigen::Matrix<double, 6, 6>* covariance) {
  CHECK(measurement != nullptr);
  CHECK(covariance != nullptr);

  *measurement = se3_w_b0.inverse() * se3_w_b1;
  Eigen::Matrix3d inv_right_jaco_r = math::sophus::InverseRightJacobian(measurement->so3().log());
  Eigen::Matrix3d mea_rot_t = measurement->so3().matrix().transpose();
  Eigen::Matrix3d pose_0_rot_t = se3_w_b0.so3().matrix().transpose();

  Eigen::Matrix<double, 6, 6> jacobian0 = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::MatrixXd jaco_rot0 = math::sophus::RightJacobian(se3_w_b0.so3().log());
  jacobian0.block(0, 0, 3, 3) = -pose_0_rot_t;
  jacobian0.block(0, 3, 3, 3) = Sophus::SO3d::hat(measurement->translation()) * jaco_rot0;
  jacobian0.block(3, 3, 3, 3) = -inv_right_jaco_r * mea_rot_t * jaco_rot0;

  Eigen::Matrix<double, 6, 6> jacobian1 = Eigen::Matrix<double, 6, 6>::Zero();
  jacobian1.block(0, 0, 3, 3) = pose_0_rot_t;
  jacobian1.block(3, 3, 3, 3) =
      inv_right_jaco_r * math::sophus::RightJacobian(se3_w_b1.so3().log());

  *covariance =
      jacobian0 * cov_w_b0 * jacobian0.transpose() + jacobian1 * cov_w_b1 * jacobian1.transpose();
}

void GetPoseAdditionMeasurementCovariance(const Sophus::SE3d& se3_w_b0,
                                          const Eigen::Matrix<double, 6, 6>& cov_w_b0,
                                          const Sophus::SE3d& se3_b0_b1,
                                          const Eigen::Matrix<double, 6, 6>& cov_b0_b1,
                                          Sophus::SE3d* measurement,
                                          Eigen::Matrix<double, 6, 6>* covariance) {
  CHECK(measurement != nullptr);
  CHECK(covariance != nullptr);

  *measurement = se3_w_b0 * se3_b0_b1;
  Eigen::Matrix3d inv_right_jaco_m = math::sophus::InverseRightJacobian(measurement->so3().log());
  Eigen::Matrix3d rot_b0_b1_t = se3_b0_b1.so3().matrix().transpose();
  Eigen::MatrixXd jaco_rot_w_b0 = math::sophus::RightJacobian(se3_w_b0.so3().log());

  Eigen::Matrix<double, 6, 6> jacobian_1 = Eigen::Matrix<double, 6, 6>::Zero();

  jacobian_1.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
  jacobian_1.block(0, 3, 3, 3) =
      -se3_w_b0.so3().matrix() * Sophus::SO3d::hat(se3_b0_b1.translation()) * jaco_rot_w_b0;
  jacobian_1.block(3, 3, 3, 3) = inv_right_jaco_m * rot_b0_b1_t * jaco_rot_w_b0;

  Eigen::Matrix<double, 6, 6> jacobian_2 = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::MatrixXd jaco_rot_b0_b1 = math::sophus::RightJacobian(se3_b0_b1.so3().log());

  jacobian_2.block(0, 0, 3, 3) = se3_w_b0.so3().matrix();
  jacobian_2.block(3, 3, 3, 3) = inv_right_jaco_m * jaco_rot_b0_b1;

  *covariance = jacobian_1 * cov_w_b0 * jacobian_1.transpose() +
                jacobian_2 * cov_b0_b1 * jacobian_2.transpose();
}

// measurement of a extrinsic transformed pose
// T_w_b = T_w_c * T_c_b
void GetPoseWithExtrinsicMeasurementCovariance(const Sophus::SE3d& se3_w_c,
                                               const Eigen::Matrix<double, 6, 6>& cov_w_c,
                                               const Sophus::SE3d& se3_c_b,
                                               Sophus::SE3d* measurement,
                                               Eigen::Matrix<double, 6, 6>* covariance) {
  CHECK(measurement != nullptr);
  CHECK(covariance != nullptr);

  *measurement = se3_w_c * se3_c_b;
  Eigen::Matrix3d inv_right_jaco_m = math::sophus::InverseRightJacobian(measurement->so3().log());
  Eigen::Matrix3d rot_c_b_t = se3_c_b.so3().matrix().transpose();
  Eigen::MatrixXd jaco_rot_wc = math::sophus::RightJacobian(se3_w_c.so3().log());

  Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero();

  jacobian.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
  jacobian.block(0, 3, 3, 3) =
      -se3_w_c.so3().matrix() * Sophus::SO3d::hat(se3_c_b.translation()) * jaco_rot_wc;
  jacobian.block(3, 3, 3, 3) = inv_right_jaco_m * rot_c_b_t * jaco_rot_wc;
  *covariance = jacobian * cov_w_c * jacobian.transpose();
}

// T_w2_b = T_w2_w1 * T_w1_b
void GetPoseWorldChangeMeasurementCovariance(const Sophus::SE3d& se3_w1_b,
                                             const Eigen::Matrix<double, 6, 6>& cov_w1_b,
                                             const Sophus::SE3d& se3_w2_w1,
                                             Sophus::SE3d* measurement,
                                             Eigen::Matrix<double, 6, 6>* covariance) {
  CHECK(measurement != nullptr);
  CHECK(covariance != nullptr);

  *measurement = se3_w2_w1 * se3_w1_b;
  Eigen::Matrix3d inv_right_jaco_m = math::sophus::InverseRightJacobian(measurement->so3().log());
  Eigen::MatrixXd jaco_rot_w1b = math::sophus::RightJacobian(se3_w1_b.so3().log());
  Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero();

  jacobian.block(0, 0, 3, 3) = se3_w2_w1.so3().matrix();
  jacobian.block(3, 3, 3, 3) = inv_right_jaco_m * jaco_rot_w1b;
  *covariance = jacobian * cov_w1_b * jacobian.transpose();
}

// measurement from global localization : T_wl = T_wc * T_cb * T_lb^T
void GetGlobalLocalizationMeasurementCovariance(
    const Sophus::SE3d& se3_wc, const Sophus::SE3d& se3_cb, const Sophus::SE3d& se3_lb,
    const Eigen::Matrix<double, 6, 6>& cov_wc, const Eigen::Matrix<double, 6, 6>& cov_lb,
    Sophus::SE3d* measure_wl, Eigen::Matrix<double, 6, 6>* covariance_wl) {
  CHECK(measure_wl != nullptr);
  CHECK(covariance_wl != nullptr);

  const Sophus::SE3d se3_cl = se3_cb * se3_lb.inverse();
  *measure_wl = se3_wc * se3_cl;

  Eigen::Matrix3d inv_right_jaco_wl = math::sophus::InverseRightJacobian(measure_wl->so3().log());

  Eigen::MatrixXd right_jaco_wc = math::sophus::RightJacobian(se3_wc.so3().log());
  Eigen::MatrixXd right_jaco_lb = math::sophus::RightJacobian(se3_lb.so3().log());

  // jacobian of the result w.r.t T_wc
  Eigen::Matrix<double, 6, 6> jacobian_wc;
  // translation_res w.r.t translation_wc
  jacobian_wc.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
  // translation_res w.r.t rotation_wc
  jacobian_wc.block(0, 3, 3, 3) =
      -se3_wc.so3().matrix() * Sophus::SO3d::hat(se3_cl.translation()) * right_jaco_wc;
  // rotation_res w.r.t translation_wc
  jacobian_wc.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
  // rotation_res w.r.t rotation_wc
  jacobian_wc.block(3, 3, 3, 3) =
      inv_right_jaco_wl * se3_cl.so3().matrix().transpose() * right_jaco_wc;

  // jacobian of the result w.r.t T_lb
  Eigen::Matrix<double, 6, 6> jacobian_lb;
  // translation_res w.r.t translation_lb
  jacobian_lb.block(0, 0, 3, 3) = -measure_wl->so3().matrix();
  // translation_res w.r.t rotation_lb
  jacobian_lb.block(0, 3, 3, 3) =
      (se3_wc.so3() * se3_cb.so3()).matrix() *
      Sophus::SO3d::hat(-(se3_lb.so3().inverse() * se3_lb.translation())) * right_jaco_lb;
  // rotation_res w.r.t translation_lb
  jacobian_lb.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
  // rotation_res w.r.t rotation_lb
  jacobian_lb.block(3, 3, 3, 3) = -inv_right_jaco_wl * se3_lb.so3().matrix() * right_jaco_lb;

  *covariance_wl = jacobian_wc * cov_wc * jacobian_wc.transpose() +
                   jacobian_lb * cov_lb * jacobian_lb.transpose();
}

void GetGlobalLocalizationMeasurementCovariance(
    const Sophus::SE3d& se3_w_c0, const Eigen::Matrix<double, 6, 6>& cov_w_c0,
    const Sophus::SE3d& se3_w_b0, const Eigen::Matrix<double, 6, 6>& cov_w_b0,
    const Sophus::SE3d& se3_w_b1, const Eigen::Matrix<double, 6, 6>& cov_w_b1,
    const Sophus::SE3d& se3_c_b, Sophus::SE3d* measurement,
    Eigen::Matrix<double, 6, 6>* covariance) {
  CHECK(measurement != nullptr);
  CHECK(covariance != nullptr);

  // T_w_b1_global = T_w_c0 * T_c_b * (T_w_b1^T * T_w_b0)^T
  // compute T_b1_b0 & covariance
  Sophus::SE3d se3_b1_b0;
  Eigen::Matrix<double, 6, 6> cov_b1_b0;
  GetRelativePoseMeasurementCovariance(se3_w_b1, cov_w_b1, se3_w_b0, cov_w_b0, &se3_b1_b0,
                                       &cov_b1_b0);

  // apply the upper function T_wl = T_wc * T_cb * T_lb^T
  GetGlobalLocalizationMeasurementCovariance(se3_w_c0, se3_c_b, se3_b1_b0, cov_w_c0, cov_b1_b0,
                                             measurement, covariance);
}

}  // namespace mapmind::ikfom
