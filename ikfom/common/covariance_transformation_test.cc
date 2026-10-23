// Copyright 2022 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#include "ikfom/common/covariance_transformation.h"

#include <gtest/gtest.h>
#include "common/math/covariance_test_utils.h"

namespace mapmind::ikfom {

inline Eigen::Matrix<double, 6, 6> RandomCovariance(double level) {
  Eigen::Matrix<double, 6, 6> matrix_tmp = level * Eigen::Matrix<double, 6, 6>::Random();
  return math::testing_utils::MakeMatrixPositiveDefinite<double, 6>(matrix_tmp +
                                                                    matrix_tmp.transpose());
}

inline Eigen::Matrix<double, 6, 1> ComputeResultOdometry(
    const Eigen::Matrix<double, 12, 1>& pose0_pose1) {
  return math::sophus::Log(math::sophus::Exp(pose0_pose1.segment<6>(0)).inverse() *
                           math::sophus::Exp(pose0_pose1.segment<6>(6)));
}

TEST(CovarianceTransformation, Odoemtry) {
  Sophus::SE3d pose0(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());
  Sophus::SE3d pose1(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());

  Eigen::Matrix<double, 12, 1> pose0_pose1;
  pose0_pose1.segment<6>(0) = math::sophus::Log(pose0);
  pose0_pose1.segment<6>(6) = math::sophus::Log(pose1);

  Eigen::Matrix<double, 12, 12> covariance_pose0_pose1 = Eigen::Matrix<double, 12, 12>::Zero();
  covariance_pose0_pose1.block(0, 0, 6, 6) = RandomCovariance(1e-4);
  covariance_pose0_pose1.block(6, 6, 6, 6) = RandomCovariance(1e-4);

  // compare the covariance result
  Sophus::SE3d measurement;
  Eigen::Matrix<double, 6, 6> cov_residual_algo;
  GetRelativePoseMeasurementCovariance(pose0, covariance_pose0_pose1.block(0, 0, 6, 6), pose1,
                                       covariance_pose0_pose1.block(6, 6, 6, 6), &measurement,
                                       &cov_residual_algo);

  // simulate the residual covariance
  Eigen::Matrix<double, 6, 6> cov_residual_sim =
      math::testing_utils::StateTransitionCovariance<double, 12, 6>(
          pose0_pose1, covariance_pose0_pose1, ComputeResultOdometry, 1e5);

  // check the relative difference
  auto diff_mat = cov_residual_algo - cov_residual_sim;
  EXPECT_NEAR(diff_mat.norm() / cov_residual_sim.norm(), 0.0, 0.01);
}

inline Eigen::Matrix<double, 6, 1> ComputeResultPoseExtrinsic(
    const Sophus::SE3d extrinsic, const Eigen::Matrix<double, 6, 1>& pose) {
  return math::sophus::Log(math::sophus::Exp(pose) * extrinsic);
}

TEST(CovarianceTransformation, PoseExtrinsic) {
  Sophus::SE3d pose0(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());
  Sophus::SE3d extrinsic(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());

  Eigen::Matrix<double, 6, 1> pose0_array = math::sophus::Log(pose0);
  Eigen::Matrix<double, 6, 6> covariance_pose0 = RandomCovariance(1e-4);

  // compare the covariance result
  Sophus::SE3d measurement;
  Eigen::Matrix<double, 6, 6> cov_residual_algo;
  GetPoseWithExtrinsicMeasurementCovariance(pose0, covariance_pose0, extrinsic, &measurement,
                                            &cov_residual_algo);

  // simulate the residual covariance
  Eigen::Matrix<double, 6, 6> cov_residual_sim =
      math::testing_utils::StateTransitionCovariance<double, 6, 6>(
          pose0_array, covariance_pose0,
          std::bind(&ComputeResultPoseExtrinsic, extrinsic, std::placeholders::_1), 1e5);

  // check the relative difference
  auto diff_mat = cov_residual_algo - cov_residual_sim;
  EXPECT_NEAR(diff_mat.norm() / cov_residual_sim.norm(), 0.0, 0.01);
}

inline Eigen::Matrix<double, 6, 1> ComputeResultPoseWorldChange(
    const Sophus::SE3d world_change, const Eigen::Matrix<double, 6, 1>& pose) {
  return math::sophus::Log(world_change * math::sophus::Exp(pose));
}

TEST(CovarianceTransformation, PoseWorldChange) {
  Sophus::SE3d pose0(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());
  Sophus::SE3d world_change(Sophus::SO3d::exp(Eigen::Vector3d::Random()),
                            Eigen::Vector3d::Random());

  Eigen::Matrix<double, 6, 1> pose0_array = math::sophus::Log(pose0);
  Eigen::Matrix<double, 6, 6> covariance_pose0 = RandomCovariance(1e-4);

  // compare the covariance result
  Sophus::SE3d measurement;
  Eigen::Matrix<double, 6, 6> cov_residual_algo;
  GetPoseWorldChangeMeasurementCovariance(pose0, covariance_pose0, world_change, &measurement,
                                          &cov_residual_algo);

  // simulate the residual covariance
  Eigen::Matrix<double, 6, 6> cov_residual_sim =
      math::testing_utils::StateTransitionCovariance<double, 6, 6>(
          pose0_array, covariance_pose0,
          std::bind(&ComputeResultPoseWorldChange, world_change, std::placeholders::_1), 1e5);

  // check the relative difference
  auto diff_mat = cov_residual_algo - cov_residual_sim;
  EXPECT_NEAR(diff_mat.norm() / cov_residual_sim.norm(), 0.0, 0.01);
}

inline Eigen::Matrix<double, 6, 1> ComputeResultPoseAddition(
    const Eigen::Matrix<double, 12, 1>& pose0_pose1) {
  return math::sophus::Log(math::sophus::Exp(pose0_pose1.segment<6>(0)) *
                           math::sophus::Exp(pose0_pose1.segment<6>(6)));
}

TEST(CovarianceTransformation, PoseAddition) {
  Sophus::SE3d pose0(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());
  Sophus::SE3d pose1(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());

  Eigen::Matrix<double, 12, 1> pose0_pose1;
  pose0_pose1.segment<6>(0) = math::sophus::Log(pose0);
  pose0_pose1.segment<6>(6) = math::sophus::Log(pose1);

  Eigen::Matrix<double, 12, 12> covariance_pose0_pose1 = Eigen::Matrix<double, 12, 12>::Zero();
  covariance_pose0_pose1.block(0, 0, 6, 6) = RandomCovariance(1e-4);
  covariance_pose0_pose1.block(6, 6, 6, 6) = RandomCovariance(1e-4);

  // compare the covariance result
  Sophus::SE3d measurement;
  Eigen::Matrix<double, 6, 6> cov_residual_algo;
  GetPoseAdditionMeasurementCovariance(pose0, covariance_pose0_pose1.block(0, 0, 6, 6), pose1,
                                       covariance_pose0_pose1.block(6, 6, 6, 6), &measurement,
                                       &cov_residual_algo);

  // simulate the residual covariance
  Eigen::Matrix<double, 6, 6> cov_residual_sim =
      math::testing_utils::StateTransitionCovariance<double, 12, 6>(
          pose0_pose1, covariance_pose0_pose1, ComputeResultPoseAddition, 1e5);

  // check the relative difference
  auto diff_mat = cov_residual_algo - cov_residual_sim;
  EXPECT_NEAR(diff_mat.norm() / cov_residual_sim.norm(), 0.0, 0.01);
}

inline Eigen::Matrix<double, 6, 1> ComputeResultGlobalLocalization(
    const Sophus::SE3d& pose_cb, const Eigen::Matrix<double, 12, 1>& pose_wc_pose_lb) {
  return math::sophus::Log(math::sophus::Exp(pose_wc_pose_lb.segment<6>(0)) * pose_cb *
                           math::sophus::Exp(pose_wc_pose_lb.segment<6>(6)).inverse());
}

TEST(CovarianceTransformation, GlobalLocalization) {
  Sophus::SE3d pose_wc(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());
  Sophus::SE3d pose_cb(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());
  Sophus::SE3d pose_lb(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());

  Eigen::Matrix<double, 12, 1> pose_wc_pose_lb;
  pose_wc_pose_lb.segment<6>(0) = math::sophus::Log(pose_wc);
  pose_wc_pose_lb.segment<6>(6) = math::sophus::Log(pose_lb);

  Eigen::Matrix<double, 12, 12> covariance_pose_wc_pose_lb = Eigen::Matrix<double, 12, 12>::Zero();
  covariance_pose_wc_pose_lb.block(0, 0, 6, 6) = RandomCovariance(1e-4);
  covariance_pose_wc_pose_lb.block(6, 6, 6, 6) = RandomCovariance(1e-4);

  // compare the covariance result
  Sophus::SE3d result;
  Eigen::Matrix<double, 6, 6> covariance_algo;
  GetGlobalLocalizationMeasurementCovariance(
      pose_wc, pose_cb, pose_lb, covariance_pose_wc_pose_lb.block(0, 0, 6, 6),
      covariance_pose_wc_pose_lb.block(6, 6, 6, 6), &result, &covariance_algo);

  // simulate the residual covariance
  Eigen::Matrix<double, 6, 6> covariance_sim =
      math::testing_utils::StateTransitionCovariance<double, 12, 6>(
          pose_wc_pose_lb, covariance_pose_wc_pose_lb,
          std::bind(&ComputeResultGlobalLocalization, pose_cb, std::placeholders::_1), 1e5);

  // check the relative difference
  auto diff_mat = covariance_algo - covariance_sim;
  EXPECT_NEAR(diff_mat.norm() / covariance_sim.norm(), 0.0, 0.01);
}

inline Eigen::Matrix<double, 6, 1> ComputeResultDelayedGlobalLocalization(
    const Sophus::SE3d& pose_cb, const Eigen::Matrix<double, 18, 1>& pose_wc_pose_lb) {
  // T_w_b1_global = T_w_c0 * T_c_b * T_w_b0^T * T_w_b1
  return math::sophus::Log(math::sophus::Exp(pose_wc_pose_lb.segment<6>(0)) * pose_cb *
                           math::sophus::Exp(pose_wc_pose_lb.segment<6>(6)).inverse() *
                           math::sophus::Exp(pose_wc_pose_lb.segment<6>(12)));
}

TEST(CovarianceTransformation, DelayedGlobalLocalization) {
  Sophus::SE3d pose_wc(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());
  Sophus::SE3d pose_cb(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());
  Sophus::SE3d pose_wb0(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());
  Sophus::SE3d pose_wb1(Sophus::SO3d::exp(Eigen::Vector3d::Random()), Eigen::Vector3d::Random());

  Eigen::Matrix<double, 18, 1> pose_all;
  pose_all.segment<6>(0) = math::sophus::Log(pose_wc);
  pose_all.segment<6>(6) = math::sophus::Log(pose_wb0);
  pose_all.segment<6>(12) = math::sophus::Log(pose_wb1);

  Eigen::Matrix<double, 18, 18> covariance_all = Eigen::Matrix<double, 18, 18>::Zero();
  covariance_all.block(0, 0, 6, 6) = RandomCovariance(1e-4);
  covariance_all.block(6, 6, 6, 6) = RandomCovariance(1e-4);
  covariance_all.block(12, 12, 6, 6) = RandomCovariance(1e-4);

  // compare the covariance result
  Sophus::SE3d result;
  Eigen::Matrix<double, 6, 6> covariance_algo;
  GetGlobalLocalizationMeasurementCovariance(
      pose_wc, covariance_all.block(0, 0, 6, 6), pose_wb0, covariance_all.block(6, 6, 6, 6),
      pose_wb1, covariance_all.block(12, 12, 6, 6), pose_cb, &result, &covariance_algo);

  // simulate the residual covariance
  Eigen::Matrix<double, 6, 6> covariance_sim =
      math::testing_utils::StateTransitionCovariance<double, 18, 6>(
          pose_all, covariance_all,
          std::bind(&ComputeResultDelayedGlobalLocalization, pose_cb, std::placeholders::_1), 1e5);

  // check the relative difference
  auto diff_mat = covariance_algo - covariance_sim;
  EXPECT_NEAR(diff_mat.norm() / covariance_sim.norm(), 0.0, 0.01);
}

}  // namespace mapmind::ikfom
