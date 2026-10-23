// Copyright 2026 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#include "ikfom/backend/ikf_ins.h"

#include <algorithm>

#include <gtest/gtest.h>
#include "common/simulation/imu_simulation.h"

namespace mapmind::ikfom::ins {

using Filter = esekfom::esekf<StateIkfom, ImuNoiseIkfom::DOF, ImuIkfom>;

// Propagate on simulated IMU and correct with a 10 Hz pose measurement (a stand-in for GPS +
// heading); the estimate must track the ground-truth trajectory.
TEST(IkfLio, TracksSimulatedTrajectoryWithPoseUpdates) {
  constexpr int kImuFps = 100;
  constexpr int kPoseUpdateEvery = 10;
  const simulation::ImuSimulation simulation =
      simulation::SimulateImuData(0.0, 0.0, 10.0, kImuFps, Eigen::Vector3d(0, 0, -9.8099));

  Filter filter;
  double limit[StateIkfom::DOF];
  std::fill(limit, limit + StateIkfom::DOF, 1e-3);
  filter.init_dyn(GetVelocityField, DVelocityFieldDx, DVelocityFieldDw, 3, limit);
  filter.add_dyn_vec(PoseMeasurement, DPoseMeasurementDx, DPoseMeasurementDv,
                     PoseMeasurementResidual);

  StateIkfom& state = filter.mutable_state();
  state.pos = simulation.poses.front().translation();
  state.rot = simulation.poses.front().so3().unit_quaternion();
  state.vel = simulation.velocities.front();
  state.grav = MTK::Gravity(simulation.gravity);
  filter.mutable_covariance() =
      1e-4 * Eigen::Matrix<double, StateIkfom::DOF, StateIkfom::DOF>::Identity();

  const auto process_noise = GetProcessNoiseCov(1e-4, 1e-4, 1e-6, 1e-6);
  const Eigen::MatrixXd pose_noise = 1e-4 * Eigen::MatrixXd::Identity(6, 6);
  const double dt = 1.0 / kImuFps;

  for (size_t i = 1; i < simulation.imu_data.size(); ++i) {
    ImuIkfom input;
    input.acc = simulation.imu_data[i].linear_acceleration;
    input.gyro = simulation.imu_data[i].angular_velocity;
    filter.predict(dt, process_noise, input);

    if (i % kPoseUpdateEvery == 0) {
      const Sophus::SE3d& truth = simulation.poses[i];
      Eigen::VectorXd observation(6);
      observation << truth.translation(), truth.so3().log();
      ASSERT_TRUE(filter.update_iterated_dyn(observation, pose_noise));
    }
  }

  const Sophus::SE3d& truth = simulation.poses.back();
  const StateIkfom& estimate = filter.get_x();
  EXPECT_LT((estimate.pos - truth.translation()).norm(), 0.05);
  EXPECT_LT((Sophus::SO3d(estimate.rot).inverse() * truth.so3()).log().norm(), 0.01);
  EXPECT_LT((estimate.vel - simulation.velocities.back()).norm(), 0.1);
}

}  // namespace mapmind::ikfom::ins
