// Copyright 2022 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#ifndef COMMON_SIMULATION_IMU_SIMULATION_H_
#define COMMON_SIMULATION_IMU_SIMULATION_H_

#include <optional>
#include <vector>

#include <Eigen/Core>

#include "common/sensor/imu_data.h"
#include "sophus/se3.hpp"

namespace mapmind::simulation {

struct ImuSimulation {
  Eigen::Vector3d bias_gyr;
  Eigen::Vector3d bias_acc;
  Eigen::Vector3d gravity;
  std::vector<mapmind::sensor::ImuData> imu_data;
  std::vector<Eigen::Vector3d> velocities;
  std::vector<Sophus::SE3d> poses;
  void Add(int64_t time, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr,
           const Eigen::Vector3d& velo, const Sophus::SE3d& pose) {
    imu_data.emplace_back(mapmind::sensor::ImuData{time, acc, gyr});
    velocities.emplace_back(velo);
    poses.emplace_back(pose);
  }
};

ImuSimulation SimulateImuData(double bias_acc = 1e-3, double bias_gyr = 1e-3,
                              double duration_second = 30, int imu_fps = 100,
                              std::optional<Eigen::Vector3d> gravity = std::nullopt);

}  // namespace mapmind::simulation

#endif  // COMMON_SIMULATION_IMU_SIMULATION_H_
