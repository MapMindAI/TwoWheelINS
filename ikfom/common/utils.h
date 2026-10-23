// Copyright 2022 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#ifndef IKFOM_COMMON_UTILS_H_
#define IKFOM_COMMON_UTILS_H_

#include <optional>
#include <ostream>
#include <vector>

#include "common/sensor/imu_data.h"
#include "common/sensor/imu_intrinsics.h"
#include "ikfom/toolkit/esekfom/esekfom.hpp"

#include "sophus/se3.hpp"

template <typename Scalar>
std::ostream& operator<<(std::ostream& os, const Sophus::SE3<Scalar>& se3) {
  return os << "{ t: [" << se3.translation().transpose() << "], q: ["
            << se3.so3().unit_quaternion().coeffs().transpose() << "] }";
}

namespace mapmind::ikfom::utils {

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

}  // namespace mapmind::ikfom::utils

#endif  // IKFOM_COMMON_UTILS_H_
