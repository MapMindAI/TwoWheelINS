// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_SENSOR_IMU_DATA_H_
#define COMMON_SENSOR_IMU_DATA_H_

#include <cstdint>
#include <optional>

#include <Eigen/Core>

namespace mapmind::sensor {

struct ImuData {
  int64_t timestamp_ns;
  // m/s^2, body frame
  Eigen::Vector3d linear_acceleration;
  // rad/s, body frame
  Eigen::Vector3d angular_velocity;
  // milligauss, body frame; only from IMUs with a magnetometer
  std::optional<Eigen::Vector3d> magnetic_field_mgauss = std::nullopt;
};

}  // namespace mapmind::sensor

#endif  // COMMON_SENSOR_IMU_DATA_H_
