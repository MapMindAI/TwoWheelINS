// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_SENSOR_IMU_DATA_H_
#define COMMON_SENSOR_IMU_DATA_H_

#include <cstdint>

#include <Eigen/Core>

namespace mapmind::sensor {

struct ImuData {
  int64_t timestamp_ns;
  // m/s^2, body frame
  Eigen::Vector3d linear_acceleration;
  // rad/s, body frame
  Eigen::Vector3d angular_velocity;
};

}  // namespace mapmind::sensor

#endif  // COMMON_SENSOR_IMU_DATA_H_
