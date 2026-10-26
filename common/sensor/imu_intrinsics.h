// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_SENSOR_IMU_INTRINSICS_H_
#define COMMON_SENSOR_IMU_INTRINSICS_H_

#include <Eigen/Core>

namespace mapmind::sensor {

struct ImuIntrinsics {
  static ImuIntrinsics Default();

  double frequency;
  // accelerometer white noise density (m/s^2/sqrt(Hz))
  Eigen::Vector3d accelerometer_noise;
  // accelerometer bias random walk (m/s^3/sqrt(Hz))
  Eigen::Vector3d accelerometer_bias_random_walk;
  // gyroscope white noise density (rad/s/sqrt(Hz))
  Eigen::Vector3d gyroscope_noise;
  // gyroscope bias random walk (rad/s^2/sqrt(Hz))
  Eigen::Vector3d gyroscope_bias_random_walk;
};

}  // namespace mapmind::sensor

#endif  // COMMON_SENSOR_IMU_INTRINSICS_H_
