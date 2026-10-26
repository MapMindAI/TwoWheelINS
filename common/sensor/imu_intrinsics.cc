// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/sensor/imu_intrinsics.h"

namespace mapmind::sensor {

ImuIntrinsics ImuIntrinsics::Default() {
  ImuIntrinsics intrinsics;
  intrinsics.frequency = 100;
  intrinsics.accelerometer_noise = Eigen::Vector3d::Constant(3.9939570888238808e-3);
  intrinsics.accelerometer_bias_random_walk = Eigen::Vector3d::Constant(2.03492e-5);
  intrinsics.gyroscope_noise = Eigen::Vector3d::Constant(1.5636343949698187e-3);
  intrinsics.gyroscope_bias_random_walk = Eigen::Vector3d::Constant(1.12704e-5);
  return intrinsics;
}

}  // namespace mapmind::sensor
