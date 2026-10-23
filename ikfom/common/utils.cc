// Copyright 2022 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#include "ikfom/common/utils.h"

namespace mapmind::ikfom::utils {

ImuSimulation SimulateImuData(double bias_acc, double bias_gyr, double duration_second, int imu_fps,
                              std::optional<Eigen::Vector3d> gravity) {
  // simulate 30 seconds of data
  int simulate_imu_num = imu_fps * duration_second;
  int64_t time_interval = 1e9 / imu_fps;
  double time_interval_sec = 1.0 / imu_fps;
  double acc_change_rate = 10.0 / imu_fps;
  double gyr_change_rate = 2.0 / imu_fps;

  ImuSimulation result;
  {
    result.bias_gyr = bias_gyr * Eigen::Vector3d::Random();
    result.bias_acc = bias_acc * Eigen::Vector3d::Random();
    if (!gravity) {
      Eigen::Vector3d tmp = Eigen::Vector3d::Random();
      result.gravity = 9.8099 * tmp.normalized();
    } else {
      result.gravity = gravity.value();
    }
  }

  // buffer the current state
  int64_t current_time = 1631171635250333786;
  Eigen::Vector3d curr_acc_world = result.gravity;
  Eigen::Vector3d curr_gyr = Eigen::Vector3d::Zero();
  Eigen::Vector3d curr_velo = Eigen::Vector3d::Zero();
  Sophus::SE3d curr_pose;
  result.Add(current_time,
             curr_pose.so3().inverse() * (curr_acc_world - result.gravity) + result.bias_acc,
             curr_gyr + result.bias_gyr, curr_velo, curr_pose);

  for (int iter = 0; iter < simulate_imu_num; iter++) {
    current_time += time_interval;

    // get imu data (linearly changing, reference real data to get the factor)
    Eigen::Vector3d curr_gyr_t = curr_gyr;
    Eigen::Vector3d curr_acc_world_t = curr_acc_world;
    curr_acc_world += acc_change_rate * Eigen::Vector3d::Random();
    curr_gyr += gyr_change_rate * Eigen::Vector3d::Random();
    Eigen::Vector3d curr_gyr_mid = 0.5 * (curr_gyr_t + curr_gyr);
    Eigen::Vector3d curr_acc_world_mid = 0.5 * (curr_acc_world_t + curr_acc_world);

    // add bias and add to result to get imu measurement
    Eigen::Vector3d gyr_measure = curr_gyr + result.bias_gyr;
    Eigen::Vector3d acc_measure =
        curr_pose.so3().inverse() * (curr_acc_world - result.gravity) + result.bias_acc;

    // update pose
    // use mid point value to compute pose
    curr_pose.so3() = curr_pose.so3() * Sophus::SO3d::exp(time_interval_sec * curr_gyr_mid);
    curr_pose.translation() += time_interval_sec * curr_velo +
                               (0.5 * time_interval_sec * time_interval_sec) * curr_acc_world_mid;
    // update velocity
    curr_velo += time_interval_sec * curr_acc_world_mid;
    result.Add(current_time, acc_measure, gyr_measure, curr_velo, curr_pose);
  }
  return result;
}

}  // namespace mapmind::ikfom::utils
