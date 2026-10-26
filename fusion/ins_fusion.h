// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FUSION_INS_FUSION_H_
#define FUSION_INS_FUSION_H_

#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "common/sensor/gps_data.h"
#include "common/sensor/imu_data.h"
#include "common/sensor/wheel_speed_data.h"
#include "fusion/kfcore_ins.h"
#include "sophus/se3.hpp"

namespace mapmind::fusion {

// IMU + GPS INS on the KfcoreIns filter: IMU propagation and GPS position updates, plus velocity
// constraints for a road vehicle: zero velocity while the wheel speed reads zero (GPS is ignored
// then, since a parked receiver drifts) and near-zero vertical velocity while moving.
// A GPS altitude whose innovation fails a chi-square gate is dropped.
//
// Feed samples in timestamp order. The first second of IMU data must be (nearly) static: its mean
// specific force levels roll and pitch, its mean angular velocity is the initial gyroscope bias.
// Yaw needs no knowledge of the IMU mount: once two GPS fixes give the velocity, filters start at
// evenly spaced yaw hypotheses and, after a few more fixes, the one whose GPS innovations fit best
// is kept. Yaw shows in the GPS track only while the vehicle speeds up, brakes or turns.
class InsFusion {
 public:
  InsFusion();

  void AddImu(const sensor::ImuData& imu);
  // Returns the innovation (fix minus predicted position, UTM, m) if the fix was applied to the
  // selected filter.
  std::optional<Eigen::Vector3d> AddGps(const sensor::GpsData& gps);
  // Only used to detect standstill.
  void AddWheelSpeed(const sensor::WheelSpeedData& wheel_speed);

  // True once a yaw hypothesis is selected; pose() is valid from then on.
  bool initialized() const { return initialized_; }
  // IMU pose: body to UTM (easting, northing, altitude) in utm_zone().
  Sophus::SE3d pose() const;
  // UTM zone of the first usable GPS fix, used for all fixes.
  std::optional<int> utm_zone() const { return utm_zone_; }
  const Eigen::Vector3d& gyroscope_bias() const { return filter_.gyroscope_bias(); }
  const Eigen::Vector3d& accelerometer_bias() const { return filter_.accelerometer_bias(); }

 private:
  struct YawHypothesis {
    KfcoreIns filter;
    // sum over fixes of the negative log-likelihood of the innovation (up to a constant)
    double cost = 0.0;
  };

  bool IsUsable(const sensor::GpsData& gps) const;
  void StartYawHypotheses(const sensor::GpsData& gps);
  void UpdateYawHypotheses(const Eigen::Vector3d& position, const Eigen::Matrix3d& gps_covariance);
  // Returns the innovation (UTM, m).
  Eigen::Vector3d UpdateGpsPosition(const Eigen::Vector3d& position,
                                    const Eigen::Matrix3d& gps_covariance);
  bool IsStationary(int64_t timestamp_ns) const;
  // Zero-velocity update while stationary, level-road prior while moving.
  void ConstrainVelocity(int64_t timestamp_ns);

  KfcoreIns filter_;
  bool initialized_ = false;
  std::vector<YawHypothesis> hypotheses_;
  int hypothesis_fix_count_ = 0;

  // levelling and pre-initialization attitude
  int64_t first_imu_timestamp_ns_ = 0;
  int64_t last_imu_timestamp_ns_ = 0;
  int static_sample_count_ = 0;
  Eigen::Vector3d static_acc_sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d static_gyro_sum_ = Eigen::Vector3d::Zero();
  std::optional<Sophus::SO3d> attitude_;
  // mean angular velocity while static (rad/s, body)
  Eigen::Vector3d gyroscope_bias_ = Eigen::Vector3d::Zero();

  std::optional<int> utm_zone_;
  std::optional<sensor::GpsData> last_gps_;
  int dropped_altitude_count_ = 0;
  std::optional<sensor::WheelSpeedData> last_wheel_speed_;
};

}  // namespace mapmind::fusion

#endif  // FUSION_INS_FUSION_H_
