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

struct InsFusionOptions {
  // level-road prior: near-zero world vertical velocity while moving
  bool constrain_vertical_velocity = true;
  // GPS speed between two fixes that starts the yaw selection (m/s); 3 m/s for a vehicle, lower
  // for a walk with accurate GPS
  double min_start_speed_mps = 3.0;
  // A longer IMU gap restarts the yaw selection; a shorter one is integrated with the first sample
  // after it (INSLIB's max_prediction_time_sec).
  double max_imu_gap_seconds = 0.5;
};

// IMU + GPS INS on the KfcoreIns filter: IMU propagation and GPS position updates, plus velocity
// constraints for a road vehicle: zero velocity while the wheel speed reads zero (GPS is ignored
// then, since a parked receiver drifts) and, if `constrain_vertical_velocity`, near-zero vertical
// velocity while moving (see InsFusionOptions).
// A GPS altitude whose innovation fails a chi-square gate is dropped.
//
// Feed samples in timestamp order. The first second of IMU data must be (nearly) static: its mean
// specific force levels roll and pitch, its mean angular velocity is the initial gyroscope bias.
// Yaw needs no knowledge of the IMU mount: once two GPS fixes give a velocity of at least
// `min_start_speed_mps`, filters start at
// evenly spaced yaw hypotheses and, after a few more fixes, the one whose GPS innovations fit best
// is kept. Yaw shows in the GPS track only while the vehicle speeds up, brakes or turns.
// After an IMU gap over `max_imu_gap_seconds` the yaw selection restarts from the last attitude.
class InsFusion {
 public:
  InsFusion() : InsFusion(InsFusionOptions()) {}
  explicit InsFusion(const InsFusionOptions& options);

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
  // UTM position of `gps` relative to origin_ (m).
  Eigen::Vector3d LocalPosition(const sensor::GpsData& gps) const;
  void StartYawHypotheses(const sensor::GpsData& gps);
  void UpdateYawHypotheses(const Eigen::Vector3d& position, const Eigen::Matrix3d& gps_covariance);
  // Returns the innovation (UTM, m).
  Eigen::Vector3d UpdateGpsPosition(const Eigen::Vector3d& position,
                                    const Eigen::Matrix3d& gps_covariance);
  // Inside an IMU gap, predicts the filters to `timestamp_ns` with the last IMU sample.
  void PredictToMeasurement(int64_t timestamp_ns);
  // Back to yaw selection, keeping the attitude and gyroscope bias.
  void Rearm();
  bool IsStationary(int64_t timestamp_ns) const;
  // Zero-velocity update while stationary, level-road prior while moving.
  void ConstrainVelocity(int64_t timestamp_ns);

  InsFusionOptions options_;
  KfcoreIns filter_;
  bool initialized_ = false;
  std::vector<YawHypothesis> hypotheses_;
  int hypothesis_fix_count_ = 0;

  // levelling and pre-initialization attitude
  int64_t first_imu_timestamp_ns_ = 0;
  int64_t last_imu_timestamp_ns_ = 0;
  std::optional<sensor::ImuData> last_imu_;
  int static_sample_count_ = 0;
  Eigen::Vector3d static_acc_sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d static_gyro_sum_ = Eigen::Vector3d::Zero();
  std::optional<Sophus::SO3d> attitude_;
  // mean angular velocity while static (rad/s, body)
  Eigen::Vector3d gyroscope_bias_ = Eigen::Vector3d::Zero();

  std::optional<int> utm_zone_;
  // UTM position of the first usable fix: the filter runs in UTM axes relative to it, so its state
  // holds metres from the start rather than ~1e6 m northings.
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
  std::optional<sensor::GpsData> last_gps_;
  int dropped_altitude_count_ = 0;
  std::optional<sensor::WheelSpeedData> last_wheel_speed_;
};

}  // namespace mapmind::fusion

#endif  // FUSION_INS_FUSION_H_
