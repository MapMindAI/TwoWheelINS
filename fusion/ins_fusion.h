// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FUSION_INS_FUSION_H_
#define FUSION_INS_FUSION_H_

#include <cstdint>
#include <optional>

#include <Eigen/Core>

#include "common/sensor/gps_data.h"
#include "common/sensor/imu_data.h"
#include "common/sensor/wheel_speed_data.h"
#include "fusion/kfcore_ins.h"
#include "sophus/se3.hpp"

namespace mapmind::fusion {

struct InsFusionOptions {
  // level-road prior: near-zero world vertical velocity while moving
  bool constrain_vertical_velocity = false;
  // A longer IMU gap restarts the initialization; a shorter one is integrated with the first
  // sample after it (INSLIB's max_prediction_time_sec).
  double max_imu_gap_seconds = 0.5;
  // direction of travel in the IMU frame (unit vector)
  Eigen::Vector3d forward = Eigen::Vector3d::UnitX();
};

// IMU + GPS INS on the KfcoreIns filter: IMU propagation and GPS position updates, plus velocity
// constraints for a road vehicle: zero velocity while the wheel speed reads zero (GPS is ignored
// then, since a parked receiver drifts) and, if `constrain_vertical_velocity`, near-zero vertical
// velocity while moving (see InsFusionOptions).
// A GPS altitude whose innovation fails a chi-square gate is dropped.
//
// Feed samples in timestamp order. The mean specific force of the first second of IMU data levels
// roll and pitch; the gyroscope bias starts at zero (the start need not be static).
// The filter starts at the second usable GPS fix after levelling: position from that fix, velocity
// from the step between the two, and yaw turning `forward`, integrated in the levelled yaw-free
// frame over the step, onto the step's course. After an IMU gap over `max_imu_gap_seconds` it
// starts again from the next two fixes, keeping the last attitude.
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

  // True once the filter started; pose() is valid from then on.
  bool initialized() const { return initialized_; }
  // IMU pose: body to UTM (easting, northing, altitude) in utm_zone().
  Sophus::SE3d pose() const;
  // UTM zone of the first usable GPS fix, used for all fixes.
  std::optional<int> utm_zone() const { return utm_zone_; }
  const Eigen::Vector3d& gyroscope_bias() const { return filter_.gyroscope_bias(); }
  const Eigen::Vector3d& accelerometer_bias() const { return filter_.accelerometer_bias(); }

 private:
  // the first of the two fixes the filter starts from
  struct FirstFix {
    int64_t timestamp_ns;
    // UTM, relative to origin_ (m), and its horizontal variance (m^2)
    Eigen::Vector3d gps_position;
    double gps_variance;
  };

  bool IsUsable(const sensor::GpsData& gps) const;
  // UTM position of `gps` relative to origin_ (m).
  Eigen::Vector3d LocalPosition(const sensor::GpsData& gps) const;
  // Keeps the first fix, starts the filter at the second.
  void Start(const sensor::GpsData& gps);
  // Returns the innovation (UTM, m).
  Eigen::Vector3d UpdateGpsPosition(const Eigen::Vector3d& position,
                                    const Eigen::Matrix3d& gps_covariance);
  // Inside an IMU gap, predicts the filters to `timestamp_ns` with the last IMU sample.
  void PredictToMeasurement(int64_t timestamp_ns);
  // Back to the start from two fixes, keeping the attitude and gyroscope bias.
  void Rearm();
  bool IsStationary(int64_t timestamp_ns) const;
  // Zero-velocity update while stationary, level-road prior while moving.
  void ConstrainVelocity(int64_t timestamp_ns);

  InsFusionOptions options_;
  KfcoreIns filter_;
  bool initialized_ = false;
  std::optional<FirstFix> first_fix_;

  // levelling, and the pre-initialization attitude and forward integral (levelled frame, s)
  int64_t first_imu_timestamp_ns_ = 0;
  int64_t last_imu_timestamp_ns_ = 0;
  std::optional<sensor::ImuData> last_imu_;
  int static_sample_count_ = 0;
  Eigen::Vector3d static_acc_sum_ = Eigen::Vector3d::Zero();
  std::optional<Sophus::SO3d> attitude_;
  Eigen::Vector3d integrated_forward_ = Eigen::Vector3d::Zero();
  // rad/s, body: zero until a restart carries the filter's estimate
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
