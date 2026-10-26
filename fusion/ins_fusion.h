// Copyright 2026 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#ifndef FUSION_INS_FUSION_H_
#define FUSION_INS_FUSION_H_

#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "common/sensor/gps_data.h"
#include "common/sensor/imu_data.h"
#include "common/sensor/wheel_speed_data.h"
#include "ikfom/backend/ikf_ins.h"
#include "sophus/se3.hpp"

namespace mapmind::fusion {

// GPS position covariance (UTM easting, northing, altitude; m^2) for a fix from
// `satellite_count` satellites: > 20 is a fix solution, 11-20 float, fewer single.
Eigen::Matrix3d GpsCovariance(int satellite_count);

// IMU + GPS INS on the ikfom filter: IMU propagation and GPS position updates.
//
// Feed samples in timestamp order. The first second of IMU data must be (nearly) static: its mean
// specific force levels roll and pitch. Yaw needs no knowledge of the IMU mount: once two GPS
// fixes give the velocity, filters start at evenly spaced yaw hypotheses and, after a few more
// fixes, the one whose GPS innovations fit best is kept. Yaw shows in the GPS track only while the
// vehicle accelerates (speeds up, brakes or turns).
class InsFusion {
 public:
  InsFusion();

  void AddImu(const sensor::ImuData& imu);
  // Returns the innovation (fix minus predicted position, UTM, m) if the fix was applied to the
  // selected filter.
  std::optional<Eigen::Vector3d> AddGps(const sensor::GpsData& gps);
  // Not fused in this version.
  void AddWheelSpeed(const sensor::WheelSpeedData& wheel_speed);

  // True once a yaw hypothesis is selected; pose() is valid from then on.
  bool initialized() const { return initialized_; }
  // IMU pose: body to UTM (easting, northing, altitude) in utm_zone().
  Sophus::SE3d pose() const;
  // UTM zone of the first usable GPS fix, used for all fixes.
  std::optional<int> utm_zone() const { return utm_zone_; }
  const ikfom::ins::StateIkfom& state() const { return filter_.get_x(); }

 private:
  using Filter = esekfom::esekf<ikfom::ins::StateIkfom, ikfom::ImuNoiseIkfom::DOF, ikfom::ImuIkfom>;

  struct YawHypothesis {
    Filter filter;
    // sum over fixes of the negative log-likelihood of the innovation (up to a constant)
    double cost = 0.0;
  };

  bool IsUsable(const sensor::GpsData& gps) const;
  void StartYawHypotheses(const sensor::GpsData& gps);
  void UpdateYawHypotheses(const Eigen::Vector3d& position, const Eigen::Matrix3d& gps_covariance);

  Filter filter_;
  bool initialized_ = false;
  std::vector<YawHypothesis> hypotheses_;
  int hypothesis_fix_count_ = 0;

  // levelling and pre-initialization attitude
  int64_t first_imu_timestamp_ns_ = 0;
  int64_t last_imu_timestamp_ns_ = 0;
  int static_sample_count_ = 0;
  Eigen::Vector3d static_acc_sum_ = Eigen::Vector3d::Zero();
  std::optional<Sophus::SO3d> attitude_;

  std::optional<int> utm_zone_;
  std::optional<sensor::GpsData> last_gps_;
};

}  // namespace mapmind::fusion

#endif  // FUSION_INS_FUSION_H_
