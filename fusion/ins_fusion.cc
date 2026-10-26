// Copyright 2026 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#include "fusion/ins_fusion.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include "common/math/geodesy.h"

namespace mapmind::fusion {

namespace {

using ikfom::ins::StateIkfom;

enum MeasurementIndex { kGpsPosition = 0, kWorldVelocity = 1 };

constexpr double kGravity = 9.8099;
constexpr double kStaticAlignmentSeconds = 1.0;
// The filters start from the velocity between two GPS fixes once the vehicle moves this fast.
constexpr double kMinStartSpeedMps = 3.0;
constexpr double kMinStartIntervalSeconds = 0.5;
constexpr double kMaxStartIntervalSeconds = 1.5;
constexpr int kYawHypotheses = 12;
constexpr int kYawSelectionFixes = 10;
constexpr int kMinSatellites = 4;
constexpr int kMaxIterations = 4;

// GPS fix status from the satellite count (the receiver's own status flag is not used).
constexpr int kMinFixSatellites = 21;
constexpr int kMinFloatSatellites = 11;
// RTK-style accuracy for fix and float solutions (m).
constexpr double kGpsFixHorizontalStd = 0.05;
constexpr double kGpsFixVerticalStd = 0.1;
constexpr double kGpsFloatHorizontalStd = 0.5;
constexpr double kGpsFloatVerticalStd = 1.0;
// Single solutions: std = scale / (satellites - 3). Fix-to-fix scatter in mobili_2 falls this way
// with the satellite count; the scale keeps 1.5 m / 10 m at the median of 8 satellites.
constexpr double kGpsSingleHorizontalStdScale = 7.5;  // m
constexpr double kGpsSingleVerticalStdScale = 50.0;   // m
constexpr double kAccelerometerStd = 1.0;             // m/s^2, dominated by vibration
constexpr double kGyroscopeStd = 0.05;                // rad/s
constexpr double kAccelerometerBiasStd = 1e-2;
constexpr double kGyroscopeBiasStd = 1e-3;

// Stationary below this wheel speed (the wheel sensor reports whole km/h).
constexpr double kStationarySpeedMps = 0.1;
constexpr double kMaxWheelSpeedAgeSeconds = 0.5;
constexpr double kZeroVelocityStd = 0.05;  // m/s
// Level-road prior while moving: world vertical velocity near zero. Single-solution altitude
// wanders tens of metres on flat roads (a 70 m climb in mobili_2 with no change in IMU tilt), and
// a body-frame constraint lets that through the poorly observed pitch and accelerometer bias.
// Real slopes (0.5 m/s at 10 % and 5 m/s) still come through, lagged, from GPS.
constexpr double kVerticalVelocityStd = 0.3;       // m/s
constexpr double kUnconstrainedVelocityStd = 1e3;  // m/s
// Altitude innovation gate (chi-square, 1 dof, 99.9 %): an outlier fix is applied without its
// altitude. After this many dropped altitudes in a row the next one is applied, so a drifted
// height recovers. Horizontal gating is not used: single-solution fixes stray far beyond their
// covariance, and dropping them let the filter drift away in mobili_2.
constexpr double kVerticalGate = 10.83;
constexpr int kMaxDroppedAltitudes = 5;
constexpr double kDroppedAltitudeVariance = 1e8;  // m^2

double Seconds(int64_t duration_ns) { return 1e-9 * static_cast<double>(duration_ns); }

Eigen::Vector3d Geodetic(const sensor::GpsData& gps) {
  return {gps.latitude_deg, gps.longitude_deg, gps.altitude_m};
}

}  // namespace

Eigen::Matrix3d GpsCovariance(int satellite_count) {
  double horizontal = kGpsFixHorizontalStd;
  double vertical = kGpsFixVerticalStd;
  if (satellite_count < kMinFloatSatellites) {
    const double redundancy = std::max(satellite_count - 3, 1);
    horizontal = kGpsSingleHorizontalStdScale / redundancy;
    vertical = kGpsSingleVerticalStdScale / redundancy;
  } else if (satellite_count < kMinFixSatellites) {
    horizontal = kGpsFloatHorizontalStd;
    vertical = kGpsFloatVerticalStd;
  }
  return Eigen::Vector3d(horizontal, horizontal, vertical).cwiseAbs2().asDiagonal();
}

InsFusion::InsFusion() {
  double limit[StateIkfom::DOF];
  std::fill(limit, limit + StateIkfom::DOF, 1e-3);
  filter_.init_dyn(ikfom::ins::GetVelocityField, ikfom::ins::DVelocityFieldDx,
                   ikfom::ins::DVelocityFieldDw, kMaxIterations, limit);
  filter_.add_dyn_vec(ikfom::ins::PositionMeasurement, ikfom::ins::DPositionMeasurementDx,
                      ikfom::ins::DPositionMeasurementDv);
  filter_.add_dyn_vec(ikfom::ins::WorldVelocityMeasurement, ikfom::ins::DWorldVelocityMeasurementDx,
                      ikfom::ins::DWorldVelocityMeasurementDv);
}

void InsFusion::AddImu(const sensor::ImuData& imu) {
  if (static_sample_count_ == 0) {
    first_imu_timestamp_ns_ = imu.timestamp_ns;
    last_imu_timestamp_ns_ = imu.timestamp_ns;
  }
  const double dt = Seconds(imu.timestamp_ns - last_imu_timestamp_ns_);
  last_imu_timestamp_ns_ = imu.timestamp_ns;

  static const auto process_noise = ikfom::GetProcessNoiseCov(
      kAccelerometerStd * kAccelerometerStd, kGyroscopeStd * kGyroscopeStd,
      kAccelerometerBiasStd * kAccelerometerBiasStd, kGyroscopeBiasStd * kGyroscopeBiasStd);
  ikfom::ImuIkfom input;
  input.acc = imu.linear_acceleration;
  input.gyro = imu.angular_velocity;
  if (initialized_) {
    filter_.predict(dt, process_noise, input);
    ConstrainVelocity(imu.timestamp_ns);
  } else if (!hypotheses_.empty()) {
    for (YawHypothesis& hypothesis : hypotheses_) {
      hypothesis.filter.predict(dt, process_noise, input);
    }
  } else if (attitude_) {
    *attitude_ = *attitude_ * Sophus::SO3d::exp(dt * imu.angular_velocity);
  } else if (Seconds(imu.timestamp_ns - first_imu_timestamp_ns_) < kStaticAlignmentSeconds) {
    static_acc_sum_ += imu.linear_acceleration;
    ++static_sample_count_;
  } else {
    // level: rotate the mean specific force onto world up
    const Eigen::Vector3d up_body = static_acc_sum_.normalized();
    const Eigen::Vector3d axis = up_body.cross(Eigen::Vector3d::UnitZ());
    attitude_ = Sophus::SO3d::exp(std::atan2(axis.norm(), up_body.z()) * axis.normalized());
  }
}

bool InsFusion::IsUsable(const sensor::GpsData& gps) const {
  const bool repeated = last_gps_ && last_gps_->latitude_deg == gps.latitude_deg &&
                        last_gps_->longitude_deg == gps.longitude_deg;
  return gps.satellite_count >= kMinSatellites && !repeated;
}

std::optional<Eigen::Vector3d> InsFusion::AddGps(const sensor::GpsData& gps) {
  if (!IsUsable(gps)) {
    return std::nullopt;
  }
  if (!utm_zone_) {
    utm_zone_ = math::UtmZone(gps.longitude_deg);
  }
  const Eigen::Vector3d position = math::GeodeticToUtm(Geodetic(gps), *utm_zone_);
  std::optional<Eigen::Vector3d> innovation;
  if (initialized_) {
    if (!IsStationary(gps.timestamp_ns)) {
      innovation = UpdateGpsPosition(position, GpsCovariance(gps.satellite_count));
    }
  } else if (!hypotheses_.empty()) {
    UpdateYawHypotheses(position, GpsCovariance(gps.satellite_count));
  } else {
    StartYawHypotheses(gps);
  }
  last_gps_ = gps;
  return innovation;
}

Eigen::Vector3d InsFusion::UpdateGpsPosition(const Eigen::Vector3d& position,
                                             Eigen::Matrix3d gps_covariance) {
  const int altitude_index = filter_.get_x().pos.IDX + 2;
  const Eigen::Vector3d innovation = position - Eigen::Vector3d(filter_.get_x().pos);
  const double altitude_variance =
      filter_.get_P()(altitude_index, altitude_index) + gps_covariance(2, 2);
  if (innovation.z() * innovation.z() > kVerticalGate * altitude_variance &&
      dropped_altitude_count_ < kMaxDroppedAltitudes) {
    gps_covariance(2, 2) = kDroppedAltitudeVariance;
    ++dropped_altitude_count_;
  } else {
    dropped_altitude_count_ = 0;
  }
  filter_.update_iterated_dyn(position, gps_covariance, kGpsPosition);
  return innovation;
}

bool InsFusion::IsStationary(int64_t timestamp_ns) const {
  return last_wheel_speed_ &&
         Seconds(timestamp_ns - last_wheel_speed_->timestamp_ns) < kMaxWheelSpeedAgeSeconds &&
         last_wheel_speed_->speed_mps < kStationarySpeedMps;
}

void InsFusion::ConstrainVelocity(int64_t timestamp_ns) {
  if (IsStationary(timestamp_ns)) {
    filter_.update_iterated_dyn(Eigen::Vector3d::Zero(),
                                kZeroVelocityStd * kZeroVelocityStd * Eigen::Matrix3d::Identity(),
                                kWorldVelocity);
    return;
  }
  const Eigen::Vector3d velocity = filter_.get_x().vel;
  filter_.update_iterated_dyn(
      Eigen::Vector3d(velocity.x(), velocity.y(), 0.0),
      Eigen::Vector3d(kUnconstrainedVelocityStd, kUnconstrainedVelocityStd, kVerticalVelocityStd)
          .cwiseAbs2()
          .asDiagonal()
          .toDenseMatrix(),
      kWorldVelocity);
}

void InsFusion::StartYawHypotheses(const sensor::GpsData& gps) {
  if (!attitude_ || !last_gps_) {
    return;
  }
  const double dt = Seconds(gps.timestamp_ns - last_gps_->timestamp_ns);
  const Eigen::Vector3d position = math::GeodeticToUtm(Geodetic(gps), *utm_zone_);
  const Eigen::Vector3d displacement =
      position - math::GeodeticToUtm(Geodetic(*last_gps_), *utm_zone_);
  if (dt < kMinStartIntervalSeconds || dt > kMaxStartIntervalSeconds ||
      displacement.head<2>().norm() / dt < kMinStartSpeedMps) {
    return;
  }

  Eigen::Matrix<double, StateIkfom::DOF, 1> initial_std;
  initial_std << GpsCovariance(gps.satellite_count).diagonal().cwiseSqrt(),
      Eigen::Vector3d(0.05, 0.05, 0.3), Eigen::Vector3d::Constant(1.0),
      Eigen::Vector3d::Constant(0.02), Eigen::Vector3d::Constant(0.3),
      Eigen::Vector2d::Constant(1e-3);
  for (int i = 0; i < kYawHypotheses; ++i) {
    YawHypothesis& hypothesis = hypotheses_.emplace_back(YawHypothesis{filter_});
    StateIkfom& state = hypothesis.filter.mutable_state();
    state.pos = position;
    state.rot =
        (Sophus::SO3d::rotZ(2.0 * M_PI * i / kYawHypotheses) * *attitude_).unit_quaternion();
    state.vel = Eigen::Vector3d(displacement.x() / dt, displacement.y() / dt, 0.0);
    state.bg.setZero();
    state.ba.setZero();
    state.grav = MTK::Gravity(Eigen::Vector3d(0.0, 0.0, -kGravity));
    hypothesis.filter.mutable_covariance() = initial_std.cwiseAbs2().asDiagonal();
  }
  hypothesis_fix_count_ = 0;
}

void InsFusion::UpdateYawHypotheses(const Eigen::Vector3d& position,
                                    const Eigen::Matrix3d& gps_covariance) {
  for (YawHypothesis& hypothesis : hypotheses_) {
    const int position_index = hypothesis.filter.get_x().pos.IDX;
    const Eigen::Matrix3d innovation_covariance =
        hypothesis.filter.get_P().block<3, 3>(position_index, position_index) + gps_covariance;
    const Eigen::Vector3d innovation = position - Eigen::Vector3d(hypothesis.filter.get_x().pos);
    hypothesis.cost += innovation.dot(innovation_covariance.ldlt().solve(innovation)) +
                       std::log(innovation_covariance.determinant());
    hypothesis.filter.update_iterated_dyn(position, gps_covariance, kGpsPosition);
  }
  if (++hypothesis_fix_count_ < kYawSelectionFixes) {
    return;
  }
  filter_ = std::min_element(
                hypotheses_.begin(), hypotheses_.end(),
                [](const YawHypothesis& a, const YawHypothesis& b) { return a.cost < b.cost; })
                ->filter;
  hypotheses_.clear();
  initialized_ = true;
}

void InsFusion::AddWheelSpeed(const sensor::WheelSpeedData& wheel_speed) {
  last_wheel_speed_ = wheel_speed;
}

Sophus::SE3d InsFusion::pose() const {
  const StateIkfom& state = filter_.get_x();
  return Sophus::SE3d(state.rot.normalized(), state.pos);
}

}  // namespace mapmind::fusion
