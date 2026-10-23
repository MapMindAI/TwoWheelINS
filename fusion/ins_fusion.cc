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

enum MeasurementIndex { kGpsPosition = 0 };

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

constexpr double kGpsHorizontalStd = 1.5;  // m
constexpr double kGpsVerticalStd = 10.0;   // m
constexpr double kAccelerometerStd = 1.0;  // m/s^2, dominated by vibration
constexpr double kGyroscopeStd = 0.05;     // rad/s
constexpr double kAccelerometerBiasStd = 1e-2;
constexpr double kGyroscopeBiasStd = 1e-3;

double Seconds(int64_t duration_ns) { return 1e-9 * static_cast<double>(duration_ns); }

Eigen::Vector3d Geodetic(const sensor::GpsData& gps) {
  return {gps.latitude_deg, gps.longitude_deg, gps.altitude_m};
}

const Eigen::Matrix3d& GpsNoise() {
  static const Eigen::Matrix3d noise =
      Eigen::Vector3d(kGpsHorizontalStd, kGpsHorizontalStd, kGpsVerticalStd)
          .cwiseAbs2()
          .asDiagonal();
  return noise;
}

}  // namespace

InsFusion::InsFusion() {
  double limit[StateIkfom::DOF];
  std::fill(limit, limit + StateIkfom::DOF, 1e-3);
  filter_.init_dyn(ikfom::ins::GetVelocityField, ikfom::ins::DVelocityFieldDx,
                   ikfom::ins::DVelocityFieldDw, kMaxIterations, limit);
  filter_.add_dyn_vec(ikfom::ins::PositionMeasurement, ikfom::ins::DPositionMeasurementDx,
                      ikfom::ins::DPositionMeasurementDv);
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
    innovation = position - Eigen::Vector3d(filter_.get_x().pos);
    filter_.update_iterated_dyn(position, GpsNoise(), kGpsPosition);
  } else if (!hypotheses_.empty()) {
    UpdateYawHypotheses(position);
  } else {
    StartYawHypotheses(gps);
  }
  last_gps_ = gps;
  return innovation;
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
  initial_std << Eigen::Vector3d::Constant(kGpsHorizontalStd), Eigen::Vector3d(0.05, 0.05, 0.3),
      Eigen::Vector3d::Constant(1.0), Eigen::Vector3d::Constant(0.02),
      Eigen::Vector3d::Constant(0.3), Eigen::Vector2d::Constant(1e-3);
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

void InsFusion::UpdateYawHypotheses(const Eigen::Vector3d& position) {
  for (YawHypothesis& hypothesis : hypotheses_) {
    const int position_index = hypothesis.filter.get_x().pos.IDX;
    const Eigen::Matrix3d innovation_covariance =
        hypothesis.filter.get_P().block<3, 3>(position_index, position_index) + GpsNoise();
    const Eigen::Vector3d innovation = position - Eigen::Vector3d(hypothesis.filter.get_x().pos);
    hypothesis.cost += innovation.dot(innovation_covariance.ldlt().solve(innovation)) +
                       std::log(innovation_covariance.determinant());
    hypothesis.filter.update_iterated_dyn(position, GpsNoise(), kGpsPosition);
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

void InsFusion::AddWheelSpeed(const sensor::WheelSpeedData& /*wheel_speed*/) {}

Sophus::SE3d InsFusion::pose() const {
  const StateIkfom& state = filter_.get_x();
  return Sophus::SE3d(state.rot.normalized(), state.pos);
}

}  // namespace mapmind::fusion
