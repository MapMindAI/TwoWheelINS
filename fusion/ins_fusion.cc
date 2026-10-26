// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "fusion/ins_fusion.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include "common/math/geodesy.h"

namespace mapmind::fusion {

namespace {

constexpr double kGravity = 9.8099;
constexpr double kStaticAlignmentSeconds = 1.0;
// The filters start from the velocity between two GPS fixes once the vehicle moves this fast.
constexpr double kMinStartSpeedMps = 3.0;
constexpr double kMinStartIntervalSeconds = 0.5;
constexpr double kMaxStartIntervalSeconds = 1.5;
constexpr int kYawHypotheses = 12;
constexpr int kYawSelectionFixes = 10;
constexpr int kMinSatellites = 4;

// Accelerometer and gyroscope densities are the vibration measured on mobili_2; the position,
// velocity and attitude random walks are INSLIB's defaults.
constexpr KfcoreIns::ProcessNoise kProcessNoise = {/*accelerometer=*/0.2,
                                                   /*gyroscope=*/0.01,
                                                   /*accelerometer_bias=*/2e-3,
                                                   /*gyroscope_bias=*/2e-4,
                                                   /*position=*/0.01,
                                                   /*velocity=*/5e-4,
                                                   /*attitude=*/1e-3};
// covariance prediction period (INSLIB's default 20 Hz); the strapdown runs every IMU sample
constexpr double kCovariancePeriodSeconds = 0.05;
// A longer IMU gap is not integrated: the filter restarts yaw selection, keeping attitude and
// gyroscope bias (INSLIB's max_prediction_time_sec).
constexpr double kMaxImuGapSeconds = 0.5;

// Initial standard deviations of a yaw hypothesis; position comes from the fix's covariance.
constexpr double kInitialVelocityStd = 1.0;           // m/s
constexpr double kInitialRollPitchStd = 0.05;         // rad
constexpr double kInitialYawStd = 0.3;                // rad, about half the hypothesis spacing
constexpr double kInitialAccelerometerBiasStd = 0.3;  // m/s^2
constexpr double kInitialGyroscopeBiasStd = 0.02;     // rad/s

// Stationary below this wheel speed (the wheel sensor reports whole km/h).
constexpr double kStationarySpeedMps = 0.1;
constexpr double kMaxWheelSpeedAgeSeconds = 0.5;
constexpr double kZeroVelocityStd = 0.05;     // m/s
constexpr double kVerticalVelocityStd = 0.3;  // m/s, level-road prior while moving
// Altitude innovation gate (chi-square, 1 dof, 99.9 %): an outlier fix is applied without its
// altitude, until this many in a row are dropped.
constexpr double kVerticalGate = 10.83;
constexpr int kMaxDroppedAltitudes = 5;

using StateIndex = KfcoreIns::StateIndex;

double Seconds(int64_t duration_ns) { return 1e-9 * static_cast<double>(duration_ns); }

Eigen::Vector3d Geodetic(const sensor::GpsData& gps) {
  return {gps.latitude_deg, gps.longitude_deg, gps.altitude_m};
}

// Error-state Jacobian selecting kRows states from `index`.
template <int kRows>
Eigen::Matrix<double, kRows, KfcoreIns::kStateSize> Selection(int index) {
  Eigen::Matrix<double, kRows, KfcoreIns::kStateSize> jacobian;
  jacobian.setZero();
  jacobian.template block<kRows, kRows>(0, index).setIdentity();
  return jacobian;
}

}  // namespace

InsFusion::InsFusion(bool constrain_vertical_velocity)
    : constrain_vertical_velocity_(constrain_vertical_velocity),
      filter_(kProcessNoise, kGravity, kCovariancePeriodSeconds) {}

void InsFusion::AddImu(const sensor::ImuData& imu) {
  if (static_sample_count_ == 0) {
    first_imu_timestamp_ns_ = imu.timestamp_ns;
    last_imu_timestamp_ns_ = imu.timestamp_ns;
  }
  const double dt = Seconds(imu.timestamp_ns - last_imu_timestamp_ns_);
  if (dt < 0.0) {
    return;
  }
  last_imu_timestamp_ns_ = imu.timestamp_ns;
  if (dt > kMaxImuGapSeconds) {
    Rearm();
    return;
  }

  if (initialized_) {
    filter_.Predict(imu.linear_acceleration, imu.angular_velocity, dt);
    ConstrainVelocity(imu.timestamp_ns);
  } else if (!hypotheses_.empty()) {
    for (YawHypothesis& hypothesis : hypotheses_) {
      hypothesis.filter.Predict(imu.linear_acceleration, imu.angular_velocity, dt);
    }
  } else if (attitude_) {
    *attitude_ = *attitude_ * Sophus::SO3d::exp(dt * (imu.angular_velocity - gyroscope_bias_));
  } else if (Seconds(imu.timestamp_ns - first_imu_timestamp_ns_) < kStaticAlignmentSeconds) {
    static_acc_sum_ += imu.linear_acceleration;
    static_gyro_sum_ += imu.angular_velocity;
    ++static_sample_count_;
  } else {
    // level: rotate the mean specific force onto world up
    gyroscope_bias_ = static_gyro_sum_ / static_sample_count_;
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

Eigen::Vector3d InsFusion::LocalPosition(const sensor::GpsData& gps) const {
  return math::GeodeticToUtm(Geodetic(gps), *utm_zone_) - origin_;
}

std::optional<Eigen::Vector3d> InsFusion::AddGps(const sensor::GpsData& gps) {
  if (!IsUsable(gps)) {
    return std::nullopt;
  }
  if (!utm_zone_) {
    utm_zone_ = math::UtmZone(gps.longitude_deg);
    origin_ = math::GeodeticToUtm(Geodetic(gps), *utm_zone_);
  }
  const Eigen::Vector3d position = LocalPosition(gps);
  std::optional<Eigen::Vector3d> innovation;
  if (initialized_) {
    if (!IsStationary(gps.timestamp_ns)) {
      innovation = UpdateGpsPosition(position, sensor::GpsCovariance(gps.satellite_count));
    }
  } else if (!hypotheses_.empty()) {
    UpdateYawHypotheses(position, sensor::GpsCovariance(gps.satellite_count));
  } else {
    StartYawHypotheses(gps);
  }
  last_gps_ = gps;
  return innovation;
}

Eigen::Vector3d InsFusion::UpdateGpsPosition(const Eigen::Vector3d& position,
                                             const Eigen::Matrix3d& gps_covariance) {
  const Eigen::Vector3d innovation = position - filter_.position();
  const Eigen::Vector3d variance = gps_covariance.diagonal();
  const int altitude_index = StateIndex::kPosition + 2;
  const double altitude_variance =
      filter_.covariance()(altitude_index, altitude_index) + variance.z();
  if (innovation.z() * innovation.z() > kVerticalGate * altitude_variance &&
      dropped_altitude_count_ < kMaxDroppedAltitudes) {
    ++dropped_altitude_count_;
    filter_.Update<2>(innovation.head<2>(), Selection<2>(StateIndex::kPosition),
                      variance.head<2>());
  } else {
    dropped_altitude_count_ = 0;
    filter_.Update<3>(innovation, Selection<3>(StateIndex::kPosition), variance);
  }
  return innovation;
}

bool InsFusion::IsStationary(int64_t timestamp_ns) const {
  return last_wheel_speed_ &&
         Seconds(timestamp_ns - last_wheel_speed_->timestamp_ns) < kMaxWheelSpeedAgeSeconds &&
         last_wheel_speed_->speed_mps < kStationarySpeedMps;
}

void InsFusion::ConstrainVelocity(int64_t timestamp_ns) {
  if (IsStationary(timestamp_ns)) {
    filter_.Update<3>(-filter_.velocity(), Selection<3>(StateIndex::kVelocity),
                      Eigen::Vector3d::Constant(kZeroVelocityStd * kZeroVelocityStd));
    return;
  }
  if (!constrain_vertical_velocity_) {
    return;
  }
  filter_.Update<1>(Eigen::Matrix<double, 1, 1>(-filter_.velocity().z()),
                    Selection<1>(StateIndex::kVelocity + 2),
                    Eigen::Matrix<double, 1, 1>(kVerticalVelocityStd * kVerticalVelocityStd));
}

void InsFusion::StartYawHypotheses(const sensor::GpsData& gps) {
  if (!attitude_ || !last_gps_) {
    return;
  }
  const double dt = Seconds(gps.timestamp_ns - last_gps_->timestamp_ns);
  const Eigen::Vector3d position = LocalPosition(gps);
  const Eigen::Vector3d displacement = position - LocalPosition(*last_gps_);
  if (dt < kMinStartIntervalSeconds || dt > kMaxStartIntervalSeconds ||
      displacement.head<2>().norm() / dt < kMinStartSpeedMps) {
    return;
  }

  KfcoreIns::StateVector initial_std;
  initial_std << sensor::GpsCovariance(gps.satellite_count).diagonal().cwiseSqrt(),
      Eigen::Vector3d::Constant(kInitialVelocityStd),
      Eigen::Vector3d(kInitialRollPitchStd, kInitialRollPitchStd, kInitialYawStd),
      Eigen::Vector3d::Constant(kInitialAccelerometerBiasStd),
      Eigen::Vector3d::Constant(kInitialGyroscopeBiasStd);
  for (int i = 0; i < kYawHypotheses; ++i) {
    YawHypothesis& hypothesis = hypotheses_.emplace_back(YawHypothesis{filter_});
    hypothesis.filter.Reset(position,
                            Sophus::SO3d::rotZ(2.0 * M_PI * i / kYawHypotheses) * *attitude_,
                            Eigen::Vector3d(displacement.x() / dt, displacement.y() / dt, 0.0),
                            gyroscope_bias_, initial_std);
  }
  hypothesis_fix_count_ = 0;
}

void InsFusion::UpdateYawHypotheses(const Eigen::Vector3d& position,
                                    const Eigen::Matrix3d& gps_covariance) {
  for (YawHypothesis& hypothesis : hypotheses_) {
    const Eigen::Matrix3d innovation_covariance =
        hypothesis.filter.covariance().block<3, 3>(StateIndex::kPosition, StateIndex::kPosition) +
        gps_covariance;
    const Eigen::Vector3d innovation = position - hypothesis.filter.position();
    hypothesis.cost += innovation.dot(innovation_covariance.ldlt().solve(innovation)) +
                       std::log(innovation_covariance.determinant());
    hypothesis.filter.Update<3>(innovation, Selection<3>(StateIndex::kPosition),
                                gps_covariance.diagonal());
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

void InsFusion::Rearm() {
  if (!initialized_ && hypotheses_.empty()) {
    return;
  }
  const KfcoreIns& filter = initialized_ ? filter_ : hypotheses_.front().filter;
  attitude_ = filter.rotation();
  gyroscope_bias_ = filter.gyroscope_bias();
  hypotheses_.clear();
  initialized_ = false;
}

void InsFusion::AddWheelSpeed(const sensor::WheelSpeedData& wheel_speed) {
  last_wheel_speed_ = wheel_speed;
}

Sophus::SE3d InsFusion::pose() const {
  return Sophus::SE3d(filter_.rotation(), origin_ + filter_.position());
}

}  // namespace mapmind::fusion
