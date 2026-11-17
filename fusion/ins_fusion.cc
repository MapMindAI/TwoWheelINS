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
constexpr int kMinSatellites = 4;

// Accelerometer and gyroscope densities are the vibration measured on mobili_2; the position,
// velocity and attitude random walks are INSLIB's defaults.
constexpr KfcoreInsProcessNoise kProcessNoise = {/*accelerometer=*/0.2,
                                                 /*gyroscope=*/0.01,
                                                 /*accelerometer_bias=*/2e-3,
                                                 /*gyroscope_bias=*/2e-4,
                                                 /*position=*/0.01,
                                                 /*velocity=*/5e-4,
                                                 /*attitude=*/1e-3};
// A fix this much newer than the last IMU sample (inside an IMU gap) is applied after predicting
// to its time with that sample; closer ones are applied at the last sample.
constexpr double kMaxMeasurementLeadSeconds = 0.2;
// covariance prediction period (INSLIB's default 20 Hz); the strapdown runs every IMU sample
constexpr double kCovariancePeriodSeconds = 0.05;

// Initial standard deviations, at least; position, velocity and yaw also from the GPS noise.
constexpr double kInitialVelocityStd = 1.0;           // m/s
constexpr double kInitialRollPitchStd = 0.05;         // rad
constexpr double kMinInitialYawStd = 0.05;            // rad
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

double Seconds(int64_t duration_ns) { return 1e-9 * static_cast<double>(duration_ns); }

Eigen::Vector3d Geodetic(const sensor::GpsData& gps) {
  return {gps.latitude_deg, gps.longitude_deg, gps.altitude_m};
}

Eigen::Map<const Eigen::Vector3d> Vector(const double (&vector)[3]) {
  return Eigen::Map<const Eigen::Vector3d>(vector);
}

// body to world
Sophus::SO3d Rotation(const KfcoreIns& filter) {
  const double* q = filter.rotation;
  return Sophus::SO3d(Eigen::Quaterniond(q[0], q[1], q[2], q[3]));
}

// Fuses `residual` = error of the kRows states from `index` + noise of `variance`.
template <int kRows>
void UpdateStates(KfcoreIns* filter, int index, const Eigen::Matrix<double, kRows, 1>& residual,
                  const Eigen::Matrix<double, kRows, 1>& variance) {
  Eigen::Matrix<double, kRows, KFCORE_INS_STATE_SIZE, Eigen::RowMajor> jacobian;
  jacobian.setZero();
  jacobian.template block<kRows, kRows>(0, index).setIdentity();
  kfcore_ins_update(filter, residual.data(), jacobian.data(), variance.data(), kRows);
}

}  // namespace

InsFusion::InsFusion(const InsFusionOptions& options) : options_(options) {
  kfcore_ins_init(&filter_, &kProcessNoise, kGravity, kCovariancePeriodSeconds);
}

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
  last_imu_ = imu;
  if (dt > options_.max_imu_gap_seconds) {
    Rearm();
    return;
  }

  if (initialized_) {
    kfcore_ins_predict(&filter_, imu.linear_acceleration.data(), imu.angular_velocity.data(), dt);
    ConstrainVelocity(imu.timestamp_ns);
  } else if (attitude_) {
    integrated_forward_ += dt * (*attitude_ * options_.forward);
    *attitude_ = *attitude_ * Sophus::SO3d::exp(dt * (imu.angular_velocity - gyroscope_bias_));
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
  PredictToMeasurement(gps.timestamp_ns);
  const Eigen::Vector3d position = LocalPosition(gps);
  std::optional<Eigen::Vector3d> innovation;
  if (initialized_) {
    if (!IsStationary(gps.timestamp_ns)) {
      innovation = UpdateGpsPosition(position, sensor::GpsCovariance(gps.satellite_count));
    }
  } else {
    Start(gps);
  }
  last_gps_ = gps;
  return innovation;
}

Eigen::Vector3d InsFusion::UpdateGpsPosition(const Eigen::Vector3d& position,
                                             const Eigen::Matrix3d& gps_covariance) {
  const Eigen::Vector3d innovation = position - Vector(filter_.position);
  const Eigen::Vector3d variance = gps_covariance.diagonal();
  Eigen::Matrix<double, KFCORE_INS_STATE_SIZE, KFCORE_INS_STATE_SIZE> covariance;
  kfcore_ins_covariance(&filter_, covariance.data());
  const int altitude_index = KFCORE_INS_POSITION + 2;
  const double altitude_variance = covariance(altitude_index, altitude_index) + variance.z();
  if (innovation.z() * innovation.z() > kVerticalGate * altitude_variance &&
      dropped_altitude_count_ < kMaxDroppedAltitudes) {
    ++dropped_altitude_count_;
    UpdateStates<2>(&filter_, KFCORE_INS_POSITION, innovation.head<2>(), variance.head<2>());
  } else {
    dropped_altitude_count_ = 0;
    UpdateStates<3>(&filter_, KFCORE_INS_POSITION, innovation, variance);
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
    UpdateStates<3>(&filter_, KFCORE_INS_VELOCITY, -Vector(filter_.velocity),
                    Eigen::Vector3d::Constant(kZeroVelocityStd * kZeroVelocityStd));
    return;
  }
  if (!options_.constrain_vertical_velocity) {
    return;
  }
  UpdateStates<1>(&filter_, KFCORE_INS_VELOCITY + 2,
                  Eigen::Matrix<double, 1, 1>(-filter_.velocity[2]),
                  Eigen::Matrix<double, 1, 1>(kVerticalVelocityStd * kVerticalVelocityStd));
}

void InsFusion::Start(const sensor::GpsData& gps) {
  if (!attitude_) {
    return;
  }
  const Eigen::Vector3d position = LocalPosition(gps);
  const Eigen::Matrix3d gps_covariance = sensor::GpsCovariance(gps.satellite_count);
  if (!first_fix_) {
    first_fix_ = {gps.timestamp_ns, position, gps_covariance(0, 0)};
    integrated_forward_.setZero();
    return;
  }
  // the step's course minus the heading of the forward direction over it
  const Eigen::Vector3d step = position - first_fix_->gps_position;
  const double yaw =
      std::atan2(step.y(), step.x()) - std::atan2(integrated_forward_.y(), integrated_forward_.x());
  const double step_std = std::sqrt(first_fix_->gps_variance + gps_covariance(0, 0));
  const double dt = Seconds(gps.timestamp_ns - first_fix_->timestamp_ns);
  const double velocity_std = std::max(step_std / dt, kInitialVelocityStd);
  const double yaw_std = std::clamp(step_std / step.head<2>().norm(), kMinInitialYawStd, M_PI);

  Eigen::Matrix<double, KFCORE_INS_STATE_SIZE, 1> initial_std;
  initial_std << gps_covariance.diagonal().cwiseSqrt(), Eigen::Vector3d::Constant(velocity_std),
      Eigen::Vector3d(kInitialRollPitchStd, kInitialRollPitchStd, yaw_std),
      Eigen::Vector3d::Constant(kInitialAccelerometerBiasStd),
      Eigen::Vector3d::Constant(kInitialGyroscopeBiasStd);
  const Eigen::Quaterniond rotation = (Sophus::SO3d::rotZ(yaw) * *attitude_).unit_quaternion();
  const double rotation_wxyz[4] = {rotation.w(), rotation.x(), rotation.y(), rotation.z()};
  const Eigen::Vector3d velocity(step.x() / dt, step.y() / dt, 0.0);
  kfcore_ins_reset(&filter_, position.data(), rotation_wxyz, velocity.data(),
                   gyroscope_bias_.data(), initial_std.data());
  first_fix_.reset();
  initialized_ = true;
}

void InsFusion::PredictToMeasurement(int64_t timestamp_ns) {
  const double dt = Seconds(timestamp_ns - last_imu_timestamp_ns_);
  if (!last_imu_ || dt <= kMaxMeasurementLeadSeconds || dt > options_.max_imu_gap_seconds) {
    return;
  }
  if (!initialized_) {
    return;
  }
  kfcore_ins_predict(&filter_, last_imu_->linear_acceleration.data(),
                     last_imu_->angular_velocity.data(), dt);
  last_imu_timestamp_ns_ = timestamp_ns;
}

void InsFusion::Rearm() {
  // the forward integral does not span the gap
  first_fix_.reset();
  if (!initialized_) {
    return;
  }
  attitude_ = Rotation(filter_);
  gyroscope_bias_ = Vector(filter_.gyroscope_bias);
  initialized_ = false;
}

void InsFusion::AddWheelSpeed(const sensor::WheelSpeedData& wheel_speed) {
  last_wheel_speed_ = wheel_speed;
}

Sophus::SE3d InsFusion::pose() const {
  return Sophus::SE3d(Rotation(filter_), origin_ + Vector(filter_.position));
}

}  // namespace mapmind::fusion
