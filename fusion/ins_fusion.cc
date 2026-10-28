// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "fusion/ins_fusion.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "common/math/geodesy.h"

namespace mapmind::fusion {

namespace {

constexpr double kGravity = 9.8099;
constexpr double kStaticAlignmentSeconds = 1.0;
// The yaw is fitted over a window of consecutive fixes that grows from kMinYawFitFixes until the
// fit's standard deviation is below kMaxYawFitStd, then slides at kMaxYawFitFixes. Its precision
// grows with about the third power of the window length; the filter refines the yaw after.
constexpr int kMinYawFitFixes = 5;
constexpr int kMaxYawFitFixes = 30;
constexpr double kMaxTrackYawStd = 0.3;   // rad, then refined by the filter
constexpr double kMaxCourseYawStd = 0.1;  // rad
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
// A fix this much newer than the last IMU sample (inside an IMU gap) is applied after predicting
// to its time with that sample; closer ones are applied at the last sample.
constexpr double kMaxMeasurementLeadSeconds = 0.1;
// covariance prediction period (INSLIB's default 20 Hz); the strapdown runs every IMU sample
constexpr double kCovariancePeriodSeconds = 0.05;

// Initial standard deviations; position comes from the fix's covariance, yaw from the fit.
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

InsFusion::InsFusion(const InsFusionOptions& options)
    : options_(options), filter_(kProcessNoise, kGravity, kCovariancePeriodSeconds) {}

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
    filter_.Predict(imu.linear_acceleration, imu.angular_velocity, dt);
    ConstrainVelocity(imu.timestamp_ns);
  } else if (attitude_) {
    const Eigen::Vector3d acceleration =
        *attitude_ * imu.linear_acceleration - Eigen::Vector3d(0.0, 0.0, kGravity);
    integrated_position_ += dt * integrated_velocity_ + 0.5 * dt * dt * acceleration;
    integrated_velocity_ += dt * acceleration;
    if (options_.forward) {
      integrated_forward_ += dt * (*attitude_ * *options_.forward);
    }
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
    FitYaw(gps);
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
  if (!options_.constrain_vertical_velocity) {
    return;
  }
  filter_.Update<1>(Eigen::Matrix<double, 1, 1>(-filter_.velocity().z()),
                    Selection<1>(StateIndex::kVelocity + 2),
                    Eigen::Matrix<double, 1, 1>(kVerticalVelocityStd * kVerticalVelocityStd));
}

void InsFusion::FitYaw(const sensor::GpsData& gps) {
  if (!attitude_) {
    return;
  }
  // IMU integrals extrapolated from the last IMU sample to the fix
  const double lead = Seconds(gps.timestamp_ns - last_imu_timestamp_ns_);
  yaw_fit_fixes_.push_back({gps.timestamp_ns, LocalPosition(gps),
                            sensor::GpsCovariance(gps.satellite_count)(0, 0), integrated_velocity_,
                            integrated_position_ + lead * integrated_velocity_,
                            integrated_forward_});
  const int fix_count = static_cast<int>(yaw_fit_fixes_.size());
  if (fix_count < kMinYawFitFixes) {
    return;
  }
  double gps_variance = 0.0;
  for (const YawFitFix& fix : yaw_fit_fixes_) {
    gps_variance = std::max(gps_variance, fix.gps_variance);
  }
  const YawFit fit = options_.forward ? FitCourseYaw(gps_variance) : FitTrackYaw(gps_variance);
  // also rejects NaN from a singular fit
  if (!(fit.yaw_std <= (options_.forward ? kMaxCourseYawStd : kMaxTrackYawStd))) {
    if (fix_count == kMaxYawFitFixes) {
      yaw_fit_fixes_.erase(yaw_fit_fixes_.begin());
    }
    return;
  }

  KfcoreIns::StateVector initial_std;
  initial_std << sensor::GpsCovariance(gps.satellite_count).diagonal().cwiseSqrt(),
      Eigen::Vector3d::Constant(kInitialVelocityStd),
      Eigen::Vector3d(kInitialRollPitchStd, kInitialRollPitchStd,
                      std::max(fit.yaw_std, kMinInitialYawStd)),
      Eigen::Vector3d::Constant(kInitialAccelerometerBiasStd),
      Eigen::Vector3d::Constant(kInitialGyroscopeBiasStd);
  filter_.Reset(yaw_fit_fixes_.back().gps_position, Sophus::SO3d::rotZ(fit.yaw) * *attitude_,
                fit.velocity, gyroscope_bias_, initial_std);
  yaw_fit_fixes_.clear();
  initialized_ = true;
}

InsFusion::YawFit InsFusion::FitCourseYaw(double gps_variance) const {
  // Each GPS step, turned back by the mean heading of the forward direction over its interval,
  // points along the yaw; their sum weighs the steps by length and follows turns.
  //   sum = sum_k R(-h_k) (p_k - p_{k-1}) = sum_j C_j p_j,
  // so its covariance is gps_variance * sum_j C_j C_j^T.
  const int fix_count = static_cast<int>(yaw_fit_fixes_.size());
  std::vector<Eigen::Matrix2d> step_rotations;
  for (int k = 1; k < fix_count; ++k) {
    const Eigen::Vector3d forward = yaw_fit_fixes_[k].forward - yaw_fit_fixes_[k - 1].forward;
    step_rotations.push_back(Eigen::Rotation2Dd(-std::atan2(forward.y(), forward.x())).matrix());
  }
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (int j = 0; j < fix_count; ++j) {
    const Eigen::Matrix2d coefficient =
        (j > 0 ? step_rotations[j - 1] : Eigen::Matrix2d::Zero()) -
        (j < fix_count - 1 ? step_rotations[j] : Eigen::Matrix2d::Zero());
    sum += coefficient * yaw_fit_fixes_[j].gps_position.head<2>();
    covariance += gps_variance * coefficient * coefficient.transpose();
  }
  const Eigen::Vector2d yaw_gradient = Eigen::Vector2d(-sum.y(), sum.x()) / sum.squaredNorm();

  const YawFitFix& last = yaw_fit_fixes_.back();
  const YawFitFix& previous = yaw_fit_fixes_[fix_count - 2];
  Eigen::Vector3d velocity = (last.gps_position - previous.gps_position) /
                             Seconds(last.timestamp_ns - previous.timestamp_ns);
  velocity.z() = 0.0;
  return {std::atan2(sum.y(), sum.x()), std::sqrt(yaw_gradient.dot(covariance * yaw_gradient)),
          velocity};
}

InsFusion::YawFit InsFusion::FitTrackYaw(double gps_variance) const {
  // GPS displacement from the first fix of the window, for fix k at dt_k after it:
  //   p_k - p_0 = v_0 dt_k + Rz(yaw) d_k + b dt_k^2 / 2,
  // with d_k the levelled IMU double integral over the same time (from rest), v_0 the start
  // velocity and b a constant world acceleration (gravity leaking through the levelling error).
  // Linear in x = (v_0, cos(yaw), sin(yaw), b).
  const int fix_count = static_cast<int>(yaw_fit_fixes_.size());
  const YawFitFix& first = yaw_fit_fixes_.front();
  Eigen::MatrixXd design = Eigen::MatrixXd::Zero(2 * (fix_count - 1), 6);
  Eigen::VectorXd displacement(2 * (fix_count - 1));
  for (int k = 1; k < fix_count; ++k) {
    const YawFitFix& fix = yaw_fit_fixes_[k];
    const double dt = Seconds(fix.timestamp_ns - first.timestamp_ns);
    const Eigen::Vector3d d = fix.position - first.position - dt * first.velocity;
    const int x = 2 * (k - 1);
    design.row(x) << dt, 0.0, d.x(), -d.y(), 0.5 * dt * dt, 0.0;
    design.row(x + 1) << 0.0, dt, d.y(), d.x(), 0.0, 0.5 * dt * dt;
    displacement.segment<2>(x) = (fix.gps_position - first.gps_position).head<2>();
  }
  const Eigen::Matrix<double, 6, 6> information = design.transpose() * design;
  const Eigen::Matrix<double, 6, 1> solution =
      information.ldlt().solve(design.transpose() * displacement);
  const Eigen::Vector2d rotation = solution.segment<2>(2);
  // yaw std from the GPS horizontal variance through the least-squares covariance
  const Eigen::Matrix2d rotation_covariance =
      gps_variance * information.inverse().block<2, 2>(2, 2);
  const Eigen::Vector2d yaw_gradient =
      Eigen::Vector2d(-rotation.y(), rotation.x()) / rotation.squaredNorm();
  const double yaw = std::atan2(rotation.y(), rotation.x());

  const YawFitFix& last = yaw_fit_fixes_.back();
  const double dt = Seconds(last.timestamp_ns - first.timestamp_ns);
  Eigen::Vector3d velocity = Sophus::SO3d::rotZ(yaw) * (last.velocity - first.velocity);
  velocity.head<2>() += solution.head<2>() + dt * solution.tail<2>();
  velocity.z() = 0.0;
  return {yaw, std::sqrt(yaw_gradient.dot(rotation_covariance * yaw_gradient)), velocity};
}

void InsFusion::PredictToMeasurement(int64_t timestamp_ns) {
  const double dt = Seconds(timestamp_ns - last_imu_timestamp_ns_);
  if (!last_imu_ || dt <= kMaxMeasurementLeadSeconds || dt > options_.max_imu_gap_seconds) {
    return;
  }
  if (!initialized_) {
    return;
  }
  filter_.Predict(last_imu_->linear_acceleration, last_imu_->angular_velocity, dt);
  last_imu_timestamp_ns_ = timestamp_ns;
}

void InsFusion::Rearm() {
  // the IMU integrals do not span the gap
  yaw_fit_fixes_.clear();
  integrated_velocity_.setZero();
  integrated_position_.setZero();
  integrated_forward_.setZero();
  if (!initialized_) {
    return;
  }
  attitude_ = filter_.rotation();
  gyroscope_bias_ = filter_.gyroscope_bias();
  initialized_ = false;
}

void InsFusion::AddWheelSpeed(const sensor::WheelSpeedData& wheel_speed) {
  last_wheel_speed_ = wheel_speed;
}

Sophus::SE3d InsFusion::pose() const {
  return Sophus::SE3d(filter_.rotation(), origin_ + filter_.position());
}

}  // namespace mapmind::fusion
