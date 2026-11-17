// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "fusion/ins_fusion.h"

#include <math.h>
#include <string.h>

#include "common/math/geodesy.h"
#include "common/math/quaternion.h"
#include "common/sensor/gps_accuracy.h"

#define PI 3.14159265358979323846

static const double kGravity = 9.8099;
static const double kStaticAlignmentSeconds = 1.0;
enum { kMinSatellites = 4 };

// Accelerometer and gyroscope densities are the vibration measured on mobili_2; the position,
// velocity and attitude random walks are INSLIB's defaults.
static const KfcoreInsProcessNoise kProcessNoise = {/*accelerometer=*/0.2,
                                                    /*gyroscope=*/0.01,
                                                    /*accelerometer_bias=*/2e-3,
                                                    /*gyroscope_bias=*/2e-4,
                                                    /*position=*/0.01,
                                                    /*velocity=*/5e-4,
                                                    /*attitude=*/1e-3};
// A fix this much newer than the last IMU sample (inside an IMU gap) is applied after predicting
// to its time with that sample; closer ones are applied at the last sample.
static const double kMaxMeasurementLeadSeconds = 0.2;
// covariance prediction period (INSLIB's default 20 Hz); the strapdown runs every IMU sample
static const double kCovariancePeriodSeconds = 0.05;

// Initial standard deviations, at least; position, velocity and yaw also from the GPS noise.
static const double kInitialVelocityStd = 1.0;           // m/s
static const double kInitialRollPitchStd = 0.05;         // rad
static const double kMinInitialYawStd = 0.05;            // rad
static const double kInitialAccelerometerBiasStd = 0.3;  // m/s^2
static const double kInitialGyroscopeBiasStd = 0.02;     // rad/s

// Stationary below this wheel speed (the wheel sensor reports whole km/h).
static const double kStationarySpeedMps = 0.1;
static const double kMaxWheelSpeedAgeSeconds = 0.5;
static const double kZeroVelocityStd = 0.05;     // m/s
static const double kVerticalVelocityStd = 0.3;  // m/s, level-road prior while moving
// Altitude innovation gate (chi-square, 1 dof, 99.9 %): an outlier fix is applied without its
// altitude, until this many in a row are dropped.
static const double kVerticalGate = 10.83;
enum { kMaxDroppedAltitudes = 5 };

static double Seconds(int64_t duration_ns) { return 1e-9 * (double)duration_ns; }

// Fuses `residual` = error of the `rows` states from `index` + noise of `variance`.
static void UpdateStates(KfcoreIns* filter, int index, const double* residual,
                         const double* variance, int rows) {
  double jacobian[3 * KFCORE_INS_STATE_SIZE] = {0};
  for (int i = 0; i < rows; ++i) {
    jacobian[i * KFCORE_INS_STATE_SIZE + index + i] = 1.0;
  }
  kfcore_ins_update(filter, residual, jacobian, variance, rows);
}

InsFusionOptions ins_fusion_default_options(void) {
  const InsFusionOptions options = {false, 0.5, {1.0, 0.0, 0.0}};
  return options;
}

void ins_fusion_init(InsFusion* fusion, const InsFusionOptions* options) {
  memset(fusion, 0, sizeof(*fusion));
  fusion->options = *options;
  kfcore_ins_init(&fusion->filter, &kProcessNoise, kGravity, kCovariancePeriodSeconds);
}

static bool IsStationary(const InsFusion* fusion, int64_t timestamp_ns) {
  return fusion->has_wheel_speed &&
         Seconds(timestamp_ns - fusion->wheel_speed_timestamp_ns) < kMaxWheelSpeedAgeSeconds &&
         fusion->wheel_speed_mps < kStationarySpeedMps;
}

// Zero-velocity update while stationary, level-road prior while moving.
static void ConstrainVelocity(InsFusion* fusion, int64_t timestamp_ns) {
  KfcoreIns* filter = &fusion->filter;
  if (IsStationary(fusion, timestamp_ns)) {
    const double residual[3] = {-filter->velocity[0], -filter->velocity[1], -filter->velocity[2]};
    const double variance = kZeroVelocityStd * kZeroVelocityStd;
    const double variances[3] = {variance, variance, variance};
    UpdateStates(filter, KFCORE_INS_VELOCITY, residual, variances, 3);
    return;
  }
  if (!fusion->options.constrain_vertical_velocity) {
    return;
  }
  const double residual = -filter->velocity[2];
  const double variance = kVerticalVelocityStd * kVerticalVelocityStd;
  UpdateStates(filter, KFCORE_INS_VELOCITY + 2, &residual, &variance, 1);
}

// Back to the start from two fixes, keeping the attitude and gyroscope bias.
static void Rearm(InsFusion* fusion) {
  // the forward integral does not span the gap
  fusion->has_first_fix = false;
  if (!fusion->initialized) {
    return;
  }
  memcpy(fusion->attitude, fusion->filter.rotation, sizeof(fusion->attitude));
  fusion->has_attitude = true;
  memcpy(fusion->gyroscope_bias, fusion->filter.gyroscope_bias, sizeof(fusion->gyroscope_bias));
  fusion->initialized = false;
}

void ins_fusion_add_imu(InsFusion* fusion, int64_t timestamp_ns, const double acceleration[3],
                        const double angular_velocity[3]) {
  if (fusion->static_sample_count == 0) {
    fusion->first_imu_timestamp_ns = timestamp_ns;
    fusion->last_imu_timestamp_ns = timestamp_ns;
  }
  const double dt = Seconds(timestamp_ns - fusion->last_imu_timestamp_ns);
  if (dt < 0.0) {
    return;
  }
  fusion->last_imu_timestamp_ns = timestamp_ns;
  fusion->has_last_imu = true;
  memcpy(fusion->last_acceleration, acceleration, sizeof(fusion->last_acceleration));
  memcpy(fusion->last_angular_velocity, angular_velocity, sizeof(fusion->last_angular_velocity));
  if (dt > fusion->options.max_imu_gap_seconds) {
    Rearm(fusion);
    return;
  }

  if (fusion->initialized) {
    kfcore_ins_predict(&fusion->filter, acceleration, angular_velocity, dt);
    ConstrainVelocity(fusion, timestamp_ns);
  } else if (fusion->has_attitude) {
    double forward[3];
    quaternion_rotate(fusion->attitude, fusion->options.forward, forward);
    double rotation_vector[3];
    for (int i = 0; i < 3; ++i) {
      fusion->integrated_forward[i] += dt * forward[i];
      rotation_vector[i] = dt * (angular_velocity[i] - fusion->gyroscope_bias[i]);
    }
    double step[4];
    quaternion_exp(rotation_vector, step);
    quaternion_multiply(fusion->attitude, step, fusion->attitude);
    quaternion_normalize(fusion->attitude);
  } else if (Seconds(timestamp_ns - fusion->first_imu_timestamp_ns) < kStaticAlignmentSeconds) {
    for (int i = 0; i < 3; ++i) {
      fusion->static_acceleration_sum[i] += acceleration[i];
    }
    ++fusion->static_sample_count;
  } else {
    // level: rotate the mean specific force onto world up, about up_body x z
    const double* sum = fusion->static_acceleration_sum;
    const double up_norm = sqrt(sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]);
    const double up[3] = {sum[0] / up_norm, sum[1] / up_norm, sum[2] / up_norm};
    const double axis_norm = hypot(up[0], up[1]);
    const double angle = atan2(axis_norm, up[2]);
    double rotation_vector[3] = {0.0, 0.0, 0.0};
    if (axis_norm > 0.0) {
      rotation_vector[0] = angle * up[1] / axis_norm;
      rotation_vector[1] = -angle * up[0] / axis_norm;
    }
    quaternion_exp(rotation_vector, fusion->attitude);
    fusion->has_attitude = true;
  }
}

static bool IsUsable(const InsFusion* fusion, const InsFusionGps* gps) {
  const bool repeated = fusion->has_last_gps &&
                        fusion->last_gps_latitude_deg == gps->latitude_deg &&
                        fusion->last_gps_longitude_deg == gps->longitude_deg;
  return gps->satellite_count >= kMinSatellites && !repeated;
}

// UTM position of `gps` relative to the origin (m).
static void LocalPosition(const InsFusion* fusion, const InsFusionGps* gps, double position[3]) {
  const double geodetic[3] = {gps->latitude_deg, gps->longitude_deg, gps->altitude_m};
  geodetic_to_utm(geodetic, fusion->utm_zone, position);
  for (int i = 0; i < 3; ++i) {
    position[i] -= fusion->origin[i];
  }
}

// Inside an IMU gap, predicts the filter to `timestamp_ns` with the last IMU sample.
static void PredictToMeasurement(InsFusion* fusion, int64_t timestamp_ns) {
  const double dt = Seconds(timestamp_ns - fusion->last_imu_timestamp_ns);
  if (!fusion->has_last_imu || dt <= kMaxMeasurementLeadSeconds ||
      dt > fusion->options.max_imu_gap_seconds || !fusion->initialized) {
    return;
  }
  kfcore_ins_predict(&fusion->filter, fusion->last_acceleration, fusion->last_angular_velocity, dt);
  fusion->last_imu_timestamp_ns = timestamp_ns;
}

// Writes the innovation (UTM, m).
static void UpdateGpsPosition(InsFusion* fusion, const double position[3],
                              const GpsPositionStd* gps_std, double innovation[3]) {
  KfcoreIns* filter = &fusion->filter;
  for (int i = 0; i < 3; ++i) {
    innovation[i] = position[i] - filter->position[i];
  }
  const double horizontal = gps_std->horizontal * gps_std->horizontal;
  const double variance[3] = {horizontal, horizontal, gps_std->vertical * gps_std->vertical};
  const double altitude_variance =
      kfcore_ins_variance(filter, KFCORE_INS_POSITION + 2) + variance[2];
  if (innovation[2] * innovation[2] > kVerticalGate * altitude_variance &&
      fusion->dropped_altitude_count < kMaxDroppedAltitudes) {
    ++fusion->dropped_altitude_count;
    UpdateStates(filter, KFCORE_INS_POSITION, innovation, variance, 2);
  } else {
    fusion->dropped_altitude_count = 0;
    UpdateStates(filter, KFCORE_INS_POSITION, innovation, variance, 3);
  }
}

// Keeps the first fix, starts the filter at the second.
static void Start(InsFusion* fusion, const InsFusionGps* gps) {
  if (!fusion->has_attitude) {
    return;
  }
  double position[3];
  LocalPosition(fusion, gps, position);
  const GpsPositionStd gps_std = gps_position_std(gps->satellite_count);
  const double gps_variance = gps_std.horizontal * gps_std.horizontal;
  if (!fusion->has_first_fix) {
    fusion->has_first_fix = true;
    fusion->first_fix_timestamp_ns = gps->timestamp_ns;
    memcpy(fusion->first_fix_position, position, sizeof(position));
    fusion->first_fix_variance = gps_variance;
    memset(fusion->integrated_forward, 0, sizeof(fusion->integrated_forward));
    return;
  }
  // the step's course minus the heading of the forward direction over it
  double step[3];
  for (int i = 0; i < 3; ++i) {
    step[i] = position[i] - fusion->first_fix_position[i];
  }
  const double* forward = fusion->integrated_forward;
  const double yaw = atan2(step[1], step[0]) - atan2(forward[1], forward[0]);
  const double step_std = sqrt(fusion->first_fix_variance + gps_variance);
  const double dt = Seconds(gps->timestamp_ns - fusion->first_fix_timestamp_ns);
  const double velocity_std = fmax(step_std / dt, kInitialVelocityStd);
  const double yaw_std = fmin(fmax(step_std / hypot(step[0], step[1]), kMinInitialYawStd), PI);

  const double initial_std[KFCORE_INS_STATE_SIZE] = {gps_std.horizontal,
                                                     gps_std.horizontal,
                                                     gps_std.vertical,
                                                     velocity_std,
                                                     velocity_std,
                                                     velocity_std,
                                                     kInitialRollPitchStd,
                                                     kInitialRollPitchStd,
                                                     yaw_std,
                                                     kInitialAccelerometerBiasStd,
                                                     kInitialAccelerometerBiasStd,
                                                     kInitialAccelerometerBiasStd,
                                                     kInitialGyroscopeBiasStd,
                                                     kInitialGyroscopeBiasStd,
                                                     kInitialGyroscopeBiasStd};
  const double yaw_rotation[4] = {cos(0.5 * yaw), 0.0, 0.0, sin(0.5 * yaw)};
  double rotation[4];
  quaternion_multiply(yaw_rotation, fusion->attitude, rotation);
  quaternion_normalize(rotation);
  const double velocity[3] = {step[0] / dt, step[1] / dt, 0.0};
  kfcore_ins_reset(&fusion->filter, position, rotation, velocity, fusion->gyroscope_bias,
                   initial_std);
  fusion->has_first_fix = false;
  fusion->initialized = true;
}

bool ins_fusion_add_gps(InsFusion* fusion, const InsFusionGps* gps, double innovation[3]) {
  if (!IsUsable(fusion, gps)) {
    return false;
  }
  if (fusion->utm_zone == 0) {
    fusion->utm_zone = utm_zone(gps->longitude_deg);
    const double geodetic[3] = {gps->latitude_deg, gps->longitude_deg, gps->altitude_m};
    geodetic_to_utm(geodetic, fusion->utm_zone, fusion->origin);
  }
  PredictToMeasurement(fusion, gps->timestamp_ns);
  bool applied = false;
  if (fusion->initialized) {
    if (!IsStationary(fusion, gps->timestamp_ns)) {
      double position[3];
      LocalPosition(fusion, gps, position);
      const GpsPositionStd gps_std = gps_position_std(gps->satellite_count);
      double fix_innovation[3];
      UpdateGpsPosition(fusion, position, &gps_std, fix_innovation);
      if (innovation != NULL) {
        memcpy(innovation, fix_innovation, sizeof(fix_innovation));
      }
      applied = true;
    }
  } else {
    Start(fusion, gps);
  }
  fusion->has_last_gps = true;
  fusion->last_gps_latitude_deg = gps->latitude_deg;
  fusion->last_gps_longitude_deg = gps->longitude_deg;
  return applied;
}

void ins_fusion_add_wheel_speed(InsFusion* fusion, int64_t timestamp_ns, double speed_mps) {
  fusion->has_wheel_speed = true;
  fusion->wheel_speed_timestamp_ns = timestamp_ns;
  fusion->wheel_speed_mps = speed_mps;
}

void ins_fusion_pose(const InsFusion* fusion, double position[3], double rotation[4]) {
  for (int i = 0; i < 3; ++i) {
    position[i] = fusion->origin[i] + fusion->filter.position[i];
  }
  memcpy(rotation, fusion->filter.rotation, 4 * sizeof(double));
}
