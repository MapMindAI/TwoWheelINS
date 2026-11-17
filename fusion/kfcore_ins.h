// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FUSION_KFCORE_INS_H_
#define FUSION_KFCORE_INS_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 15-state error-state INS on the KFCore UDU filter, after INSLIB's design: the nominal state is
// integrated in double, the error covariance is kept as float U-D factors (P = U diag(d) U^T),
// predicted with Thornton's and updated with Bierman's algorithm. Gravity is constant along world
// -z; Earth rotation is ignored. Truth = nominal + error, with the attitude error in the world
// frame (rotation_true = exp(error) * rotation). The nominal state is integrated every IMU sample;
// the covariance only once `covariance_period` has elapsed, over the elapsed time (INSLIB's
// kalman_update_dt_sec). C11, fixed-size storage only, no heap.
enum {
  KFCORE_INS_POSITION = 0,
  KFCORE_INS_VELOCITY = 3,
  KFCORE_INS_ATTITUDE = 6,
  KFCORE_INS_ACCELEROMETER_BIAS = 9,
  KFCORE_INS_GYROSCOPE_BIAS = 12,
  KFCORE_INS_STATE_SIZE = 15,
  // most measurement rows one kfcore_ins_update fuses
  KFCORE_INS_MAX_UPDATE_ROWS = 6,
};

// Continuous-time noise densities.
typedef struct {
  double accelerometer;       // m/s^2/sqrt(Hz)
  double gyroscope;           // rad/s/sqrt(Hz)
  double accelerometer_bias;  // m/s^3/sqrt(Hz)
  double gyroscope_bias;      // rad/s^2/sqrt(Hz)
  // random walk added on top of the IMU-driven terms
  double position;  // m/sqrt(s)
  double velocity;  // m/s/sqrt(s)
  double attitude;  // rad/sqrt(s)
} KfcoreInsProcessNoise;

typedef struct {
  KfcoreInsProcessNoise process_noise;
  double gravity;            // m/s^2, along world -z
  double covariance_period;  // s
  // IMU time not yet propagated into the covariance (s)
  double pending_covariance_dt;

  double position[3];  // world, m
  // body to world, unit quaternion (w, x, y, z)
  double rotation[4];
  double velocity[3];            // world, m/s
  double accelerometer_bias[3];  // body, m/s^2
  double gyroscope_bias[3];      // body, rad/s

  // U-D factors, U column-major as KFCore expects
  float u[KFCORE_INS_STATE_SIZE * KFCORE_INS_STATE_SIZE];
  float d[KFCORE_INS_STATE_SIZE];
} KfcoreIns;

// `covariance_period` (s): 0 predicts the covariance every sample. Starts at rest at the origin
// with identity covariance.
void kfcore_ins_init(KfcoreIns* ins, const KfcoreInsProcessNoise* process_noise, double gravity,
                     double covariance_period);

// Sets the nominal state (zero accelerometer bias) and a diagonal covariance from `initial_std`
// (KFCORE_INS_STATE_SIZE values). `rotation`: body to world, unit quaternion (w, x, y, z).
void kfcore_ins_reset(KfcoreIns* ins, const double position[3], const double rotation[4],
                      const double velocity[3], const double gyroscope_bias[3],
                      const double* initial_std);

// Strapdown step over `dt` (s) with body-frame specific force (m/s^2) and angular rate (rad/s).
void kfcore_ins_predict(KfcoreIns* ins, const double acceleration[3],
                        const double angular_velocity[3], double dt);

// Fuses `residual` = jacobian * error + noise with independent noise of `variance` (`rows`
// values each; `jacobian` is rows x KFCORE_INS_STATE_SIZE, row-major), then applies the error to
// the nominal state. Returns false if `rows` is out of range or KFCore rejected a row.
bool kfcore_ins_update(KfcoreIns* ins, const double* residual, const double* jacobian,
                       const double* variance, int rows);

// Error variance of state `index` (the covariance diagonal).
double kfcore_ins_variance(const KfcoreIns* ins, int index);

// Error covariance U diag(d) U^T, KFCORE_INS_STATE_SIZE^2 values (symmetric).
void kfcore_ins_covariance(const KfcoreIns* ins, double* covariance);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // FUSION_KFCORE_INS_H_
