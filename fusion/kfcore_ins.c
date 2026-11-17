// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "fusion/kfcore_ins.h"

#include <math.h>
#include <string.h>

#include "third_party/kfcore/kalman_udu.h"

enum {
  kStateSize = KFCORE_INS_STATE_SIZE,
  // Noise inputs: accelerometer, gyroscope, accelerometer bias walk, gyroscope bias walk, then
  // position, velocity and attitude random walk.
  kNoiseSize = 21,
};

// A period that is due within this fraction still predicts, so IMU timestamp jitter cannot halve
// the prediction rate.
static const double kCovariancePeriodTolerance = 0.01;

// result = a * b; quaternions (w, x, y, z). `result` may alias neither input.
static void QuaternionMultiply(const double a[4], const double b[4], double result[4]) {
  result[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  result[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  result[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  result[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

// Unit quaternion of the rotation vector `v` (rad), with Sophus's series near zero.
static void QuaternionExp(const double v[3], double result[4]) {
  const double theta_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  double real;
  double imaginary;
  if (theta_sq * theta_sq < 1e-30) {
    real = 1.0 - theta_sq / 8.0 + theta_sq * theta_sq / 384.0;
    imaginary = 0.5 - theta_sq / 48.0 + theta_sq * theta_sq / 3840.0;
  } else {
    const double theta = sqrt(theta_sq);
    real = cos(0.5 * theta);
    imaginary = sin(0.5 * theta) / theta;
  }
  result[0] = real;
  for (int i = 0; i < 3; ++i) {
    result[i + 1] = imaginary * v[i];
  }
}

static void QuaternionNormalize(double q[4]) {
  const double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (int i = 0; i < 4; ++i) {
    q[i] /= norm;
  }
}

// Row-major rotation matrix of the unit quaternion `q` (w, x, y, z).
static void QuaternionToMatrix(const double q[4], double m[9]) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  m[0] = 1.0 - 2.0 * (y * y + z * z);
  m[1] = 2.0 * (x * y - w * z);
  m[2] = 2.0 * (x * z + w * y);
  m[3] = 2.0 * (x * y + w * z);
  m[4] = 1.0 - 2.0 * (x * x + z * z);
  m[5] = 2.0 * (y * z - w * x);
  m[6] = 2.0 * (x * z - w * y);
  m[7] = 2.0 * (y * z + w * x);
  m[8] = 1.0 - 2.0 * (x * x + y * y);
}

static void MatrixVector(const double m[9], const double v[3], double result[3]) {
  for (int i = 0; i < 3; ++i) {
    result[i] = m[3 * i] * v[0] + m[3 * i + 1] * v[1] + m[3 * i + 2] * v[2];
  }
}

// Writes the row-major 3x3 `block` * scale at (row, column) of a column-major matrix with
// `rows` rows.
static void SetBlock(float* matrix, int rows, int row, int column, const double block[9],
                     double scale) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      matrix[(row + i) + (column + j) * rows] = (float)(block[3 * i + j] * scale);
    }
  }
}

static void SetIdentity(float* matrix, int rows, int row, int column, int size, double scale) {
  for (int i = 0; i < size; ++i) {
    matrix[(row + i) + (column + i) * rows] = (float)scale;
  }
}

void kfcore_ins_init(KfcoreIns* ins, const KfcoreInsProcessNoise* process_noise, double gravity,
                     double covariance_period) {
  memset(ins, 0, sizeof(*ins));
  ins->process_noise = *process_noise;
  ins->gravity = gravity;
  ins->covariance_period = covariance_period;
  ins->rotation[0] = 1.0;
  SetIdentity(ins->u, kStateSize, 0, 0, kStateSize, 1.0);
  for (int i = 0; i < kStateSize; ++i) {
    ins->d[i] = 1.0f;
  }
}

void kfcore_ins_reset(KfcoreIns* ins, const double position[3], const double rotation[4],
                      const double velocity[3], const double gyroscope_bias[3],
                      const double* initial_std) {
  memcpy(ins->position, position, sizeof(ins->position));
  memcpy(ins->rotation, rotation, sizeof(ins->rotation));
  memcpy(ins->velocity, velocity, sizeof(ins->velocity));
  memset(ins->accelerometer_bias, 0, sizeof(ins->accelerometer_bias));
  memcpy(ins->gyroscope_bias, gyroscope_bias, sizeof(ins->gyroscope_bias));
  memset(ins->u, 0, sizeof(ins->u));
  SetIdentity(ins->u, kStateSize, 0, 0, kStateSize, 1.0);
  for (int i = 0; i < kStateSize; ++i) {
    ins->d[i] = (float)(initial_std[i] * initial_std[i]);
  }
  ins->pending_covariance_dt = 0.0;
}

// `rotation` row-major body to world; `specific_force_world` m/s^2.
static void PredictCovariance(KfcoreIns* ins, const double rotation[9],
                              const double specific_force_world[3], double dt) {
  // error transition, first order: Phi = I + F dt
  float transition[kStateSize * kStateSize] = {0};
  SetIdentity(transition, kStateSize, 0, 0, kStateSize, 1.0);
  SetIdentity(transition, kStateSize, KFCORE_INS_POSITION, KFCORE_INS_VELOCITY, 3, dt);
  const double* f = specific_force_world;
  const double force_hat[9] = {0.0, -f[2], f[1], f[2], 0.0, -f[0], -f[1], f[0], 0.0};
  SetBlock(transition, kStateSize, KFCORE_INS_VELOCITY, KFCORE_INS_ATTITUDE, force_hat, -dt);
  SetBlock(transition, kStateSize, KFCORE_INS_VELOCITY, KFCORE_INS_ACCELEROMETER_BIAS, rotation,
           -dt);
  SetBlock(transition, kStateSize, KFCORE_INS_ATTITUDE, KFCORE_INS_GYROSCOPE_BIAS, rotation, -dt);

  float noise_input[kStateSize * kNoiseSize] = {0};
  SetBlock(noise_input, kStateSize, KFCORE_INS_VELOCITY, 0, rotation, 1.0);
  SetBlock(noise_input, kStateSize, KFCORE_INS_ATTITUDE, 3, rotation, 1.0);
  SetIdentity(noise_input, kStateSize, KFCORE_INS_ACCELEROMETER_BIAS, 6, 3, 1.0);
  SetIdentity(noise_input, kStateSize, KFCORE_INS_GYROSCOPE_BIAS, 9, 3, 1.0);
  SetIdentity(noise_input, kStateSize, KFCORE_INS_POSITION, 12, 9, 1.0);
  const KfcoreInsProcessNoise* noise = &ins->process_noise;
  const double densities[kNoiseSize / 3] = {
      noise->accelerometer,  noise->gyroscope, noise->accelerometer_bias,
      noise->gyroscope_bias, noise->position,  noise->velocity,
      noise->attitude};
  float noise_variance[kNoiseSize];
  for (int i = 0; i < kNoiseSize; ++i) {
    noise_variance[i] = (float)(densities[i / 3] * densities[i / 3] * dt);
  }
  // The error state is zero after every correction, so only the covariance is predicted.
  kalman_udu_predict(NULL, ins->u, ins->d, transition, noise_input, noise_variance, kStateSize,
                     kNoiseSize);
}

void kfcore_ins_predict(KfcoreIns* ins, const double acceleration[3],
                        const double angular_velocity[3], double dt) {
  if (dt <= 0.0) {
    return;
  }
  double rotation[9];
  QuaternionToMatrix(ins->rotation, rotation);
  double specific_force[3];
  for (int i = 0; i < 3; ++i) {
    specific_force[i] = acceleration[i] - ins->accelerometer_bias[i];
  }
  double specific_force_world[3];
  MatrixVector(rotation, specific_force, specific_force_world);

  ins->pending_covariance_dt += dt;
  if (ins->pending_covariance_dt >= (1.0 - kCovariancePeriodTolerance) * ins->covariance_period) {
    PredictCovariance(ins, rotation, specific_force_world, ins->pending_covariance_dt);
    ins->pending_covariance_dt = 0.0;
  }

  // nominal state: trapezoidal position
  specific_force_world[2] -= ins->gravity;
  double rotation_vector[3];
  for (int i = 0; i < 3; ++i) {
    const double previous_velocity = ins->velocity[i];
    ins->velocity[i] += specific_force_world[i] * dt;
    ins->position[i] += 0.5 * dt * (previous_velocity + ins->velocity[i]);
    rotation_vector[i] = (angular_velocity[i] - ins->gyroscope_bias[i]) * dt;
  }
  double step[4];
  QuaternionExp(rotation_vector, step);
  const double previous_rotation[4] = {ins->rotation[0], ins->rotation[1], ins->rotation[2],
                                       ins->rotation[3]};
  QuaternionMultiply(previous_rotation, step, ins->rotation);
  QuaternionNormalize(ins->rotation);
}

static void Correct(KfcoreIns* ins, const float error[kStateSize]) {
  double correction[4];
  double attitude_error[3];
  for (int i = 0; i < 3; ++i) {
    ins->position[i] += error[KFCORE_INS_POSITION + i];
    ins->velocity[i] += error[KFCORE_INS_VELOCITY + i];
    attitude_error[i] = error[KFCORE_INS_ATTITUDE + i];
    ins->accelerometer_bias[i] += error[KFCORE_INS_ACCELEROMETER_BIAS + i];
    ins->gyroscope_bias[i] += error[KFCORE_INS_GYROSCOPE_BIAS + i];
  }
  QuaternionExp(attitude_error, correction);
  const double previous_rotation[4] = {ins->rotation[0], ins->rotation[1], ins->rotation[2],
                                       ins->rotation[3]};
  QuaternionMultiply(correction, previous_rotation, ins->rotation);
  QuaternionNormalize(ins->rotation);
}

bool kfcore_ins_update(KfcoreIns* ins, const double* residual, const double* jacobian,
                       const double* variance, int rows) {
  if (rows < 1 || rows > KFCORE_INS_MAX_UPDATE_ROWS) {
    return false;
  }
  float measurement[KFCORE_INS_MAX_UPDATE_ROWS];
  float measurement_covariance[KFCORE_INS_MAX_UPDATE_ROWS * KFCORE_INS_MAX_UPDATE_ROWS] = {0};
  // The row-major jacobian is the column-major transpose KFCore takes.
  float jacobian_transpose[kStateSize * KFCORE_INS_MAX_UPDATE_ROWS];
  for (int i = 0; i < rows; ++i) {
    measurement[i] = (float)residual[i];
    measurement_covariance[i + i * rows] = (float)variance[i];
  }
  for (int i = 0; i < rows * kStateSize; ++i) {
    jacobian_transpose[i] = (float)jacobian[i];
  }
  float error[kStateSize] = {0};
  const int status = kalman_udu(error, ins->u, ins->d, measurement, measurement_covariance,
                                jacobian_transpose, kStateSize, rows, /*chi2_threshold=*/0.0f,
                                /*downweight_outlier=*/0);
  Correct(ins, error);
  return status == 0;
}

void kfcore_ins_covariance(const KfcoreIns* ins, double* covariance) {
  for (int i = 0; i < kStateSize; ++i) {
    for (int j = 0; j < kStateSize; ++j) {
      double sum = 0.0;
      for (int k = 0; k < kStateSize; ++k) {
        sum += (double)ins->u[i + k * kStateSize] * ins->d[k] * ins->u[j + k * kStateSize];
      }
      covariance[i * kStateSize + j] = sum;
    }
  }
}
