// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#include "common/math/quaternion.h"

#include <math.h>

void quaternion_multiply(const double a[4], const double b[4], double result[4]) {
  const double w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  const double x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  const double y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  const double z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
  result[0] = w;
  result[1] = x;
  result[2] = y;
  result[3] = z;
}

void quaternion_exp(const double rotation_vector[3], double result[4]) {
  const double* v = rotation_vector;
  const double theta_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  double real;
  double imaginary;
  // Sophus's threshold: below it the series is exact in double
  if (theta_sq * theta_sq < 1e-30) {
    real = 1.0 - theta_sq / 8.0 + theta_sq * theta_sq / 384.0;
    imaginary = 0.5 - theta_sq / 48.0 + theta_sq * theta_sq / 3840.0;
  } else {
    const double theta = sqrt(theta_sq);
    real = cos(0.5 * theta);
    imaginary = sin(0.5 * theta) / theta;
  }
  const double x = imaginary * v[0];
  const double y = imaginary * v[1];
  const double z = imaginary * v[2];
  result[0] = real;
  result[1] = x;
  result[2] = y;
  result[3] = z;
}

void quaternion_normalize(double q[4]) {
  const double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (int i = 0; i < 4; ++i) {
    q[i] /= norm;
  }
}

void quaternion_to_matrix(const double q[4], double matrix[9]) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  matrix[0] = 1.0 - 2.0 * (y * y + z * z);
  matrix[1] = 2.0 * (x * y - w * z);
  matrix[2] = 2.0 * (x * z + w * y);
  matrix[3] = 2.0 * (x * y + w * z);
  matrix[4] = 1.0 - 2.0 * (x * x + z * z);
  matrix[5] = 2.0 * (y * z - w * x);
  matrix[6] = 2.0 * (x * z - w * y);
  matrix[7] = 2.0 * (y * z + w * x);
  matrix[8] = 1.0 - 2.0 * (x * x + y * y);
}

void quaternion_rotate(const double q[4], const double v[3], double result[3]) {
  double matrix[9];
  quaternion_to_matrix(q, matrix);
  double rotated[3];
  for (int i = 0; i < 3; ++i) {
    rotated[i] = matrix[3 * i] * v[0] + matrix[3 * i + 1] * v[1] + matrix[3 * i + 2] * v[2];
  }
  for (int i = 0; i < 3; ++i) {
    result[i] = rotated[i];
  }
}
