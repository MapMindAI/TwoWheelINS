// Copyright 2026 MapMind
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_MATH_QUATERNION_H_
#define COMMON_MATH_QUATERNION_H_

#ifdef __cplusplus
extern "C" {
#endif

// Quaternions are (w, x, y, z); rotation vectors are axis * angle (rad). Outputs may alias inputs.

// result = a * b (apply b, then a)
void quaternion_multiply(const double a[4], const double b[4], double result[4]);

// Unit quaternion of `rotation_vector`, with a series near zero.
void quaternion_exp(const double rotation_vector[3], double result[4]);

void quaternion_normalize(double q[4]);

// Row-major rotation matrix of the unit quaternion `q`.
void quaternion_to_matrix(const double q[4], double matrix[9]);

// result = q * v * q^-1 for the unit quaternion `q`.
void quaternion_rotate(const double q[4], const double v[3], double result[3]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_MATH_QUATERNION_H_
