// Copyright 2022 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#ifndef IKFOM_BACKEND_COMMON_H_
#define IKFOM_BACKEND_COMMON_H_

#include "common/math/sophus_utils.h"
#include "common/sensor/imu_data.h"
#include "common/sensor/imu_intrinsics.h"
#include "ikfom/toolkit/esekfom/esekfom.hpp"

// no template can be set as input of MTK_BUILD_MANIFOLD
namespace MTK {
// length of the S2 variable: 98099 / 10000; choose e1 as the original point of rotation: 1
// for the gravity direction
typedef S2<double, 98099, 10000, 1> Gravity;
typedef vect<3, double> vect3;
typedef SO3<double> SO3d;
typedef vect<1, double> vect1;
typedef vect<2, double> vect2;
}  // namespace MTK

namespace mapmind::ikfom {

/////  define the manifold for the kalman filter /////////

struct ImuIkfom {
  enum { DOF = 6, DIM = 6 };
  IKFOM_MTK_DECLATION(ImuIkfom, MTK::vect3, acc, 0, 0, MTK::vect3, gyro, 3, 3)
  IKFOM_MTK_FUNCTIONS(acc, gyro)
};

struct ImuNoiseIkfom {
  enum { DOF = 12, DIM = 12 };
  IKFOM_MTK_DECLATION(ImuNoiseIkfom, MTK::vect3, ng, 0, 0, MTK::vect3, na, 3, 3, MTK::vect3, nbg, 6,
                      6, MTK::vect3, nba, 9, 9)
  IKFOM_MTK_FUNCTIONS(ng, na, nbg, nba)
};

struct ChassisIkfom {
  enum { DOF = 6, DIM = 6 };
  IKFOM_MTK_DECLATION(ChassisIkfom, MTK::vect3, vel, 0, 0, MTK::vect3, angularvel, 3, 3)
  IKFOM_MTK_FUNCTIONS(vel, angularvel)
};

struct ChassisNoiseIkfom {
  enum { DOF = 6, DIM = 6 };
  IKFOM_MTK_DECLATION(ChassisNoiseIkfom, MTK::vect3, nvel, 0, 0, MTK::vect3, nangularvel, 3, 3)
  IKFOM_MTK_FUNCTIONS(nvel, nangularvel)
};

MTK::get_cov<ImuNoiseIkfom>::type GetProcessNoiseCov(double noise_acc = 0.0001,
                                                     double noise_gyr = 0.0001,
                                                     double noise_bias_acc = 0.003,
                                                     double noise_bias_gyr = 0.003);

MTK::get_cov<ImuNoiseIkfom>::type GetProcessNoiseCov(
    const mapmind::sensor::ImuIntrinsics& intrinsics);

// pose resiudal is  Log(T_measure * T^{T}).
// which has better performance compare to Log(T) - Log(T_measure)
Eigen::VectorXd PoseMeasurementResidual(const Eigen::VectorXd& obs, const Eigen::VectorXd& raw);
Eigen::VectorXd RotationMeasurementResidual(const Eigen::VectorXd& obs, const Eigen::VectorXd& raw);

// rotation: rotation body to world
// inclination: https://en.wikipedia.org/wiki/Magnetic_dip
bool RotationFromMagneticGravity(const Eigen::Vector3d& gravity, const Eigen::Vector3d& magnetic,
                                 Eigen::Matrix3d* rotation, Eigen::Matrix3d* inclination);
double GetInclinationAngle(const Eigen::Matrix3d& inclination);

}  // namespace mapmind::ikfom

#endif  // IKFOM_BACKEND_COMMON_H_
