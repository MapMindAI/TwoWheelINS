// Copyright 2022 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#ifndef IKFOM_BACKEND_IKF_INS_H_
#define IKFOM_BACKEND_IKF_INS_H_

#include "ikfom/backend/common.h"

namespace mapmind::ikfom::ins {

struct StateIkfom {
  enum { DOF = 17, DIM = 18 };
  IKFOM_MTK_DECLATION(StateIkfom,                 // declare state
                      MTK::vect3, pos, 0, 0,      // body position in world
                      MTK::SO3d, rot, 3, 3,       // body rotation in world
                      MTK::vect3, vel, 6, 6,      // body velocity in world
                      MTK::vect3, bg, 9, 9,       // gyroscope bias
                      MTK::vect3, ba, 12, 12,     // accelerometer bias
                      MTK::Gravity, grav, 15, 15  // gravity
  )
  IKFOM_MTK_FUNCTIONS(pos, rot, vel, bg, ba, grav)
};

// Implement the vector field and its differentiation
// notice S2 has length of 3 and dimension of 2
Eigen::Matrix<double, StateIkfom::DIM, 1> GetVelocityField(StateIkfom& state,  // NOLINT
                                                           const ImuIkfom& input);

Eigen::Matrix<double, StateIkfom::DIM, StateIkfom::DOF> DVelocityFieldDx(
    StateIkfom& state, const ImuIkfom& input);  // NOLINT

Eigen::Matrix<double, StateIkfom::DIM, ImuNoiseIkfom::DIM> DVelocityFieldDw(
    StateIkfom& state, const ImuIkfom& input);  // NOLINT

// measurement of imu pose
Eigen::Matrix<double, Eigen::Dynamic, 1> PoseMeasurement(StateIkfom& state,  // NOLINT
                                                         bool& valid);       // NOLINT

Eigen::Matrix<double, Eigen::Dynamic, StateIkfom::DOF> DPoseMeasurementDx(
    const Eigen::VectorXd& residual, const Eigen::VectorXd& obs, const StateIkfom& state,
    bool valid);

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DPoseMeasurementDv(StateIkfom& state,
                                                                         bool& valid);  // NOLINT

// measurement of imu position (in world reference frame), e.g. GPS
Eigen::Matrix<double, Eigen::Dynamic, 1> PositionMeasurement(StateIkfom& state,  // NOLINT
                                                             bool& valid);       // NOLINT

Eigen::Matrix<double, Eigen::Dynamic, StateIkfom::DOF> DPositionMeasurementDx(
    const Eigen::VectorXd& residual, const Eigen::VectorXd& obs, const StateIkfom& state,
    bool valid);

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DPositionMeasurementDv(
    StateIkfom& state,
    bool& valid);  // NOLINT

// measurement of imu velocity (in world reference frame)
Eigen::Matrix<double, Eigen::Dynamic, 1> WorldVelocityMeasurement(StateIkfom& state,  // NOLINT
                                                                  bool& valid);       // NOLINT

Eigen::Matrix<double, Eigen::Dynamic, StateIkfom::DOF> DWorldVelocityMeasurementDx(
    const Eigen::VectorXd& residual, const Eigen::VectorXd& obs, const StateIkfom& state,
    bool valid);

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DWorldVelocityMeasurementDv(
    StateIkfom& state, bool& valid);  // NOLINT

// measurement of imu velocity (in local reference frame)
Eigen::Matrix<double, Eigen::Dynamic, 1> LocalVelocityMeasurement(StateIkfom& state,  // NOLINT
                                                                  bool& valid);       // NOLINT

Eigen::Matrix<double, Eigen::Dynamic, StateIkfom::DOF> DLocalVelocityMeasurementDx(
    const Eigen::VectorXd& residual, const Eigen::VectorXd& obs, const StateIkfom& state,
    bool valid);

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DLocalVelocityMeasurementDv(
    StateIkfom& state, bool& valid);  // NOLINT

}  // namespace mapmind::ikfom::ins

#endif  // IKFOM_BACKEND_IKF_INS_H_
