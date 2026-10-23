// Copyright 2022 MapMind
// SPDX-License-Identifier: GPL-2.0-only

#include "ikfom/backend/ikf_ins.h"

namespace mapmind::ikfom::ins {

Eigen::Matrix<double, StateIkfom::DIM, 1> GetVelocityField(StateIkfom& state,  // NOLINT
                                                           const ImuIkfom& input) {
  Eigen::Matrix<StateIkfom::scalar, StateIkfom::DIM, 1> res =
      Eigen::Matrix<StateIkfom::scalar, StateIkfom::DIM, 1>::Zero();
  res.block(state.pos.IDX, 0, 3, 1) = state.vel;
  res.block(state.rot.IDX, 0, 3, 1) = input.gyro - state.bg;
  res.block(state.vel.IDX, 0, 3, 1) = state.rot * (input.acc - state.ba) + state.grav.get_vect();
  return res;
}

Eigen::Matrix<double, StateIkfom::DIM, StateIkfom::DOF> DVelocityFieldDx(
    StateIkfom& state, const ImuIkfom& input) {  // NOLINT
  Eigen::Matrix<StateIkfom::scalar, StateIkfom::DIM, StateIkfom::DOF> cov =
      Eigen::Matrix<StateIkfom::scalar, StateIkfom::DIM, StateIkfom::DOF>::Zero();
  cov.template block<3, 3>(state.pos.IDX, state.vel.IDX) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(state.vel.IDX, state.rot.IDX) =
      -state.rot.toRotationMatrix() * Sophus::SO3d::hat(input.acc - state.ba);
  cov.template block<3, 3>(state.vel.IDX, state.ba.IDX) = -state.rot.toRotationMatrix();
  Eigen::Matrix<StateIkfom::scalar, 2, 1> vec = Eigen::Matrix<StateIkfom::scalar, 2, 1>::Zero();
  Eigen::Matrix<StateIkfom::scalar, 3, 2> grav_matrix;
  state.S2_Mx(grav_matrix, vec, state.grav.IDX);
  cov.template block<3, 2>(state.vel.IDX, state.grav.IDX) = grav_matrix;
  cov.template block<3, 3>(state.rot.IDX, state.bg.IDX) = -Eigen::Matrix3d::Identity();
  return cov;
}

Eigen::Matrix<double, StateIkfom::DIM, ImuNoiseIkfom::DIM> DVelocityFieldDw(
    StateIkfom& state, const ImuIkfom&) {  // NOLINT
  Eigen::Matrix<double, StateIkfom::DIM, ImuNoiseIkfom::DIM> cov =
      Eigen::Matrix<double, StateIkfom::DIM, ImuNoiseIkfom::DIM>::Zero();
  static ImuNoiseIkfom tmp;
  cov.template block<3, 3>(state.vel.IDX, tmp.na.IDX) = -state.rot.toRotationMatrix();
  cov.template block<3, 3>(state.rot.IDX, tmp.ng.IDX) = -Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(state.bg.IDX, tmp.nbg.IDX) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(state.ba.IDX, tmp.nba.IDX) = Eigen::Matrix3d::Identity();
  return cov;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> PoseMeasurement(StateIkfom& state,  // NOLINT
                                                         bool& /*valid*/) {  // NOLINT
  Eigen::Matrix<double, 6, 1> result;
  result.segment<3>(0) = state.pos;
  result.segment<3>(3) = Sophus::SO3d(state.rot).log();
  return result;
}

Eigen::Matrix<double, Eigen::Dynamic, StateIkfom::DOF> DPoseMeasurementDx(
    const Eigen::VectorXd& residual, const Eigen::VectorXd&, const StateIkfom& state, bool) {
  Eigen::Matrix<double, 6, StateIkfom::DOF> derivative =
      Eigen::Matrix<double, 6, StateIkfom::DOF>::Zero();
  derivative.block(0, state.pos.IDX, 3, 3) = Eigen::Matrix3d::Identity();
  derivative.block(3, state.rot.IDX, 3, 3) =
      math::sophus::InverseRightJacobian(residual.segment<3>(3)) * state.rot.matrix();
  return derivative;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DPoseMeasurementDv(
    StateIkfom& /*state*/, bool& /*valid*/) {  // NOLINT
  return Eigen::Matrix<double, 6, 6>::Identity();
}

Eigen::Matrix<double, Eigen::Dynamic, 1> PositionMeasurement(StateIkfom& state,  // NOLINT
                                                             bool& /*valid*/) {  // NOLINT
  return state.pos;
}

Eigen::Matrix<double, Eigen::Dynamic, StateIkfom::DOF> DPositionMeasurementDx(
    const Eigen::VectorXd&, const Eigen::VectorXd&, const StateIkfom& state, bool) {
  Eigen::Matrix<double, 3, StateIkfom::DOF> derivative =
      Eigen::Matrix<double, 3, StateIkfom::DOF>::Zero();
  derivative.block(0, state.pos.IDX, 3, 3) = Eigen::Matrix3d::Identity();
  return derivative;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DPositionMeasurementDv(
    StateIkfom& /*state*/, bool& /*valid*/) {  // NOLINT
  return Eigen::Matrix<double, 3, 3>::Identity();
}

Eigen::Matrix<double, Eigen::Dynamic, 1> WorldVelocityMeasurement(StateIkfom& state,  // NOLINT
                                                                  bool& /*valid*/) {  // NOLINT
  return state.vel;
}

Eigen::Matrix<double, Eigen::Dynamic, StateIkfom::DOF> DWorldVelocityMeasurementDx(
    const Eigen::VectorXd&, const Eigen::VectorXd&, const StateIkfom& state, bool) {
  Eigen::Matrix<double, 3, StateIkfom::DOF> derivative =
      Eigen::Matrix<double, 3, StateIkfom::DOF>::Zero();
  derivative.block(0, state.vel.IDX, 3, 3) = Eigen::Matrix3d::Identity();
  return derivative;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DWorldVelocityMeasurementDv(
    StateIkfom& /*state*/, bool& /*valid*/) {  // NOLINT
  return Eigen::Matrix<double, 3, 3>::Identity();
}

Eigen::Matrix<double, Eigen::Dynamic, 1> LocalVelocityMeasurement(StateIkfom& state,  // NOLINT
                                                                  bool& /*valid*/) {  // NOLINT
  return state.rot.inverse() * state.vel;
}

Eigen::Matrix<double, Eigen::Dynamic, StateIkfom::DOF> DLocalVelocityMeasurementDx(
    const Eigen::VectorXd&, const Eigen::VectorXd&, const StateIkfom& state, bool) {
  Eigen::Matrix<double, 3, StateIkfom::DOF> derivative =
      Eigen::Matrix<double, 3, StateIkfom::DOF>::Zero();
  Eigen::Matrix3d rot_inv = state.rot.matrix().transpose();
  derivative.block(0, state.vel.IDX, 3, 3) = rot_inv;
  derivative.block(0, state.rot.IDX, 3, 3) = Sophus::SO3d::hat(rot_inv * state.vel);
  return derivative;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DLocalVelocityMeasurementDv(
    StateIkfom& /*state*/, bool& /*valid*/) {  // NOLINT
  return Eigen::Matrix<double, 3, 3>::Identity();
}

}  // namespace mapmind::ikfom::ins
