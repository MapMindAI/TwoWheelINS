/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// Modified 2022-2026 by MapMind from IKFoM (https://github.com/hku-mars/IKFoM).

#ifndef IKFOM_TOOLKIT_ESEKFOM_UTIL_HPP_
#define IKFOM_TOOLKIT_ESEKFOM_UTIL_HPP_

#include <Eigen/Core>
#include "ikfom/toolkit/mtk/src/mtkmath.hpp"
#include "sophus/se3.hpp"

namespace esekfom {

template <typename T1, typename T2>
class is_same {
 public:
  operator bool() { return false; }
};

template <typename T1>
class is_same<T1, T1> {
 public:
  operator bool() { return true; }
};

template <typename T>
static T id(const T& x) {
  return x;
}

template <typename StateType, typename Scalar = typename StateType::scalar>
void UpdateCovarianceTransitionSO3(
    const StateType& state, const Eigen::Matrix<Scalar, StateType::DOF, 1>& delta,
    Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF>* transition,
    Eigen::Matrix<Scalar, StateType::DOF, 1>* delta_new) {
  MTK::vect<3, Scalar> seg_SO3;
  for (const auto& id_pair : state.SO3_state) {
    int idx = id_pair.first;
    seg_SO3 = delta.block(idx, 0, 3, 1);
    Eigen::Matrix<Scalar, 3, 3> res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
    delta_new->template block<3, 1>(idx, 0) =
        res_temp_SO3 * delta_new->template block<3, 1>(idx, 0);
    transition->block(idx, 0, 3, StateType::DOF) =
        res_temp_SO3 * transition->block(idx, 0, 3, StateType::DOF);
    transition->block(0, idx, StateType::DOF, 3) =
        transition->block(0, idx, StateType::DOF, 3) * res_temp_SO3.transpose();
  }
}

template <typename StateType, typename Scalar = typename StateType::scalar>
void UpdateCovarianceTransitionS2(const StateType& state, const StateType& state_propagated,
                                  const Eigen::Matrix<Scalar, StateType::DOF, 1>& delta,
                                  Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF>* transition,
                                  Eigen::Matrix<Scalar, StateType::DOF, 1>* delta_new) {
  Eigen::Matrix<Scalar, 2, 3> Nx;
  Eigen::Matrix<Scalar, 3, 2> Mx;
  for (const auto& id_pair : state.S2_state) {
    int idx = id_pair.first;
    MTK::vect<2, Scalar> seg_S2 = delta.block(idx, 0, 2, 1);

    state.S2_Nx_yy(Nx, idx);
    state_propagated.S2_Mx(Mx, seg_S2, idx);

    Eigen::Matrix<Scalar, 2, 2> res_temp_S2 = Nx * Mx;
    delta_new->template block<2, 1>(idx, 0) = res_temp_S2 * delta_new->template block<2, 1>(idx, 0);
    transition->block(idx, 0, 2, StateType::DOF) =
        res_temp_S2 * transition->block(idx, 0, 2, StateType::DOF);
    transition->block(0, idx, StateType::DOF, 2) =
        transition->block(0, idx, StateType::DOF, 2) * res_temp_S2.transpose();
  }
}

template <typename StateType, typename Scalar = typename StateType::scalar>
void UpdateFilterSO3(const StateType& state, const Eigen::Matrix<Scalar, StateType::DOF, 1>& delta,
                     int size_k, Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF>* P,
                     Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF>* L,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* K) {
  Eigen::Matrix<Scalar, 3, 3> tmp, tmp_t;
  for (const auto& id_pair : state.SO3_state) {
    int idx = id_pair.first;
    MTK::vect<3, Scalar> seg_SO3 = delta.block(idx, 0, 3, 1);
    tmp = MTK::A_matrix(seg_SO3).transpose();
    tmp_t = tmp.transpose();
    L->block(idx, 0, 3, StateType::DOF) = tmp * P->block(idx, 0, 3, StateType::DOF);
    K->block(idx, 0, 3, size_k) = tmp * K->block(idx, 0, 3, size_k);
    L->block(0, idx, StateType::DOF, 3) = L->block(0, idx, StateType::DOF, 3) * tmp_t;
    P->block(0, idx, StateType::DOF, 3) = P->block(0, idx, StateType::DOF, 3) * tmp_t;
  }
}

template <typename StateType, typename Scalar = typename StateType::scalar>
void UpdateFilterS2(const StateType& state, const StateType& state_propagated,
                    const Eigen::Matrix<Scalar, StateType::DOF, 1>& delta, int size_k,
                    Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF>* P,
                    Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF>* L,
                    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* K) {
  Eigen::Matrix<Scalar, 2, 3> Nx;
  Eigen::Matrix<Scalar, 3, 2> Mx;
  Eigen::Matrix<Scalar, 2, 2> tmp, tmp_t;
  for (const auto& id_pair : state.S2_state) {
    int idx = id_pair.first;
    MTK::vect<2, Scalar> seg_S2 = delta.block(idx, 0, 2, 1);

    state.S2_Nx_yy(Nx, idx);
    state_propagated.S2_Mx(Mx, seg_S2, idx);
    tmp = Nx * Mx;
    tmp_t = tmp.transpose();

    L->block(idx, 0, 2, StateType::DOF) = tmp * P->block(idx, 0, 2, StateType::DOF);
    K->block(idx, 0, 2, size_k) = tmp * K->block(idx, 0, 2, size_k);
    L->block(0, idx, StateType::DOF, 2) = L->block(0, idx, StateType::DOF, 2) * tmp_t;
    P->block(0, idx, StateType::DOF, 2) = P->block(0, idx, StateType::DOF, 2) * tmp_t;
  }
}

template <typename Scalar>
inline Eigen::Quaternion<Scalar> DeltaQuaternion(const Eigen::Matrix<Scalar, 3, 1>& theta,
                                                 Scalar dt) {
  Eigen::Quaternion<Scalar> dq;
  Eigen::Matrix<Scalar, 3, 1> half_theta = (0.5 * dt) * theta;
  dq.w() = 1.0;
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, 3, 3> InverseRightJacobian(const Eigen::Matrix<Scalar, 3, 1>& phi) {
  Scalar theta2 = phi.squaredNorm();
  const Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
  if (theta2 <= 1e-8) {
    return Eigen::Matrix<Scalar, 3, 3>::Identity() + 0.5 * phi_hat;
  }
  Scalar theta = std::sqrt(theta2);
  const Eigen::Matrix<Scalar, 3, 1> phi_scaled = (Scalar(1.0) / theta) * phi;
  Eigen::Matrix<Scalar, 3, 3> outer_product = phi_scaled * phi_scaled.transpose();
  return Eigen::Matrix<Scalar, 3, 3>::Identity() + 0.5 * phi_hat +
         (1 - 0.5 * theta * (1 + std::cos(theta)) / std::sin(theta)) *
             (outer_product - Eigen::Matrix<Scalar, 3, 3>::Identity());
}

// reference
// https://github.com/KumarRobotics/msckf_vio/blob/master/src/msckf_vio.cpp
template <typename StateType, typename Scalar = typename StateType::scalar>
void RungeKuttaImuUpdate(const Scalar& dt, const Eigen::Matrix<Scalar, 3, 1>& gyr,
                         const Eigen::Matrix<Scalar, 3, 1>& acc, StateType* state) {
  Scalar dt_half = 0.5 * dt;
  const Eigen::Matrix<Scalar, 3, 1>& grav_vec = state->grav.get_vect();

  Eigen::Matrix<Scalar, 3, 1> acc_un = acc - state->ba;
  Eigen::Matrix<Scalar, 3, 1> gyr_un = gyr - state->bg;

  Eigen::Quaternion<Scalar> dq_dt =
      (Sophus::SO3<Scalar>(state->rot) * Sophus::SO3<Scalar>::exp(dt * gyr_un)).unit_quaternion();
  Eigen::Quaternion<Scalar> dq_dt2 =
      (Sophus::SO3<Scalar>(state->rot) * Sophus::SO3<Scalar>::exp(dt_half * gyr_un))
          .unit_quaternion();

  // Eigen::Quaternion<Scalar> dq_dt = state->rot * DeltaQuaternion(gyr_un, dt);
  // dq_dt.normalize();
  // Eigen::Quaternion<Scalar> dq_dt2 = state->rot * DeltaQuaternion(gyr_un, dt_half);
  // dq_dt2.normalize();

  // k1 = f(tn, yn)
  Eigen::Matrix<Scalar, 3, 1> k1_v_dot = state->rot * acc_un + grav_vec;
  Eigen::Matrix<Scalar, 3, 1> k1_p_dot = state->vel;

  // k2 = f(tn+dt/2, yn+k1*dt/2)
  Eigen::Matrix<Scalar, 3, 1> k1_v = state->vel + dt_half * k1_v_dot;
  Eigen::Matrix<Scalar, 3, 1> k2_v_dot = dq_dt2 * acc_un + grav_vec;
  Eigen::Matrix<Scalar, 3, 1>& k2_p_dot = k1_v;

  // k3 = f(tn+dt/2, yn+k2*dt/2)
  Eigen::Matrix<Scalar, 3, 1> k2_v = state->vel + dt_half * k2_v_dot;
  Eigen::Matrix<Scalar, 3, 1> k3_v_dot = dq_dt2 * acc_un + grav_vec;
  Eigen::Matrix<Scalar, 3, 1>& k3_p_dot = k2_v;

  // k4 = f(tn+dt, yn+k3*dt)
  Eigen::Matrix<Scalar, 3, 1> k3_v = state->vel + dt * k3_v_dot;
  Eigen::Matrix<Scalar, 3, 1> k4_v_dot = dq_dt * acc_un + grav_vec;
  Eigen::Matrix<Scalar, 3, 1>& k4_p_dot = k3_v;

  // yn+1 = yn + dt/6*(k1+2*k2+2*k3+k4)
  Scalar dt_over_six = dt / 6.0;
  state->rot = dq_dt;
  state->vel = state->vel + dt_over_six * (k1_v_dot + 2.0 * k2_v_dot + 2.0 * k3_v_dot + k4_v_dot);
  state->pos = state->pos + dt_over_six * (k1_p_dot + 2.0 * k2_p_dot + 2.0 * k3_p_dot + k4_p_dot);
}

template <typename StateType, typename UpdateInput, typename Scalar = typename StateType::scalar>
void LinearImuUpdate(const Scalar& dt, const UpdateInput& input, StateType* state) {
  Eigen::Matrix<Scalar, 3, 1> un_gyr = input.gyro - state->bg;
  Eigen::Quaternion<Scalar> dq_dt =
      (Sophus::SO3<Scalar>(state->rot) * Sophus::SO3<Scalar>::exp(dt * un_gyr)).unit_quaternion();

  const Eigen::Matrix<Scalar, 3, 1>& grav_vec = state->grav.get_vect();
  Eigen::Vector3d un_acc = state->rot * (input.acc - state->ba) + grav_vec;

  state->rot = dq_dt;
  state->pos = state->pos + dt * state->vel + (0.5 * dt * dt) * un_acc;
  state->vel = state->vel + dt * un_acc;
}

template <typename StateType, typename UpdateInput, typename Scalar = typename StateType::scalar>
void MidPointImuUpdate(const Scalar& dt, const UpdateInput& last_input, const UpdateInput& input,
                       StateType* state) {
  Eigen::Matrix<Scalar, 3, 1> un_gyr = 0.5 * (last_input.gyro + input.gyro) - state->bg;
  Eigen::Quaternion<Scalar> dq_dt =
      (Sophus::SO3<Scalar>(state->rot) * Sophus::SO3<Scalar>::exp(dt * un_gyr)).unit_quaternion();

  const Eigen::Matrix<Scalar, 3, 1>& grav_vec = state->grav.get_vect();

  Eigen::Vector3d acc_w1 = state->rot * (last_input.acc - state->ba);
  Eigen::Vector3d acc_w2 = dq_dt * (input.acc - state->ba);
  Eigen::Vector3d un_acc = 0.5 * (acc_w1 + acc_w2) + grav_vec;

  state->rot = dq_dt;
  state->pos = state->pos + dt * state->vel + (0.5 * dt * dt) * un_acc;
  state->vel = state->vel + dt * un_acc;
}

template <typename StateType, typename UpdateInput, int NoiseDof,
          typename Scalar = typename StateType::scalar>
void ImuCovarianceUpdate(const Scalar& dt, const StateType& state, const UpdateInput& input,
                         const Eigen::Matrix<Scalar, NoiseDof, NoiseDof>& noise_covariance,
                         Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF>* covariance) {
  MTK::vect<3, Scalar> gyr_un = dt * (input.gyro - state.bg);
  Eigen::Matrix<Scalar, 3, 3> tmp_so3 = MTK::A_matrix(gyr_un);

  // add state update terms
  Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF> cov_dx =
      Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF>::Identity();

  Eigen::Matrix<Scalar, 3, 3> dt_identity = dt * Eigen::Matrix<Scalar, 3, 3>::Identity();
  Eigen::Matrix<Scalar, 3, 3> dt_rot = dt * state.rot.toRotationMatrix();
  cov_dx.template block<3, 3>(state.pos.IDX, state.vel.IDX) = dt_identity;
  cov_dx.template block<3, 3>(state.vel.IDX, state.rot.IDX) =
      -dt_rot * Sophus::SO3d::hat(input.acc - state.ba);
  cov_dx.template block<3, 3>(state.vel.IDX, state.ba.IDX) = -dt_rot;

  Eigen::Matrix<Scalar, 2, 1> vec = Eigen::Matrix<Scalar, 2, 1>::Zero();
  Eigen::Matrix<Scalar, 3, 2> grav_matrix;
  state.grav.S2_Mx(grav_matrix, vec);
  cov_dx.template block<3, 2>(state.vel.IDX, state.grav.IDX) = dt * grav_matrix;
  cov_dx.template block<3, 3>(state.rot.IDX, state.rot.IDX) = tmp_so3;
  cov_dx.template block<3, 3>(state.rot.IDX, state.bg.IDX) = -dt * tmp_so3;

  // add noise terms
  Eigen::Matrix<double, StateType::DOF, NoiseDof> cov_dw =
      Eigen::Matrix<double, StateType::DOF, NoiseDof>::Zero();
  // noise is : ng, na, nbg, nba
  cov_dw.template block<3, 3>(state.vel.IDX, 3) = dt_rot;
  cov_dw.template block<3, 3>(state.rot.IDX, 0) = dt * tmp_so3;
  cov_dw.template block<3, 3>(state.bg.IDX, 6) = dt_identity;
  cov_dw.template block<3, 3>(state.ba.IDX, 9) = dt_identity;

  *covariance =
      cov_dx * (*covariance) * cov_dx.transpose() + cov_dw * noise_covariance * cov_dw.transpose();
}

template <typename StateType, typename UpdateInput, int NoiseDof,
          typename Scalar = typename StateType::scalar>
void HighOrderImuCovarianceUpdate(
    const Scalar& dt, const StateType& state, const UpdateInput& input,
    const Eigen::Matrix<Scalar, NoiseDof, NoiseDof>& noise_covariance,
    Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF>* covariance) {
  MTK::vect<3, Scalar> gyr_un = dt * (input.gyro - state.bg);
  MTK::vect<3, Scalar> acc_un = input.acc - state.ba;
  Eigen::Matrix<Scalar, 3, 3> rot_mat = state.rot.toRotationMatrix();
  Eigen::Matrix<Scalar, 3, 3> tmp_so3 = MTK::A_matrix(gyr_un);

  // add state update terms
  Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF> cov_base =
      Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF>::Identity();
  Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF> cov_dx =
      Eigen::Matrix<Scalar, StateType::DOF, StateType::DOF>::Zero();

  {
    cov_dx.template block<3, 3>(state.pos.IDX, state.vel.IDX) =
        Eigen::Matrix<Scalar, 3, 3>::Identity();
    cov_dx.template block<3, 3>(state.vel.IDX, state.rot.IDX) =
        -rot_mat * Sophus::SO3<Scalar>::hat(acc_un);
    cov_dx.template block<3, 3>(state.vel.IDX, state.ba.IDX) = -rot_mat;

    Eigen::Matrix<Scalar, 2, 1> vec = Eigen::Matrix<Scalar, 2, 1>::Zero();
    Eigen::Matrix<Scalar, 3, 2> grav_matrix;
    state.grav.S2_Mx(grav_matrix, vec);
    cov_dx.template block<3, 2>(state.vel.IDX, state.grav.IDX) = grav_matrix;
    cov_dx.template block<3, 3>(state.rot.IDX, state.bg.IDX) = -tmp_so3;

    cov_base.template block<3, 3>(state.rot.IDX, state.rot.IDX) = tmp_so3;
  }

  {
    // cov_base += dt * cov_dx;
    auto cov_dx_dt = dt * cov_dx;
    auto cov_dx_dt_square = cov_dx_dt * cov_dx_dt;
    auto cov_dx_dt_cube = cov_dx_dt_square * cov_dx_dt;
    cov_base += cov_dx_dt + 0.5 * cov_dx_dt_square + (1.0 / 6.0) * cov_dx_dt_cube;
  }

  // add noise terms
  Eigen::Matrix<double, StateType::DOF, NoiseDof> cov_dw =
      Eigen::Matrix<double, StateType::DOF, NoiseDof>::Zero();
  {
    // noise is : ng, na, nbg, nba
    cov_dw.template block<3, 3>(state.vel.IDX, 3) = rot_mat;
    cov_dw.template block<3, 3>(state.rot.IDX, 0) = tmp_so3;
    cov_dw.template block<3, 3>(state.bg.IDX, 6) = Eigen::Matrix<Scalar, 3, 3>::Identity();
    cov_dw.template block<3, 3>(state.ba.IDX, 9) = Eigen::Matrix<Scalar, 3, 3>::Identity();
  }

  *covariance = cov_base * ((*covariance) + dt * cov_dw * noise_covariance * cov_dw.transpose()) *
                cov_base.transpose();
}

}  // namespace esekfom

#endif  // IKFOM_TOOLKIT_ESEKFOM_UTIL_HPP_
