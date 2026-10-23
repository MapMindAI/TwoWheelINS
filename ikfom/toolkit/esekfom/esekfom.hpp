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

#ifndef IKFOM_TOOLKIT_ESEKFOM_ESEKFOM_HPP_
#define IKFOM_TOOLKIT_ESEKFOM_ESEKFOM_HPP_

#include "glog/logging.h"

#include <cstdlib>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>

#include "ikfom/toolkit/esekfom/util.hpp"
#include "ikfom/toolkit/mtk/build_manifold.hpp"
#include "ikfom/toolkit/mtk/types/S2.hpp"
#include "ikfom/toolkit/mtk/types/SOn.hpp"
#include "ikfom/toolkit/mtk/types/vect.hpp"

namespace esekfom {

// used for iterated error state EKF update
// for the aim to calculate  measurement (z), estimate measurement (h), partial differention
// matrices (h_x, h_v) and the noise covariance (R) at the same time, by only one function. applied
// for measurement as an Eigen matrix whose dimension is changing
template <typename T>
struct dyn_share_datastruct {
  bool valid;
  bool converge;
  Eigen::Matrix<T, Eigen::Dynamic, 1> z;
  Eigen::Matrix<T, Eigen::Dynamic, 1> h;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_v;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_x;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> R;
};

template <typename state, int NoiseDof, typename UpdateInput = state, typename measurement = state,
          int measurement_noise_dof = 0>
class esekf {
  typedef esekf self;
  enum { n = state::DOF, m = state::DIM, l = measurement::DOF };

 public:
  typedef typename state::scalar Scalar;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorXs;
  typedef Eigen::Matrix<Scalar, n, n> cov;
  typedef Eigen::Matrix<Scalar, m, n> cov_;
  typedef Eigen::Matrix<Scalar, n, 1> vectorized_state;
  typedef Eigen::Matrix<Scalar, m, 1> flatted_state;
  typedef flatted_state processModel(state&, const UpdateInput&);
  typedef Eigen::Matrix<Scalar, m, n> processMatrix1(state&, const UpdateInput&);
  typedef Eigen::Matrix<Scalar, m, NoiseDof> processMatrix2(state&, const UpdateInput&);
  typedef Eigen::Matrix<Scalar, NoiseDof, NoiseDof> UpdateNoiseCovariance;
  typedef measurement measurementModel(state&, bool&);
  typedef VectorXs measurementModel_dyn(state&, bool&);
  typedef VectorXs measurement_residual_dyn(const VectorXs&, const VectorXs&);

  // use boost to allow member function from other class
  typedef std::function<void(const state&, dyn_share_datastruct<Scalar>*)>
      measurementModel_dyn_share;
  typedef Eigen::Matrix<Scalar, l, n> measurementMatrix1(state&, bool&);
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, n> measurementMatrix1_dyn(const VectorXs&,
                                                                          const VectorXs&,
                                                                          const state&, bool);
  typedef Eigen::Matrix<Scalar, l, measurement_noise_dof> measurementMatrix2(state&, bool&);
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> measurementMatrix2_dyn(state&,
                                                                                       bool&);
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> measurementnoisecovariance_dyn;

  explicit esekf(const state& x = state(), const cov& P = cov::Identity()) : x_(x), P_(P) {}

  static VectorXs MeasurementResidual(const VectorXs& obs, const VectorXs& raw) {
    return obs - raw;
  }

  size_t h_dyn_vec_size() { return h_dyn_vec_.size(); }

  // receive system-specific models and their differentions.
  // for measurement as an Eigen matrix whose dimention is chaing.
  void init_dyn(processModel f_in, processMatrix1 f_x_in, processMatrix2 f_w_in,
                int maximum_iteration, Scalar limit_vector[n]) {
    f = f_in;
    f_x = f_x_in;
    f_w = f_w_in;

    maximum_iter = maximum_iteration;
    for (int i = 0; i < n; i++) {
      limit[i] = limit_vector[i];
    }
    BuildState();
  }

  void add_dyn_vec(measurementModel_dyn h_in, measurementMatrix1_dyn h_x_in,
                   measurementMatrix2_dyn h_v_in,
                   measurement_residual_dyn h_r_in = MeasurementResidual) {
    h_dyn_vec_.emplace_back(h_in);
    h_x_dyn_vec_.emplace_back(h_x_in);
    h_v_dyn_vec_.emplace_back(h_v_in);
    h_r_dyn_vec_.emplace_back(h_r_in);
  }

  // receive system-specific models and their differentions
  // for measurement as an Eigen matrix whose dimension is changing.
  // calculate  measurement (z), estimate measurement (h), partial differention matrices (h_x, h_v)
  // and the noise covariance (R) at the same time, by only one function (h_dyn_share_in).
  void init_dyn_share(processModel f_in, processMatrix1 f_x_in, processMatrix2 f_w_in,
                      measurementModel_dyn_share h_dyn_share_in, int maximum_iteration,
                      Scalar limit_vector[n]) {
    f = f_in;
    f_x = f_x_in;
    f_w = f_w_in;
    h_dyn_share = h_dyn_share_in;

    maximum_iter = maximum_iteration;
    for (int i = 0; i < n; i++) {
      limit[i] = limit_vector[i];
    }
    BuildState();
  }

  // iterated error state EKF propogation
  void predict(const double& dt, const UpdateNoiseCovariance& Q, const UpdateInput& i_in) {
    flatted_state f_ = f(x_, i_in);
    cov_ f_x_ = f_x(x_, i_in);

    Eigen::Matrix<Scalar, m, NoiseDof> f_w_ = f_w(x_, i_in);
    cov f_x_final;
    Eigen::Matrix<Scalar, n, NoiseDof> f_w_final;
    state x_before = x_;
    x_.oplus(f_, dt);

    cov F_x1 = cov::Identity();

    // fill the jacobian matrices
    for (const auto& id_pair : x_.vect_state) {
      int idx = id_pair.first.first;
      int dim = id_pair.first.second;
      int dof = id_pair.second;
      f_x_final.block(idx, 0, dof, n) = f_x_.block(dim, 0, dof, n);
      f_w_final.block(idx, 0, dof, NoiseDof) = f_w_.block(dim, 0, dof, NoiseDof);
    }

    // modify the rotation transition matrix
    Eigen::Matrix<Scalar, 3, 3> res_temp_SO3;
    for (const auto& id_pair : x_.SO3_state) {
      int idx = id_pair.first;
      int dim = id_pair.second;
      MTK::vect<3, Scalar> seg_SO3 = -dt * f_.block(dim, 0, 3, 1);
      MTK::SO3<Scalar> res;
      res.w() = MTK::exp<Scalar, 3>(res.vec(), seg_SO3, Scalar(0.5));
      F_x1.template block<3, 3>(idx, idx) = res.toRotationMatrix();
      res_temp_SO3 = MTK::A_matrix(seg_SO3);
      f_x_final.template block<3, n>(idx, 0) = res_temp_SO3 * (f_x_.template block<3, n>(dim, 0));
      f_w_final.template block<3, NoiseDof>(idx, 0) =
          res_temp_SO3 * (f_w_.template block<3, NoiseDof>(dim, 0));
    }

    // modify the S2 transition matrix
    Eigen::Matrix<Scalar, 2, 3> res_temp_S2;
    for (const auto& id_pair : x_.S2_state) {
      int idx = id_pair.first;
      int dim = id_pair.second;
      MTK::vect<3, Scalar> seg_S2 = dt * f_.block(dim, 0, 3, 1);
      MTK::vect<2, Scalar> vec = MTK::vect<2, Scalar>::Zero();
      MTK::SO3<Scalar> res;
      res.w() = MTK::exp<Scalar, 3>(res.vec(), seg_S2, Scalar(1 / 2));  //
      Eigen::Matrix<Scalar, 2, 3> Nx;
      Eigen::Matrix<Scalar, 3, 2> Mx;
      x_.S2_Nx_yy(Nx, idx);
      x_before.S2_Mx(Mx, vec, idx);
      F_x1.template block<2, 2>(idx, idx) = Nx * res.toRotationMatrix() * Mx;

      Eigen::Matrix<Scalar, 3, 3> x_before_hat;
      x_before.S2_hat(x_before_hat, idx);
      res_temp_S2 = -Nx * res.toRotationMatrix() * x_before_hat * MTK::A_matrix(seg_S2).transpose();

      f_x_final.template block<2, n>(idx, 0) = res_temp_S2 * (f_x_.template block<3, n>(dim, 0));
      f_w_final.template block<2, NoiseDof>(idx, 0) =
          res_temp_S2 * (f_w_.template block<3, NoiseDof>(dim, 0));
    }

    // apply kalman update
    F_x1 += f_x_final * dt;
    auto f_w_final_dt = dt * f_w_final;
    P_ = F_x1 * P_ * F_x1.transpose() + f_w_final_dt * Q * f_w_final_dt.transpose();
  }

  enum CovUpdateMethod { COV_LINEAR, COV_HIGH_ORDER };
  enum ImuUpdateMethod { LINEAR, MID_POINT, RUNGE_KUTTA };
  void PredictImu(const double& dt, const UpdateNoiseCovariance& noise_covariance,
                  const UpdateInput& input, CovUpdateMethod method_cov = COV_LINEAR,
                  ImuUpdateMethod method_model = LINEAR) {
    switch (method_cov) {
      case COV_LINEAR: {
        ImuCovarianceUpdate(dt, x_, input, noise_covariance, &P_);
      } break;
      case COV_HIGH_ORDER: {
        HighOrderImuCovarianceUpdate(dt, x_, input, noise_covariance, &P_);
      } break;
      default: { LOG(ERROR) << "unknow update method : " << method_cov; }
    }

    switch (method_model) {
      case LINEAR: {
        LinearImuUpdate(dt, input, &x_);
      } break;
      case MID_POINT: {
        static UpdateInput last_input = input;
        // euler mid point method
        MidPointImuUpdate(dt, last_input, input, &x_);
        last_input = input;
      } break;
      case RUNGE_KUTTA: {
        RungeKuttaImuUpdate(dt, input.gyro, input.acc, &x_);
      } break;
      default: { LOG(ERROR) << "unknow update method : " << method_model; }
    }
  }

  // iterated error state EKF update for measurement as an Eigen matrix whose dimension is changing.
  bool update_iterated_dyn(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& z,
                           const measurementnoisecovariance_dyn& R, size_t idx_fcn = 0) {
    CHECK_LT(idx_fcn, h_dyn_vec_.size());
    int t = 0;
    bool valid = true;
    bool converg = true;
    state x_propagated = x_;
    cov P_propagated = P_;
    int dof_Measurement;

    for (int i = -1; i < maximum_iter; i++) {
      valid = true;
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1> h_ = (h_dyn_vec_[idx_fcn])(x_, valid);
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1> residual = h_r_dyn_vec_[idx_fcn](z, h_);
      Eigen::Matrix<Scalar, Eigen::Dynamic, n> h_x_ =
          (h_x_dyn_vec_[idx_fcn])(residual, z, x_, valid);
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> h_v_ =
          (h_v_dyn_vec_[idx_fcn])(x_, valid);
      dof_Measurement = residual.rows();
      vectorized_state dx, dx_new;
      x_.boxminus(dx, x_propagated);
      dx_new = dx;
      if (!valid) {
        continue;
      }

      P_ = P_propagated;
      UpdateCovarianceTransitionSO3(x_, dx, &P_, &dx_new);
      UpdateCovarianceTransitionS2(x_, x_propagated, dx, &P_, &dx_new);

      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> K_;
      if (n > dof_Measurement) {
        K_ = P_ * h_x_.transpose() *
             (h_x_ * P_ * h_x_.transpose() + h_v_ * R * h_v_.transpose()).inverse();
      } else {
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> R_in =
            (h_v_ * R * h_v_.transpose()).inverse();
        K_ = (h_x_.transpose() * R_in * h_x_ + P_.inverse()).inverse() * h_x_.transpose() * R_in;
      }
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> K_x = K_ * h_x_;

      Eigen::Matrix<Scalar, n, 1> dx_ =
          K_ * residual + (K_x - Eigen::Matrix<Scalar, n, n>::Identity()) * dx_new;
      // state x_before = x_;
      x_.boxplus(dx_);
      converg = true;
      for (int i = 0; i < n; i++) {
        if (std::fabs(dx_[i]) > limit[i]) {
          converg = false;
          break;
        }
      }
      if (converg) t++;
      // if have enough convergence -> update the state and quit
      if (t > 1) {
        L_ = P_;
        // std::cout << "iteration time:" << t << "," << i << std::endl;

        if (n > dof_Measurement) {
          UpdateFilterSO3(x_, dx_, dof_Measurement, &P_, &L_, &K_);
          UpdateFilterS2(x_, x_propagated, dx_, dof_Measurement, &P_, &L_, &K_);
          P_ = L_ - K_ * h_x_ * P_;
        } else {
          UpdateFilterSO3(x_, dx_, static_cast<int>(n), &P_, &L_, &K_x);
          UpdateFilterS2(x_, x_propagated, dx_, static_cast<int>(n), &P_, &L_, &K_x);
          P_ = L_ - K_x * P_;
        }

        return true;
      }
    }
    return false;
  }

  // iterated error state EKF update for measurement as an Eigen matrix whose dimension is changing.
  // calculate measurement (z), estimate measurement (h), partial differention matrices (h_x, h_v)
  // and the noise covariance (R) at the same time, by only one function.
  void update_iterated_dyn_share() {
    int t = 0;
    dyn_share_datastruct<Scalar> dyn_share;
    dyn_share.valid = true;
    dyn_share.converge = true;
    state x_propagated = x_;
    cov P_propagated = P_;
    int dof_Measurement;
    // int dof_Measurement_noise;
    for (int i = -1; i < maximum_iter; i++) {
      dyn_share.valid = true;
      h_dyn_share(x_, &dyn_share);
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1> z = dyn_share.z;
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1> h = dyn_share.h;
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> R = dyn_share.R;
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> h_x = dyn_share.h_x;
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> h_v = dyn_share.h_v;
      dof_Measurement = h_x.rows();
      // dof_Measurement_noise = dyn_share.R.rows();
      vectorized_state dx, dx_new;
      x_.boxminus(dx, x_propagated);
      dx_new = dx;
      if (!(dyn_share.valid)) {
        continue;
      }

      P_ = P_propagated;
      UpdateCovarianceTransitionSO3(x_, dx, &P_, &dx_new);
      UpdateCovarianceTransitionS2(x_, x_propagated, dx, &P_, &dx_new);

      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> K_;
      if (n > dof_Measurement) {
        K_ = P_ * h_x.transpose() *
             (h_x * P_ * h_x.transpose() + h_v * R * h_v.transpose()).inverse();
      } else {
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> R_in =
            (h_v * R * h_v.transpose()).inverse();
        K_ = (h_x.transpose() * R_in * h_x + P_.inverse()).inverse() * h_x.transpose() * R_in;
      }

      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> K_x = K_ * h_x;
      Eigen::Matrix<Scalar, n, 1> dx_ =
          K_ * (z - h) + (K_x - Eigen::Matrix<Scalar, n, n>::Identity()) * dx_new;
      // state x_before = x_;
      x_.boxplus(dx_);
      dyn_share.converge = true;
      for (int i = 0; i < n; i++) {
        if (std::fabs(dx_[i]) > limit[i]) {
          dyn_share.converge = false;
          break;
        }
      }
      if (dyn_share.converge) t++;
      if (t > 1 || i == maximum_iter - 1) {
        L_ = P_;
        // std::cout << "iteration time:" << t << "," << i << std::endl;

        if (n > dof_Measurement) {
          UpdateFilterSO3(x_, dx_, dof_Measurement, &P_, &L_, &K_);
          UpdateFilterS2(x_, x_propagated, dx_, dof_Measurement, &P_, &L_, &K_);
          P_ = L_ - K_ * h_x * P_;
        } else {
          UpdateFilterSO3(x_, dx_, static_cast<int>(n), &P_, &L_, &K_x);
          UpdateFilterS2(x_, x_propagated, dx_, static_cast<int>(n), &P_, &L_, &K_x);
          P_ = L_ - K_x * P_;
        }
        return;
      }
    }
  }

  // iterated error state EKF update modified for one specific system with extrinsic online
  // calibration. for lidar-imu system
  // ParamDim will be 12 if optimize extrinsic
  //                   6 if not optimize extrinsic
  template <int ParamDim = 12>
  void update_iterated_dyn_share_modified_extrinsic(double R, bool verbose = false) {
    dyn_share_datastruct<Scalar> dyn_share;
    dyn_share.valid = true;
    dyn_share.converge = true;
    int t = 0;
    state x_propagated = x_;
    cov P_propagated = P_;
    double inv_R = 1.0 / R;

    Eigen::Matrix<Scalar, n, 1> K_h;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> K_x;

    vectorized_state dx_new = vectorized_state::Zero();
    for (int i = -1; i < maximum_iter; i++) {
      dyn_share.valid = true;
      h_dyn_share(x_, &dyn_share);
      const Eigen::Matrix<Scalar, Eigen::Dynamic, ParamDim>& h_x_ = dyn_share.h_x;
      int dof_Measurement = h_x_.rows();
      vectorized_state dx;
      x_.boxminus(dx, x_propagated);
      dx_new = dx;

      if (!dyn_share.valid) {
        continue;
      }

      P_ = P_propagated;
      UpdateCovarianceTransitionSO3(x_, dx, &P_, &dx_new);
      UpdateCovarianceTransitionS2(x_, x_propagated, dx, &P_, &dx_new);

      if (n > dof_Measurement) {
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> h_x_cur =
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(dof_Measurement, n);
        h_x_cur.topLeftCorner(dof_Measurement, ParamDim) = h_x_;

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> K_ =
            inv_R * P_ * h_x_cur.transpose() *
            (inv_R * h_x_cur * P_ * h_x_cur.transpose() +
             Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(dof_Measurement,
                                                                             dof_Measurement))
                .inverse();
        K_h = K_ * (-dyn_share.h);
        K_x = K_ * h_x_cur;

      } else {
        cov P_temp = (inv_R * P_).inverse();
        Eigen::Matrix<Scalar, ParamDim, ParamDim> HTH = h_x_.transpose() * h_x_;
        P_temp.template block<ParamDim, ParamDim>(0, 0) += HTH;

        cov P_inv = P_temp.inverse();
        K_h = P_inv.template block<n, ParamDim>(0, 0) * h_x_.transpose() * (-dyn_share.h);
        K_x = Eigen::Matrix<Scalar, n, n>::Zero();
        K_x.template block<n, ParamDim>(0, 0) = P_inv.template block<n, ParamDim>(0, 0) * HTH;
      }

      Eigen::Matrix<Scalar, n, 1> dx_ =
          K_h + (K_x - Eigen::Matrix<Scalar, n, n>::Identity()) * dx_new;

      if (verbose) {
        std::cout << "iteration time:" << t << "|" << i << std::endl;
        std::cout << dx_.transpose() << std::endl;
      }

      state x_before = x_;
      x_.boxplus(dx_);
      dyn_share.converge = true;
      for (int i = 0; i < n; i++) {
        if (std::fabs(dx_[i]) > limit[i]) {
          dyn_share.converge = false;
          break;
        }
      }
      if (dyn_share.converge) t++;

      if (!t && i == maximum_iter - 2) {
        dyn_share.converge = true;
      }

      if (t > 1 || i == maximum_iter - 1) {
        L_ = P_;

        UpdateFilterSO3(x_, dx_, ParamDim, &P_, &L_, &K_x);
        UpdateFilterS2(x_, x_propagated, dx_, ParamDim, &P_, &L_, &K_x);
        P_ = L_ - K_x.template block<n, ParamDim>(0, 0) * P_.template block<ParamDim, n>(0, 0);

        return;
      }
    }
  }

  const state& get_x() const { return x_; }
  const cov& get_P() const { return P_; }

  state& mutable_state() { return x_; }
  cov& mutable_covariance() { return P_; }

  void BuildState() {
    x_.build_S2_state();
    x_.build_SO3_state();
    x_.build_vect_state();
  }

 private:
  state x_;
  cov P_;
  cov L_ = cov::Identity();

  processModel* f;
  processMatrix1* f_x;
  processMatrix2* f_w;

  std::vector<measurement_residual_dyn*> h_r_dyn_vec_;
  std::vector<measurementModel_dyn*> h_dyn_vec_;
  std::vector<measurementMatrix1_dyn*> h_x_dyn_vec_;
  std::vector<measurementMatrix2_dyn*> h_v_dyn_vec_;

  measurementModel_dyn_share h_dyn_share;

  int maximum_iter = 0;
  Scalar limit[n];  // NOLINT

  template <typename T>
  T check_safe_update(T _temp_vec) {
    T temp_vec = _temp_vec;
    if (std::isnan(temp_vec(0, 0))) {
      temp_vec.setZero();
      return temp_vec;
    }
    double angular_dis = temp_vec.block(0, 0, 3, 1).norm() * 57.3;
    double pos_dis = temp_vec.block(3, 0, 3, 1).norm();
    if (angular_dis >= 20 || pos_dis > 1) {
      printf("Angular dis = %.2f, pos dis = %.2f\r\n", angular_dis, pos_dis);
      temp_vec.setZero();
    }
    return temp_vec;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace esekfom

#endif  //  IKFOM_TOOLKIT_ESEKFOM_ESEKFOM_HPP_
