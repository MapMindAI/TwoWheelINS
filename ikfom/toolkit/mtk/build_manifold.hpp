// This is an advanced implementation of the algorithm described in the
// following paper:
//    C. Hertzberg,  R.  Wagner,  U.  Frese,  and  L.  Schroder.  Integratinggeneric   sensor   fusion   algorithms   with   sound   state   representationsthrough  encapsulation  of  manifolds.
//    CoRR,  vol.  abs/1107.1119,  2011.[Online]. Available: http://arxiv.org/abs/1107.1119

/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
 *  All rights reserved.
 *
 *  Modifier: Dongjiao HE <hdj65822@connect.hku.hk>
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
 
/*
 *  Copyright (c) 2008--2011, Universitaet Bremen
 *  All rights reserved.
 *
 *  Author: Christoph Hertzberg <chtz@informatik.uni-bremen.de>
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
/**
 * @file mtk/build_manifold.hpp
 * @brief Macro to automatically construct compound manifolds.
 * 
 */

// Modified 2022-2026 by MapMind from IKFoM (https://github.com/hku-mars/IKFoM).

#ifndef IKFOM_TOOLKIT_MTK_BUILD_MANIFOLD_HPP_
#define IKFOM_TOOLKIT_MTK_BUILD_MANIFOLD_HPP_

#include <Eigen/Core>
#include <vector>

#include "ikfom/toolkit/mtk/common.hpp"
#include "ikfom/toolkit/mtk/macros_utils.h"

// //////// internals //////

#define IKFOM_MTK_S2_hat(variable) \
  if (variable.IDX == idx) {       \
    variable.S2_hat(res);          \
  }
#define IKFOM_MTK_S2_Nx_yy(variable) \
  if (variable.IDX == idx) {         \
    variable.S2_Nx_yy(res);          \
  }
#define IKFOM_MTK_S2_Mx(variable) \
  if (variable.IDX == idx) {      \
    variable.S2_Mx(res, dx);      \
  }
#define IKFOM_MTK_S2_Bx(variable) \
  if (variable.IDX == idx) {      \
    variable.S2_Bx(res);          \
  }

#define IKFOM_MTK_ISTREAM(variable) >> __var.variable

#define IKFOM_MTK_S2_state(variable)                                \
  if (variable.TYP == 1) {                                          \
    S2_state.push_back(std::make_pair(variable.IDX, variable.DIM)); \
  }
#define IKFOM_MTK_SO3_state(variable)                                  \
  if (variable.TYP == 2) {                                             \
    (SO3_state).push_back(std::make_pair(variable.IDX, variable.DIM)); \
  }
#define IKFOM_MTK_vect3_state(variable)                                                    \
  if (variable.TYP == 0) {                                                                 \
    (vect_state).push_back(std::make_pair(std::make_pair(variable.IDX, variable.DIM), 3)); \
  }

#define IKFOM_MTK_BOXPLUS(variable) \
  variable.boxplus(MTK::subvector(vector, &self::variable), scale);
#define IKFOM_MTK_OPLUS(variable) variable.oplus(MTK::subvector_(vector, &self::variable), scale);
#define IKFOM_MTK_BOXMINUS(variable) \
  variable.boxminus(MTK::subvector(res, &self::variable), object.variable);

#define IKFOM_MTK_FUNCTIONS(...)                                                                \
  void S2_hat(Eigen::Matrix<double, 3, 3>& res, int idx) { MAP(IKFOM_MTK_S2_hat, __VA_ARGS__) } \
  void S2_Nx_yy(Eigen::Matrix<double, 2, 3>& res, int idx) const {                              \
    MAP(IKFOM_MTK_S2_Nx_yy, __VA_ARGS__)                                                        \
  }                                                                                             \
  void S2_Mx(Eigen::Matrix<double, 3, 2>& res, Eigen::Matrix<double, 2, 1> dx, int idx) const { \
    MAP(IKFOM_MTK_S2_Mx, __VA_ARGS__)                                                           \
  }                                                                                             \
  void S2_Bx(Eigen::Matrix<double, 3, 2>& res, int idx) { MAP(IKFOM_MTK_S2_Bx, __VA_ARGS__) }   \
  void build_S2_state() { MAP(IKFOM_MTK_S2_state, __VA_ARGS__) }                                \
  void build_SO3_state() { MAP(IKFOM_MTK_SO3_state, __VA_ARGS__) }                              \
  void build_vect_state() { MAP(IKFOM_MTK_vect3_state, __VA_ARGS__) }                           \
  void boxplus(const MTK::vectview<const double, DOF>& vector, double scale = 1) {              \
    MAP(IKFOM_MTK_BOXPLUS, __VA_ARGS__)                                                         \
  }                                                                                             \
  void oplus(const MTK::vectview<const double, DIM>& vector, double scale = 1) {                \
    MAP(IKFOM_MTK_OPLUS, __VA_ARGS__)                                                           \
  }                                                                                             \
  void boxminus(MTK::vectview<double, DOF> res, const self& object) const {                     \
    MAP(IKFOM_MTK_BOXMINUS, __VA_ARGS__)                                                        \
  }                                                                                             \
  friend std::istream& operator>>(std::istream& __is, self& __var) {                            \
    return __is MAP(IKFOM_MTK_ISTREAM, __VA_ARGS__);                                            \
  }

// idx is for the beginning index (of the parameter DOF length)
// dim is for the beginning index (of the parameter DIM length)
#define IKFOM_DECLARE_VARIABLE(type, var, idx, dim) MTK::SubManifold<type, idx, dim> var;

// make the define function

#define IKFOM_INIT_DECLARE_VARIABLE(type, var, idx, dim) const type& var = type()
#define IKFOM_INIT_SET_VARIABLE(type, var, idx, dim) var(var)

#define IKFOM_MTK_DECLATION(manifold_name, ...)                          \
  std::vector<std::pair<int, int>> S2_state;                             \
  std::vector<std::pair<int, int>> SO3_state;                            \
  std::vector<std::pair<std::pair<int, int>, int>> vect_state;           \
  typedef manifold_name self;                                            \
  typedef double scalar;                                                 \
  MAP_FOUR(IKFOM_DECLARE_VARIABLE, __VA_ARGS__)                          \
  manifold_name(MAP_FOUR_LIST(IKFOM_INIT_DECLARE_VARIABLE, __VA_ARGS__)) \
      : MAP_FOUR_LIST(IKFOM_INIT_SET_VARIABLE, __VA_ARGS__) {}

#endif  // IKFOM_TOOLKIT_MTK_BUILD_MANIFOLD_HPP_
