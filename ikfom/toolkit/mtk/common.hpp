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
 * @file mtk/startIdx.hpp 
 * @brief Tools to access sub-elements of compound manifolds.
 */

// Modified 2022-2026 by MapMind from IKFoM (https://github.com/hku-mars/IKFoM).

#ifndef IKFOM_TOOLKIT_MTK_COMMON_HPP_
#define IKFOM_TOOLKIT_MTK_COMMON_HPP_

#include <Eigen/Core>

#include "ikfom/toolkit/mtk/src/mtkmath.hpp"
#include "ikfom/toolkit/mtk/src/vectview.hpp"

namespace MTK {

template <class Base, class T, int idx, int dim>
void setDiagonal(Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>& cov,  // NOLINT
                 MTK::SubManifold<T, idx, dim> Base::*, const typename Base::scalar& val) {
  cov.diagonal().template segment<T::DOF>(idx).setConstant(val);
}

template <class Base, class T, int idx, int dim>
void setDiagonal(Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>& cov,  // NOLINT
                 MTK::SubManifold<T, idx, dim> Base::*,
                 const Eigen::Matrix<typename Base::scalar, T::DOF, 1>& vec) {
  cov.diagonal().template segment<T::DOF>(idx) = vec;
}

template <typename Base>
class get_cov {
 public:
  typedef Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> type;
  typedef const Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> const_type;
};

template <class Scalar, int BaseDIM, class Base, class T, int idx, int dim>
vectview<Scalar, T::DIM> subvector_(vectview<Scalar, BaseDIM> vec,
                                    SubManifold<T, idx, dim> Base::*) {
  return vec.template segment<T::DIM>(dim);
}

template <class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
vectview<Scalar, T::DOF> subvector(vectview<Scalar, BaseDOF>& cvec,  // NOLINT
                                   SubManifold<T, idx, dim> Base::*) {
  return cvec.template segment<T::DOF>(idx);
}

template <class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
vectview<const Scalar, T::DOF> subvector(const vectview<const Scalar, BaseDOF> cvec,
                                         SubManifold<T, idx, dim> Base::*) {
  return cvec.template segment<T::DOF>(idx);
}

}  // namespace MTK

#endif  // IKFOM_TOOLKIT_MTK_COMMON_HPP_
