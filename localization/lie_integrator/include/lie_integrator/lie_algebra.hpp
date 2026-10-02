// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIE_INTEGRATOR__LIE_ALGEBRA_HPP_
#define LIE_INTEGRATOR__LIE_ALGEBRA_HPP_

#include <Eigen/Dense>

namespace lie_integrator
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Skew-symmetric (cross-product) matrix from a 3-vector.
Eigen::Matrix3d skew(const Eigen::Vector3d & v);

/// Exponential map SO(3): Rodrigues' rotation formula.
Eigen::Matrix3d expSO3(const Eigen::Vector3d & omega);

/// Exponential map SE(3) for twist ξ = [v; ω] ∈ se(3).
Eigen::Matrix4d expSE3(const Vector6d & twist);

}  // namespace lie_integrator

#endif  // LIE_INTEGRATOR__LIE_ALGEBRA_HPP_
