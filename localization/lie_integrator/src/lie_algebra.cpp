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

#include "lie_integrator/lie_algebra.hpp"

#include <cmath>

namespace lie_integrator
{

Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return m;
}

Eigen::Matrix3d expSO3(const Eigen::Vector3d & omega)
{
  const double theta = omega.norm();
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  if (theta < 1e-6) {
    return I + skew(omega);
  }
  const Eigen::Matrix3d w_hat = skew(omega) / theta;
  return I + std::sin(theta) * w_hat + (1.0 - std::cos(theta)) * w_hat * w_hat;
}

Eigen::Matrix4d expSE3(const Vector6d & twist)
{
  const Eigen::Vector3d v = twist.head<3>();
  const Eigen::Vector3d omega = twist.tail<3>();

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = expSO3(omega);

  const double theta = omega.norm();
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d V;

  if (theta < 1e-6) {
    V = I + 0.5 * skew(omega);
  } else {
    // w_hat is the unit skew; V maps body linear velocity to translation
    const Eigen::Matrix3d w_hat = skew(omega) / theta;
    V = I + ((1.0 - std::cos(theta)) / theta) * w_hat +
        ((theta - std::sin(theta)) / theta) * w_hat * w_hat;
  }

  T.block<3, 1>(0, 3) = V * v;
  return T;
}

}  // namespace lie_integrator
