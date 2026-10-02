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

#include "lie_integrator/terrain.hpp"

#include <cmath>

namespace lie_integrator
{

double getTerrainHeight(double x, double y)
{
  return 0.5 * std::sin(x * 0.5) * std::cos(y * 0.5);
}

Eigen::Vector3d getTerrainNormal(double x, double y)
{
  constexpr double eps = 0.01;
  const double dzdx =
    (getTerrainHeight(x + eps, y) - getTerrainHeight(x - eps, y)) / (2.0 * eps);
  const double dzdy =
    (getTerrainHeight(x, y + eps) - getTerrainHeight(x, y - eps)) / (2.0 * eps);

  // n = (-∂z/∂x, -∂z/∂y, 1)
  Eigen::Vector3d n(-dzdx, -dzdy, 1.0);
  return n.normalized();
}

}  // namespace lie_integrator
