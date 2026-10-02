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

#ifndef LIE_INTEGRATOR__TERRAIN_HPP_
#define LIE_INTEGRATOR__TERRAIN_HPP_

#include <Eigen/Dense>

namespace lie_integrator
{

/// Mock sinusoidal height map (stand-in for grid_map / mesh sampling).
double getTerrainHeight(double x, double y);

/// Surface normal at (x, y) via central finite differences.
Eigen::Vector3d getTerrainNormal(double x, double y);

}  // namespace lie_integrator

#endif  // LIE_INTEGRATOR__TERRAIN_HPP_
