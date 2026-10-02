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

#include "lie_integrator/lie_integrator_3d.hpp"
#include "lie_integrator/terrain.hpp"

#include <cstdio>
#include <iostream>

int main()
{
  // Wheelbase 2.8 m, dt 0.1 s
  lie_integrator::LieIntegrator3D sim(2.8, 0.1);

  std::cout << "Starting Simulation. Driving onto hilly terrain.\n";
  std::cout << "Format: Time [s], X, Y, Z, Terrain Normal (Nx, Ny, Nz)\n";

  double time = 0.0;
  for (int i = 0; i < 50; ++i) {
    const Eigen::Matrix4d pose = sim.step(10.0, 0.1);
    time += 0.1;

    const double x = pose(0, 3);
    const double y = pose(1, 3);
    const double z = pose(2, 3);
    const Eigen::Vector3d normal = pose.block<3, 1>(0, 2);

    std::printf(
      "T: %.1f, P: (%.2f, %.2f, %.2f), N: (%.2f, %.2f, %.2f)\n", time, x, y, z, normal(0),
      normal(1), normal(2));
  }

  return 0;
}
