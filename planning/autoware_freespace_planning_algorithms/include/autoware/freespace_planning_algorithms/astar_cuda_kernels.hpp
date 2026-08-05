// Copyright 2026 Autoware Foundation / contributors
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

#ifndef AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ASTAR_CUDA_KERNELS_HPP_
#define AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ASTAR_CUDA_KERNELS_HPP_

#include "autoware/freespace_planning_algorithms/astar_cuda_types.hpp"

#include <cstddef>

namespace autoware::freespace_planning_algorithms
{

struct AstarCudaDeviceBuffers
{
  uint8_t * d_is_obstacle{nullptr};
  GpuEDT * d_edt{nullptr};
  float * d_col_free_distance{nullptr};
  GpuIndexXY * d_coll_indexes{nullptr};  // flattened [theta * max_coll]
  int * d_coll_counts{nullptr};          // per theta
  GpuAstarNode * d_graph{nullptr};
  GpuSuccessor * d_successors{nullptr};
  size_t graph_size{0};
  size_t grid_size{0};
  size_t max_successors{0};
};

bool cuda_device_available();

void cuda_allocate_buffers(
  AstarCudaDeviceBuffers & buffers, size_t grid_size, size_t graph_size, size_t max_coll_total,
  size_t theta_size, size_t max_successors);

void cuda_free_buffers(AstarCudaDeviceBuffers & buffers);

void cuda_upload_map(
  AstarCudaDeviceBuffers & buffers, const uint8_t * is_obstacle, const GpuEDT * edt,
  size_t grid_size, const GpuIndexXY * coll_indexes, const int * coll_counts, size_t max_coll_total,
  size_t theta_size);

void cuda_upload_col_free_distance(
  AstarCudaDeviceBuffers & buffers, const float * col_free_distance, size_t grid_size);

void cuda_reset_graph(AstarCudaDeviceBuffers & buffers);

void cuda_expand_node(
  const AstarCudaDeviceBuffers & buffers, const GpuAstarParam & param,
  const GpuVehicleShape & vehicle, const GpuPose & goal_pose, const GpuAstarNode & current,
  int current_key, GpuSuccessor * host_successors_out, int & num_successors_out);

/// Expand up to `num_nodes` open nodes in one kernel launch (wavefront / beam).
/// `host_nodes` / `host_keys` length = num_nodes.
/// Writes up to num_nodes * max_successors_per_node into host_successors_out.
void cuda_expand_nodes_batch(
  const AstarCudaDeviceBuffers & buffers, const GpuAstarParam & param,
  const GpuVehicleShape & vehicle, const GpuPose & goal_pose, const GpuAstarNode * host_nodes,
  const int * host_keys, int num_nodes, GpuSuccessor * host_successors_out,
  int & num_successors_out);

void cuda_batch_reeds_shepp_distance(
  const GpuPose * host_starts, int n, const GpuPose & goal, float rho, float * host_distances_out);

void cuda_compute_edt(
  const uint8_t * host_is_obstacle, int width, int height, float resolution, GpuEDT * host_edt_out);

}  // namespace autoware::freespace_planning_algorithms

#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ASTAR_CUDA_KERNELS_HPP_
