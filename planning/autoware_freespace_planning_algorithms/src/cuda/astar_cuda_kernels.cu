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

#include "autoware/freespace_planning_algorithms/astar_cuda_kernels.hpp"
#include "autoware/freespace_planning_algorithms/reeds_shepp_device.cuh"

#include <cuda_runtime.h>

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::freespace_planning_algorithms
{
namespace
{

#define CUDA_CHECK(expr)                                                          \
  do {                                                                            \
    const cudaError_t err = (expr);                                               \
    if (err != cudaSuccess) {                                                     \
      throw std::runtime_error(std::string("CUDA error: ") + cudaGetErrorString(err)); \
    }                                                                             \
  } while (0)

constexpr float kEps = 0.001f;

__device__ __host__ inline float normalize_radian_cuda(float rad)
{
  constexpr float kPi = 3.14159265358979323846f;
  float value = fmodf(rad, 2.0f * kPi);
  if (value < 0.0f) {
    value += 2.0f * kPi;
  }
  if (value > kPi) {
    value -= 2.0f * kPi;
  }
  return value;
}

__device__ __host__ inline int discretize_angle_cuda(float theta, int theta_size)
{
  const float angle_resolution = 2.0f * 3.14159265358979323846f / static_cast<float>(theta_size);
  float n = normalize_radian_cuda(theta);
  if (n < 0.0f) {
    n += 2.0f * 3.14159265358979323846f;
  }
  int idx = static_cast<int>(roundf(n / angle_resolution)) % theta_size;
  if (idx < 0) {
    idx += theta_size;
  }
  return idx;
}

__device__ __host__ inline float get_turning_radius(float base_length, float steering_angle)
{
  return base_length / tanf(steering_angle);
}

__device__ __host__ inline GpuPose bicycle_get_pose(
  const GpuPose & current, float base_length, float steering_angle, float distance)
{
  GpuPose pose = current;
  const float yaw = current.yaw;
  if (fabsf(steering_angle) < kEps) {
    pose.x += distance * cosf(yaw);
    pose.y += distance * sinf(yaw);
    return pose;
  }
  const float R = get_turning_radius(base_length, steering_angle);
  const float beta = distance / R;
  pose.x += (R * sinf(yaw + beta) - R * sinf(yaw));
  pose.y += (R * cosf(yaw) - R * cosf(yaw + beta));
  pose.yaw = yaw + beta;
  return pose;
}

__device__ inline int index_to_id(int x, int y, int width)
{
  return y * width + x;
}

__device__ inline bool is_out_of_range(int x, int y, int width, int height)
{
  return x < 0 || y < 0 || x >= width || y >= height;
}

__device__ float get_vehicle_base_to_frame_distance(const GpuVehicleShape & v, float angle)
{
  const float normalized_angle = fabsf(normalize_radian_cuda(angle));
  const float w = 0.5f * v.width;
  const float l_b = v.base2back;
  const float l_f = v.length - l_b;
  constexpr float kPi2 = 1.5707963267948966f;
  if (normalized_angle < atanf(w / l_f)) {
    return l_f / cosf(normalized_angle);
  }
  if (normalized_angle < kPi2) {
    return w / sinf(normalized_angle);
  }
  if (normalized_angle < kPi2 + atanf(l_b / w)) {
    return w / cosf(normalized_angle - kPi2);
  }
  return l_b / cosf(3.14159265358979323846f - normalized_angle);
}

__device__ bool detect_collision_device(
  int base_x, int base_y, int theta, const GpuAstarParam & param, const GpuVehicleShape & vehicle,
  const uint8_t * is_obstacle, const GpuEDT * edt, const GpuIndexXY * coll_indexes,
  const int * coll_counts)
{
  if (is_out_of_range(base_x, base_y, param.width, param.height)) {
    return true;
  }
  const int id = index_to_id(base_x, base_y, param.width);
  const float obstacle_edt = edt[id].distance;
  if (obstacle_edt > vehicle.max_dimension) {
    return false;
  }
  if (obstacle_edt < vehicle.min_dimension) {
    return true;
  }
  const int count = coll_counts[theta];
  const GpuIndexXY * offsets = coll_indexes + theta * param.max_coll_indexes;
  for (int i = 0; i < count; ++i) {
    const int cx = base_x + offsets[i].x;
    const int cy = base_y + offsets[i].y;
    if (is_out_of_range(cx, cy, param.width, param.height)) {
      return true;
    }
    if (is_obstacle[index_to_id(cx, cy, param.width)]) {
      return true;
    }
  }
  return false;
}

__device__ void expand_one_candidate(
  GpuAstarParam param, GpuVehicleShape vehicle, GpuPose goal, GpuAstarNode current, int current_key,
  int cand_id, const uint8_t * is_obstacle, const GpuEDT * edt, const float * col_free_distance,
  const GpuIndexXY * coll_indexes, const int * coll_counts, GpuSuccessor & out)
{
  out = GpuSuccessor{};
  out.valid = 0;

  const int turn_span = 2 * param.turning_steps + 1;
  const bool is_back = cand_id >= turn_span;
  const int steering_index = (cand_id % turn_span) - param.turning_steps;

  if (
    current.parent_key >= 0 && is_back != static_cast<bool>(current.is_back) &&
    steering_index == current.steering_index) {
    return;
  }

  float exp_dist = param.min_expansion_dist;
  if (param.adapt_expansion_distance && param.max_expansion_dist > param.min_expansion_dist) {
    exp_dist = fminf(current.dist_to_goal * 0.15f, current.dist_to_obs * 0.3f);
    exp_dist = fminf(fmaxf(exp_dist, param.min_expansion_dist), param.max_expansion_dist);
  }
  const float direction =
    (is_back == static_cast<bool>(param.is_backward_search)) ? 1.0f : -1.0f;
  const float distance = exp_dist * direction;
  const float steering = static_cast<float>(steering_index) * param.steering_resolution;

  GpuPose current_pose{current.x, current.y, current.theta};
  const GpuPose next_pose =
    bicycle_get_pose(current_pose, vehicle.base_length, steering, distance);

  const int index_x = static_cast<int>(roundf(next_pose.x / param.resolution));
  const int index_y = static_cast<int>(roundf(next_pose.y / param.resolution));
  const int index_theta = discretize_angle_cuda(next_pose.yaw, param.theta_size);

  if (is_out_of_range(index_x, index_y, param.width, param.height)) {
    return;
  }
  const int grid_id = index_to_id(index_x, index_y, param.width);
  if (is_obstacle[grid_id]) {
    return;
  }
  if (detect_collision_device(
        index_x, index_y, index_theta, param, vehicle, is_obstacle, edt, coll_indexes,
        coll_counts)) {
    return;
  }

  const GpuEDT obs_edt = edt[grid_id];
  const bool is_direction_switch =
    (current.parent_key >= 0) && (is_back != static_cast<bool>(current.is_back));

  const float turning_steps_f = fmaxf(static_cast<float>(param.turning_steps), 1.0f);
  float total_weight = 1.0f;
  total_weight += param.curve_weight * (static_cast<float>(abs(steering_index)) / turning_steps_f);
  if (is_back) {
    total_weight *= (1.0f + param.reverse_weight);
  }

  float move_cost = current.gc + (total_weight * fabsf(distance));
  move_cost += param.smoothness_weight *
               static_cast<float>(abs(steering_index - current.steering_index)) /
               (2.0f * turning_steps_f);

  if (obs_edt.distance <= vehicle.max_dimension + param.cost_free_obs_dist) {
    const float yaw = index_theta * (2.0f * 3.14159265358979323846f / param.theta_size);
    const float base_to_frame = get_vehicle_base_to_frame_distance(vehicle, yaw - obs_edt.angle);
    const float vehicle_to_obs = fmaxf(obs_edt.distance - base_to_frame, 0.0f);
    move_cost += param.obstacle_distance_weight *
                 fmaxf(1.0f - (vehicle_to_obs / param.cost_free_obs_dist), 0.0f);
  }

  {
    const float dx = next_pose.x - goal.x;
    const float dy = next_pose.y - goal.y;
    const float dist_to_goal = sqrtf(dx * dx + dy * dy);
    if (!param.is_backward_search && dist_to_goal <= param.near_goal_distance) {
      const float c = cosf(goal.yaw);
      const float s = sinf(goal.yaw);
      const float lat = fabsf(-s * dx + c * dy);
      move_cost += param.goal_lat_distance_weight * lat;
    }
  }

  if (is_direction_switch) {
    move_cost +=
      param.direction_change_weight * (1.0f + (1.0f / (1.0f + current.dir_distance)));
  }

  float heuristic_metric = col_free_distance[grid_id];
  if (param.use_reeds_shepp) {
    reeds_shepp_device::StateXYT s0{next_pose.x, next_pose.y, next_pose.yaw};
    reeds_shepp_device::StateXYT s1{goal.x, goal.y, goal.yaw};
    const float rs = static_cast<float>(
      reeds_shepp_device::distance(s0, s1, param.avg_turning_radius));
    heuristic_metric = fmaxf(heuristic_metric, rs);
  }
  const float total_cost = move_cost + param.distance_heuristic_weight * heuristic_metric;

  const int key = grid_id * param.theta_size + index_theta;
  out.x = next_pose.x;
  out.y = next_pose.y;
  out.theta = next_pose.yaw;
  out.gc = move_cost;
  out.fc = total_cost;
  out.dir_distance = fabsf(distance) + (is_direction_switch ? 0.0f : current.dir_distance);
  {
    const float dx = next_pose.x - goal.x;
    const float dy = next_pose.y - goal.y;
    out.dist_to_goal = sqrtf(dx * dx + dy * dy);
  }
  out.dist_to_obs = obs_edt.distance;
  out.key = key;
  out.parent_key = current_key;
  out.steering_index = steering_index;
  out.is_back = is_back ? 1 : 0;
  out.valid = 1;
}

__global__ void expand_node_kernel(
  GpuAstarParam param, GpuVehicleShape vehicle, GpuPose goal, GpuAstarNode current, int current_key,
  const uint8_t * is_obstacle, const GpuEDT * edt, const float * col_free_distance,
  const GpuIndexXY * coll_indexes, const int * coll_counts, GpuSuccessor * successors,
  int num_candidates)
{
  const int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= num_candidates) {
    return;
  }
  expand_one_candidate(
    param, vehicle, goal, current, current_key, tid, is_obstacle, edt, col_free_distance,
    coll_indexes, coll_counts, successors[tid]);
}

__global__ void expand_nodes_batch_kernel(
  GpuAstarParam param, GpuVehicleShape vehicle, GpuPose goal, const GpuAstarNode * currents,
  const int * current_keys, int num_nodes, int cands_per_node, const uint8_t * is_obstacle,
  const GpuEDT * edt, const float * col_free_distance, const GpuIndexXY * coll_indexes,
  const int * coll_counts, GpuSuccessor * successors)
{
  const int tid = blockIdx.x * blockDim.x + threadIdx.x;
  const int total = num_nodes * cands_per_node;
  if (tid >= total) {
    return;
  }
  const int node_idx = tid / cands_per_node;
  const int cand_id = tid % cands_per_node;
  expand_one_candidate(
    param, vehicle, goal, currents[node_idx], current_keys[node_idx], cand_id, is_obstacle, edt,
    col_free_distance, coll_indexes, coll_counts, successors[tid]);
}

__global__ void batch_reeds_shepp_kernel(
  const GpuPose * starts, int n, GpuPose goal, float rho, float * distances)
{
  const int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid >= n) {
    return;
  }
  reeds_shepp_device::StateXYT s0{starts[tid].x, starts[tid].y, starts[tid].yaw};
  reeds_shepp_device::StateXYT s1{goal.x, goal.y, goal.yaw};
  distances[tid] = static_cast<float>(reeds_shepp_device::distance(s0, s1, rho));
}

__global__ void edt_row_kernel(
  const uint8_t * is_obstacle, int width, int height, float resolution, float * dist_sq_x,
  float * offset_x)
{
  const int y = blockIdx.x * blockDim.x + threadIdx.x;
  if (y >= height) {
    return;
  }
  float distance = resolution;
  bool found = false;
  for (int x = 0; x < width; ++x) {
    const int id = y * width + x;
    if (is_obstacle[id]) {
      dist_sq_x[id] = 0.0f;
      offset_x[id] = 0.0f;
      distance = resolution;
      found = true;
    } else if (found) {
      dist_sq_x[id] = distance;
      offset_x[id] = -distance;
      distance += resolution;
    } else {
      dist_sq_x[id] = 1e20f;
      offset_x[id] = 0.0f;
    }
  }
  distance = resolution;
  found = false;
  for (int x = width - 1; x >= 0; --x) {
    const int id = y * width + x;
    if (is_obstacle[id]) {
      distance = resolution;
      found = true;
    } else if (found && dist_sq_x[id] > distance) {
      dist_sq_x[id] = distance;
      offset_x[id] = distance;
      distance += resolution;
    }
  }
}

__global__ void edt_col_kernel(
  const float * dist_x, const float * offset_x, int width, int height, float resolution,
  GpuEDT * edt_out)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  if (x >= width) {
    return;
  }
  for (int y = 0; y < height; ++y) {
    float min_value = 1e20f;
    float rel_x = 0.0f;
    float rel_y = 0.0f;
    for (int k = 0; k < height; ++k) {
      const int id = k * width + x;
      const float d = dist_x[id];
      const float dist = resolution * static_cast<float>(abs(y - k));
      const float value = d * d + dist * dist;
      if (value < min_value) {
        min_value = value;
        rel_x = offset_x[id];
        rel_y = dist;
      }
    }
    const int out_id = y * width + x;
    edt_out[out_id].distance = sqrtf(min_value);
    edt_out[out_id].angle = atan2f(rel_y, rel_x);
  }
}

}  // namespace

bool cuda_device_available()
{
  int count = 0;
  if (cudaGetDeviceCount(&count) != cudaSuccess) {
    return false;
  }
  return count > 0;
}

void cuda_allocate_buffers(
  AstarCudaDeviceBuffers & buffers, size_t grid_size, size_t graph_size, size_t max_coll_total,
  size_t theta_size, size_t max_successors)
{
  cuda_free_buffers(buffers);
  buffers.grid_size = grid_size;
  buffers.graph_size = graph_size;
  buffers.max_successors = max_successors;
  CUDA_CHECK(cudaMalloc(&buffers.d_is_obstacle, grid_size * sizeof(uint8_t)));
  CUDA_CHECK(cudaMalloc(&buffers.d_edt, grid_size * sizeof(GpuEDT)));
  CUDA_CHECK(cudaMalloc(&buffers.d_col_free_distance, grid_size * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&buffers.d_coll_indexes, max_coll_total * sizeof(GpuIndexXY)));
  CUDA_CHECK(cudaMalloc(&buffers.d_coll_counts, theta_size * sizeof(int)));
  CUDA_CHECK(cudaMalloc(&buffers.d_graph, graph_size * sizeof(GpuAstarNode)));
  CUDA_CHECK(cudaMalloc(&buffers.d_successors, max_successors * sizeof(GpuSuccessor)));
}

void cuda_free_buffers(AstarCudaDeviceBuffers & buffers)
{
  auto free_ptr = [](auto *& p) {
    if (p) {
      cudaFree(p);
      p = nullptr;
    }
  };
  free_ptr(buffers.d_is_obstacle);
  free_ptr(buffers.d_edt);
  free_ptr(buffers.d_col_free_distance);
  free_ptr(buffers.d_coll_indexes);
  free_ptr(buffers.d_coll_counts);
  free_ptr(buffers.d_graph);
  free_ptr(buffers.d_successors);
  buffers.graph_size = 0;
  buffers.grid_size = 0;
  buffers.max_successors = 0;
}

void cuda_upload_map(
  AstarCudaDeviceBuffers & buffers, const uint8_t * is_obstacle, const GpuEDT * edt,
  size_t grid_size, const GpuIndexXY * coll_indexes, const int * coll_counts, size_t max_coll_total,
  size_t theta_size)
{
  CUDA_CHECK(
    cudaMemcpy(buffers.d_is_obstacle, is_obstacle, grid_size * sizeof(uint8_t), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(buffers.d_edt, edt, grid_size * sizeof(GpuEDT), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(
    buffers.d_coll_indexes, coll_indexes, max_coll_total * sizeof(GpuIndexXY),
    cudaMemcpyHostToDevice));
  CUDA_CHECK(
    cudaMemcpy(buffers.d_coll_counts, coll_counts, theta_size * sizeof(int), cudaMemcpyHostToDevice));
}

void cuda_upload_col_free_distance(
  AstarCudaDeviceBuffers & buffers, const float * col_free_distance, size_t grid_size)
{
  CUDA_CHECK(cudaMemcpy(
    buffers.d_col_free_distance, col_free_distance, grid_size * sizeof(float),
    cudaMemcpyHostToDevice));
}

void cuda_reset_graph(AstarCudaDeviceBuffers & buffers)
{
  CUDA_CHECK(cudaMemset(buffers.d_graph, 0, buffers.graph_size * sizeof(GpuAstarNode)));
}

void cuda_expand_node(
  const AstarCudaDeviceBuffers & buffers, const GpuAstarParam & param,
  const GpuVehicleShape & vehicle, const GpuPose & goal_pose, const GpuAstarNode & current,
  int current_key, GpuSuccessor * host_successors_out, int & num_successors_out)
{
  const int keys[1] = {current_key};
  cuda_expand_nodes_batch(
    buffers, param, vehicle, goal_pose, &current, keys, 1, host_successors_out, num_successors_out);
}

void cuda_expand_nodes_batch(
  const AstarCudaDeviceBuffers & buffers, const GpuAstarParam & param,
  const GpuVehicleShape & vehicle, const GpuPose & goal_pose, const GpuAstarNode * host_nodes,
  const int * host_keys, int num_nodes, GpuSuccessor * host_successors_out,
  int & num_successors_out)
{
  if (num_nodes <= 0) {
    num_successors_out = 0;
    return;
  }
  const int turn_span = 2 * param.turning_steps + 1;
  const int cands_per_node = turn_span * (param.use_back ? 2 : 1);
  const int total = num_nodes * cands_per_node;
  if (static_cast<size_t>(total) > buffers.max_successors) {
    throw std::runtime_error("successor buffer too small for batch expand");
  }

  GpuAstarNode * d_nodes = nullptr;
  int * d_keys = nullptr;
  CUDA_CHECK(cudaMalloc(&d_nodes, num_nodes * sizeof(GpuAstarNode)));
  CUDA_CHECK(cudaMalloc(&d_keys, num_nodes * sizeof(int)));
  CUDA_CHECK(cudaMemcpy(d_nodes, host_nodes, num_nodes * sizeof(GpuAstarNode), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_keys, host_keys, num_nodes * sizeof(int), cudaMemcpyHostToDevice));

  const int threads = 256;
  const int blocks = (total + threads - 1) / threads;
  expand_nodes_batch_kernel<<<blocks, threads>>>(
    param, vehicle, goal_pose, d_nodes, d_keys, num_nodes, cands_per_node, buffers.d_is_obstacle,
    buffers.d_edt, buffers.d_col_free_distance, buffers.d_coll_indexes, buffers.d_coll_counts,
    buffers.d_successors);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  std::vector<GpuSuccessor> tmp(total);
  CUDA_CHECK(cudaMemcpy(
    tmp.data(), buffers.d_successors, total * sizeof(GpuSuccessor), cudaMemcpyDeviceToHost));
  num_successors_out = 0;
  for (const auto & s : tmp) {
    if (s.valid) {
      host_successors_out[num_successors_out++] = s;
    }
  }
  cudaFree(d_nodes);
  cudaFree(d_keys);
}

void cuda_batch_reeds_shepp_distance(
  const GpuPose * host_starts, int n, const GpuPose & goal, float rho, float * host_distances_out)
{
  if (n <= 0) {
    return;
  }
  GpuPose * d_starts = nullptr;
  float * d_dist = nullptr;
  CUDA_CHECK(cudaMalloc(&d_starts, n * sizeof(GpuPose)));
  CUDA_CHECK(cudaMalloc(&d_dist, n * sizeof(float)));
  CUDA_CHECK(cudaMemcpy(d_starts, host_starts, n * sizeof(GpuPose), cudaMemcpyHostToDevice));
  const int threads = 256;
  const int blocks = (n + threads - 1) / threads;
  batch_reeds_shepp_kernel<<<blocks, threads>>>(d_starts, n, goal, rho, d_dist);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());
  CUDA_CHECK(cudaMemcpy(host_distances_out, d_dist, n * sizeof(float), cudaMemcpyDeviceToHost));
  cudaFree(d_starts);
  cudaFree(d_dist);
}

void cuda_compute_edt(
  const uint8_t * host_is_obstacle, int width, int height, float resolution, GpuEDT * host_edt_out)
{
  const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
  uint8_t * d_obs = nullptr;
  float * d_dist_x = nullptr;
  float * d_off_x = nullptr;
  GpuEDT * d_edt = nullptr;
  CUDA_CHECK(cudaMalloc(&d_obs, n * sizeof(uint8_t)));
  CUDA_CHECK(cudaMalloc(&d_dist_x, n * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_off_x, n * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_edt, n * sizeof(GpuEDT)));
  CUDA_CHECK(cudaMemcpy(d_obs, host_is_obstacle, n * sizeof(uint8_t), cudaMemcpyHostToDevice));

  {
    const int threads = 128;
    const int blocks = (height + threads - 1) / threads;
    edt_row_kernel<<<blocks, threads>>>(d_obs, width, height, resolution, d_dist_x, d_off_x);
    CUDA_CHECK(cudaGetLastError());
  }
  {
    const int threads = 128;
    const int blocks = (width + threads - 1) / threads;
    edt_col_kernel<<<blocks, threads>>>(d_dist_x, d_off_x, width, height, resolution, d_edt);
    CUDA_CHECK(cudaGetLastError());
  }
  CUDA_CHECK(cudaDeviceSynchronize());
  CUDA_CHECK(cudaMemcpy(host_edt_out, d_edt, n * sizeof(GpuEDT), cudaMemcpyDeviceToHost));
  cudaFree(d_obs);
  cudaFree(d_dist_x);
  cudaFree(d_off_x);
  cudaFree(d_edt);
}

}  // namespace autoware::freespace_planning_algorithms
