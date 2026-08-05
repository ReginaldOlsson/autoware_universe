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

#ifndef AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ASTAR_CUDA_TYPES_HPP_
#define AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ASTAR_CUDA_TYPES_HPP_

#include <cstdint>

namespace autoware::freespace_planning_algorithms
{

enum class GpuNodeStatus : uint8_t { None = 0, Open = 1, Closed = 2 };

struct GpuEDT
{
  float distance;
  float angle;
};

struct GpuIndexXY
{
  int x;
  int y;
};

struct GpuVehicleShape
{
  float length;
  float width;
  float base_length;
  float max_steering;
  float base2back;
  float min_dimension;
  float max_dimension;
};

struct GpuAstarParam
{
  float distance_heuristic_weight;
  float smoothness_weight;
  float obstacle_distance_weight;
  float goal_lat_distance_weight;
  float expansion_distance;
  float near_goal_distance;
  float curve_weight;
  float reverse_weight;
  float direction_change_weight;
  float avg_turning_radius;
  float steering_resolution;
  float min_expansion_dist;
  float max_expansion_dist;
  float lateral_goal_range;
  float longitudinal_goal_range;
  float angle_goal_range_rad;
  float cost_free_obs_dist;
  float resolution;
  int width;
  int height;
  int theta_size;
  int turning_steps;
  int max_coll_indexes;
  int use_back;
  int adapt_expansion_distance;
  int only_behind_solutions;
  int is_backward_search;
  int use_reeds_shepp;
};

struct GpuPose
{
  float x;
  float y;
  float yaw;
};

struct GpuAstarNode
{
  float x;
  float y;
  float theta;
  float gc;
  float fc;
  float dir_distance;
  float dist_to_goal;
  float dist_to_obs;
  int steering_index;
  int parent_key;  // -1 if none
  uint8_t status;
  uint8_t is_back;
};

struct GpuSuccessor
{
  float x;
  float y;
  float theta;
  float gc;
  float fc;
  float dir_distance;
  float dist_to_goal;
  float dist_to_obs;
  int key;
  int parent_key;
  int steering_index;
  uint8_t is_back;
  uint8_t valid;
};

}  // namespace autoware::freespace_planning_algorithms

#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ASTAR_CUDA_TYPES_HPP_
