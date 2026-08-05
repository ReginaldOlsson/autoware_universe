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

#ifndef AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ASTAR_SEARCH_CUDA_HPP_
#define AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ASTAR_SEARCH_CUDA_HPP_

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include "autoware/freespace_planning_algorithms/astar_cuda_kernels.hpp"
#include "autoware/freespace_planning_algorithms/astar_cuda_types.hpp"
#include "autoware/freespace_planning_algorithms/astar_search.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <queue>
#include <string>
#include <vector>

namespace autoware::freespace_planning_algorithms
{

/**
 * @brief CUDA-accelerated Hybrid A* with the same public contract as AstarSearch.
 * Host open-list preserves A* ordering; successor expansion (kinematics, collision,
 * Reeds-Shepp heuristic) runs as a CUDA kernel per expanded node.
 */
class AstarSearchCuda : public AbstractPlanningAlgorithm
{
public:
  AstarSearchCuda(
    const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
    const AstarParam & astar_param, const rclcpp::Clock::SharedPtr & clock);

  AstarSearchCuda(
    const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
    rclcpp::Node & node)
  : AstarSearchCuda(
      planner_common_param, collision_vehicle_shape,
      AstarParam{
        node.declare_parameter<std::string>("astar.search_method"),
        node.declare_parameter<bool>("astar.only_behind_solutions"),
        node.declare_parameter<bool>("astar.use_back"),
        node.declare_parameter<bool>("astar.adapt_expansion_distance"),
        node.declare_parameter<double>("astar.expansion_distance"),
        node.declare_parameter<double>("astar.near_goal_distance"),
        node.declare_parameter<double>("astar.distance_heuristic_weight"),
        node.declare_parameter<double>("astar.smoothness_weight"),
        node.declare_parameter<double>("astar.obstacle_distance_weight"),
        node.declare_parameter<double>("astar.goal_lat_distance_weight")},
      node.get_clock())
  {
  }

  ~AstarSearchCuda() override;

  void setMap(const nav_msgs::msg::OccupancyGrid & costmap) override;
  bool makePlan(const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose)
    override;
  bool makePlan(
    const geometry_msgs::msg::Pose & start_pose,
    const std::vector<geometry_msgs::msg::Pose> & goal_candidates) override;

  /// Timing helpers used by benchmarks (milliseconds).
  double last_set_map_ms() const { return last_set_map_ms_; }
  double last_plan_ms() const { return last_plan_ms_; }
  size_t last_expansions() const { return last_expansions_; }

  static bool isCudaAvailable() { return cuda_device_available(); }

private:
  struct OpenEntry
  {
    int key;
    float fc;
  };
  struct OpenCompare
  {
    bool operator()(const OpenEntry & a, const OpenEntry & b) const { return a.fc > b.fc; }
  };

  void resetData();
  void setCollisionFreeDistanceMap();
  void setStartNode();
  bool search();
  bool isGoal(const GpuAstarNode & node) const;
  void setPath(int goal_key);
  GpuAstarParam makeGpuParam() const;
  GpuVehicleShape makeGpuVehicle() const;

  AstarParam astar_param_;
  AstarCudaDeviceBuffers buffers_;

  std::vector<GpuAstarNode> graph_;
  std::vector<float> col_free_distance_map_;
  std::vector<uint8_t> is_obstacle_u8_;
  std::vector<GpuEDT> edt_gpu_;
  std::vector<GpuIndexXY> coll_indexes_flat_;
  std::vector<int> coll_counts_;
  int max_coll_indexes_{0};

  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenCompare> openlist_;

  double steering_resolution_{0.0};
  double heading_resolution_{0.0};
  double avg_turning_radius_{0.0};
  double min_expansion_dist_{0.0};
  double max_expansion_dist_{0.0};
  double near_goal_dist_{0.0};
  bool is_backward_search_{false};

  double last_set_map_ms_{0.0};
  double last_plan_ms_{0.0};
  size_t last_expansions_{0};

  static constexpr double base_length_max_expansion_factor_ = 0.5;
  static constexpr double cost_free_obs_dist_ = 1.0;
};

}  // namespace autoware::freespace_planning_algorithms

#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ASTAR_SEARCH_CUDA_HPP_
