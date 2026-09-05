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

#ifndef AUTOWARE__MOVEIT_FREESPACE_PLANNING_ALGORITHMS__MOVEIT_ACKERMANN_PLANNER_HPP_
#define AUTOWARE__MOVEIT_FREESPACE_PLANNING_ALGORITHMS__MOVEIT_ACKERMANN_PLANNER_HPP_

#include <ackermann_ompl_plugins/ackermann_ompl_solver.hpp>
#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace autoware::moveit_freespace_planning_algorithms
{
using autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm;
using autoware::freespace_planning_algorithms::PlannerCommonParam;
using autoware::freespace_planning_algorithms::VehicleShape;

struct MoveItAckermannParam
{
  std::string planner_id{"RRTConnect"};
  double turning_radius{-1.0};
  double longest_valid_segment_fraction{0.05};
  /// Collision-check spacing [m]. <=0 uses costmap resolution.
  double valid_segment_length{-1.0};
  double interpolate_resolution{0.5};
  int interpolate_count{0};
  bool simplify{true};
  bool try_analytic_reeds_shepp{true};
  double planner_range{8.0};
  /// Hard cap on OMPL solve time [s]; Autoware time_limit is also considered.
  double max_planning_time{2.0};
};

/**
 * @brief Autoware freespace backend: AbstractPlanningAlgorithm::makePlan via OMPL Reeds-Shepp.
 *
 * Collision checking reuses AbstractPlanningAlgorithm OccupancyGrid tables.
 */
class MoveItAckermannPlanner : public AbstractPlanningAlgorithm
{
public:
  MoveItAckermannPlanner(
    const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
    const MoveItAckermannParam & moveit_param, const rclcpp::Clock::SharedPtr & clock);

  MoveItAckermannPlanner(
    const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
    rclcpp::Node & node);

  bool makePlan(
    const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Pose & goal_pose) override;

  bool makePlan(
    const geometry_msgs::msg::Pose & start_pose,
    const std::vector<geometry_msgs::msg::Pose> & goal_candidates) override;

private:
  MoveItAckermannParam moveit_param_;
  ackermann_ompl_plugins::AckermannOmplSolver solver_;
};

}  // namespace autoware::moveit_freespace_planning_algorithms

#endif  // AUTOWARE__MOVEIT_FREESPACE_PLANNING_ALGORITHMS__MOVEIT_ACKERMANN_PLANNER_HPP_
