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

#include "autoware/moveit_freespace_planning_algorithms/moveit_ackermann_planner.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::moveit_freespace_planning_algorithms
{
namespace
{
double derive_turning_radius(const VehicleShape & shape)
{
  const double steer = std::max(1e-3, std::abs(shape.max_steering));
  return std::max(0.5, shape.base_length / std::tan(steer));
}
}  // namespace

MoveItAckermannPlanner::MoveItAckermannPlanner(
  const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
  const MoveItAckermannParam & moveit_param, const rclcpp::Clock::SharedPtr & clock)
: AbstractPlanningAlgorithm(planner_common_param, clock, collision_vehicle_shape),
  moveit_param_(moveit_param)
{
  if (moveit_param_.turning_radius <= 0.0) {
    moveit_param_.turning_radius = derive_turning_radius(collision_vehicle_shape_);
  }
}

MoveItAckermannPlanner::MoveItAckermannPlanner(
  const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
  rclcpp::Node & node)
: MoveItAckermannPlanner(
    planner_common_param, collision_vehicle_shape,
    [&node]() {
      MoveItAckermannParam p;
      p.planner_id = node.declare_parameter<std::string>("moveit.planner_id", "RRTConnect");
      p.turning_radius = node.declare_parameter<double>("moveit.turning_radius", -1.0);
      p.longest_valid_segment_fraction =
        node.declare_parameter<double>("moveit.longest_valid_segment_fraction", 0.05);
      p.interpolate_resolution =
        node.declare_parameter<double>("moveit.interpolate_resolution", 0.5);
      p.interpolate_count =
        static_cast<int>(node.declare_parameter<int64_t>("moveit.interpolate_count", 0));
      p.simplify = node.declare_parameter<bool>("moveit.simplify", true);
      p.try_analytic_reeds_shepp =
        node.declare_parameter<bool>("moveit.try_analytic_reeds_shepp", true);
      p.max_planning_time = node.declare_parameter<double>("moveit.max_planning_time", 5.0);
      p.planner_range = node.declare_parameter<double>("moveit.planner_range", 5.0);
      return p;
    }(),
    node.get_clock())
{
  RCLCPP_INFO(
    node.get_logger(),
    "MoveItAckermannPlanner: planner=%s turning_radius=%.2f max_time=%.2fs analytic_first=%s",
    moveit_param_.planner_id.c_str(), moveit_param_.turning_radius,
    moveit_param_.max_planning_time, moveit_param_.try_analytic_reeds_shepp ? "true" : "false");
}

bool MoveItAckermannPlanner::makePlan(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose)
{
  return makePlan(start_pose, std::vector<geometry_msgs::msg::Pose>{goal_pose});
}

bool MoveItAckermannPlanner::makePlan(
  const geometry_msgs::msg::Pose & start_pose,
  const std::vector<geometry_msgs::msg::Pose> & goal_candidates)
{
  if (goal_candidates.empty()) {
    return false;
  }

  using autoware::freespace_planning_algorithms::global2local;
  using autoware::freespace_planning_algorithms::local2global;

  start_pose_ = global2local(costmap_, start_pose);

  ackermann_ompl_plugins::OmplSolveRequest req;
  req.start_pose = start_pose_;
  req.planner_id = moveit_param_.planner_id;
  // Cap solve time: Autoware time_limit is ms, moveit.max_planning_time is seconds.
  const double from_common = std::max(0.1, planner_common_param_.time_limit / 1000.0);
  req.allowed_planning_time = std::min(from_common, moveit_param_.max_planning_time);
  req.turning_radius = moveit_param_.turning_radius;
  req.longest_valid_segment_fraction = moveit_param_.longest_valid_segment_fraction;
  req.interpolate_resolution = moveit_param_.interpolate_resolution;
  req.interpolate_count = moveit_param_.interpolate_count;
  req.simplify = moveit_param_.simplify;
  req.try_analytic_reeds_shepp = moveit_param_.try_analytic_reeds_shepp;
  req.planner_range = moveit_param_.planner_range;
  req.goal_xy_tolerance =
    std::max(planner_common_param_.lateral_goal_range, planner_common_param_.longitudinal_goal_range);
  req.goal_yaw_tolerance = planner_common_param_.angle_goal_range * M_PI / 180.0;

  const double res = costmap_.info.resolution;
  req.bounds_low_x = 0.0;
  req.bounds_low_y = 0.0;
  req.bounds_high_x = static_cast<double>(costmap_.info.width) * res;
  req.bounds_high_y = static_cast<double>(costmap_.info.height) * res;

  const auto is_valid = [this](double x, double y, double yaw) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation.z = std::sin(yaw * 0.5);
    pose.orientation.w = std::cos(yaw * 0.5);
    return !detectCollision(pose);
  };

  ackermann_ompl_plugins::OmplSolveResult best;
  best.success = false;
  double best_length = std::numeric_limits<double>::infinity();

  for (const auto & goal_global : goal_candidates) {
    goal_pose_ = global2local(costmap_, goal_global);
    req.goal_pose = goal_pose_;

    const auto solved = solver_.solve(req, is_valid);
    if (!solved.success || solved.waypoints.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("MoveItAckermannPlanner"), *clock_, 2000,
        "Planning failed: %s", solved.message.c_str());
      continue;
    }

    double length = 0.0;
    for (size_t i = 1; i < solved.waypoints.size(); ++i) {
      const auto & a = solved.waypoints[i - 1].pose.position;
      const auto & b = solved.waypoints[i].pose.position;
      length += std::hypot(b.x - a.x, b.y - a.y);
    }
    if (length < best_length) {
      best_length = length;
      best = solved;
    }
  }

  if (!best.success) {
    return false;
  }

  waypoints_.header = costmap_.header;
  waypoints_.waypoints.clear();
  waypoints_.waypoints.reserve(best.waypoints.size());

  for (const auto & wp : best.waypoints) {
    autoware::freespace_planning_algorithms::PlannerWaypoint awp;
    awp.pose.header = costmap_.header;
    awp.pose.pose = local2global(costmap_, wp.pose);
    // Preserve global z from current/start for controller consistency.
    awp.pose.pose.position.z = start_pose.position.z;
    awp.is_back = wp.is_back;
    waypoints_.waypoints.push_back(awp);
  }

  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("MoveItAckermannPlanner"), *clock_, 2000,
    "Planned %zu waypoints (%.1fm) via %s", waypoints_.waypoints.size(), best_length,
    best.message.c_str());

  return !waypoints_.waypoints.empty();
}

}  // namespace autoware::moveit_freespace_planning_algorithms
