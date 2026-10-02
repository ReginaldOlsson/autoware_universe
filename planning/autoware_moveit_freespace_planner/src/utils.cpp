// Copyright 2024 TIER IV, Inc.
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

#include "autoware/moveit_freespace_planner/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <tf2/utils.hpp>

#include <cmath>
#include <deque>
#include <vector>

namespace autoware::moveit_freespace_planner::utils
{

PoseArray trajectory_to_pose_array(const Trajectory & trajectory)
{
  PoseArray pose_array;
  pose_array.header = trajectory.header;

  for (const auto & point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  return pose_array;
}

double calc_distance_2d(const Trajectory & trajectory, const Pose & pose)
{
  const auto idx = autoware::motion_utils::findNearestIndex(trajectory.points, pose.position);
  return autoware_utils::calc_distance2d(trajectory.points.at(idx), pose);
}

Pose transform_pose(const Pose & pose, const TransformStamped & transform)
{
  PoseStamped transformed_pose;
  PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

bool is_active(const Scenario::ConstSharedPtr & scenario)
{
  if (!scenario) return false;

  const auto & s = scenario->activating_scenarios;
  return std::find(std::begin(s), std::end(s), Scenario::PARKING) != std::end(s);
}

std::vector<size_t> get_reversing_indices(const Trajectory & trajectory)
{
  std::vector<size_t> indices;
  if (trajectory.points.size() < 2) {
    return indices;
  }

  for (size_t i = 0; i + 1 < trajectory.points.size(); ++i) {
    const double v0 = trajectory.points.at(i).longitudinal_velocity_mps;
    const double v1 = trajectory.points.at(i + 1).longitudinal_velocity_mps;
    // Ignore near-zero samples so cusp stop points do not create duplicate splits.
    if (std::abs(v0) < 1e-3 || std::abs(v1) < 1e-3) {
      continue;
    }
    if (v0 * v1 < 0.0) {
      indices.push_back(i);
    }
  }

  return indices;
}

size_t get_next_target_index(
  const size_t trajectory_size, const std::vector<size_t> & reversing_indices,
  const size_t current_target_index)
{
  if (!reversing_indices.empty()) {
    for (const auto reversing_index : reversing_indices) {
      if (reversing_index > current_target_index) {
        return reversing_index;
      }
    }
  }

  return trajectory_size - 1;
}

namespace
{
int edge_gear(const Pose & from, const Pose & to)
{
  const double yaw = tf2::getYaw(from.orientation);
  const double dx = to.position.x - from.position.x;
  const double dy = to.position.y - from.position.y;
  if (std::hypot(dx, dy) < 1e-4) {
    return 0;
  }
  const double forward_dot = std::cos(yaw) * dx + std::sin(yaw) * dy;
  return (forward_dot >= 0.0) ? 1 : -1;
}
}  // namespace

Trajectory get_partial_trajectory(
  const Trajectory & trajectory, const size_t start_index, const size_t end_index,
  const rclcpp::Clock::SharedPtr clock)
{
  Trajectory partial_trajectory;
  partial_trajectory.header = trajectory.header;
  partial_trajectory.header.stamp = clock->now();

  if (trajectory.points.empty() || start_index > end_index ||
      end_index >= trajectory.points.size()) {
    return partial_trajectory;
  }

  // One gear per partial: skip leading poses whose outgoing edge disagrees
  // with the rest of the slice (e.g. a forward hook on a reverse partial).
  size_t start = start_index;
  int segment_gear = 0;
  for (size_t i = start; i + 1 < end_index; ++i) {
    const int g0 = edge_gear(trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose);
    const int g1 = edge_gear(trajectory.points.at(i + 1).pose, trajectory.points.at(i + 2).pose);
    if (g0 != 0 && g0 == g1) {
      segment_gear = g0;
      break;
    }
  }
  if (segment_gear == 0) {
    for (size_t i = start; i < end_index; ++i) {
      const int g = edge_gear(trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose);
      if (g != 0) {
        segment_gear = g;
        break;
      }
    }
  }
  while (start < end_index && segment_gear != 0) {
    const int g = edge_gear(trajectory.points.at(start).pose, trajectory.points.at(start + 1).pose);
    if (g == 0 || g == segment_gear) {
      break;
    }
    ++start;
  }

  partial_trajectory.points.reserve(end_index - start + 1);
  for (size_t i = start; i <= end_index; ++i) {
    partial_trajectory.points.push_back(trajectory.points.at(i));
  }

  if (partial_trajectory.points.empty()) {
    return partial_trajectory;
  }

  // Force one velocity sign for the whole partial (removes reverse artifacts).
  double segment_speed = 0.0;
  for (const auto & pt : partial_trajectory.points) {
    if (std::abs(pt.longitudinal_velocity_mps) > 1e-3) {
      segment_speed = pt.longitudinal_velocity_mps;
      break;
    }
  }
  if (std::abs(segment_speed) < 1e-3 && partial_trajectory.points.size() >= 2) {
    // Infer from geometry if velocities were wiped.
    const auto & p0 = partial_trajectory.points.front().pose;
    const auto & p1 = partial_trajectory.points.at(1).pose;
    const double yaw = tf2::getYaw(p0.orientation);
    const double dx = p1.position.x - p0.position.x;
    const double dy = p1.position.y - p0.position.y;
    const double forward_dot = std::cos(yaw) * dx + std::sin(yaw) * dy;
    segment_speed = (forward_dot >= 0.0) ? 1.0 : -1.0;
    // Magnitude will be replaced below from |original| if available.
    for (const auto & pt : trajectory.points) {
      if (std::abs(pt.longitudinal_velocity_mps) > 1e-3) {
        segment_speed = std::copysign(std::abs(pt.longitudinal_velocity_mps), segment_speed);
        break;
      }
    }
  }

  const double speed_signed = segment_speed;
  for (size_t i = 0; i + 1 < partial_trajectory.points.size(); ++i) {
    auto & pt = partial_trajectory.points[i];
    pt.longitudinal_velocity_mps = speed_signed;
    pt.lateral_velocity_mps = 0.0;
    pt.heading_rate_rps = 0.0;
    pt.acceleration_mps2 = 0.0;
  }
  {
    auto & back = partial_trajectory.points.back();
    back.longitudinal_velocity_mps = 0.0;
    back.lateral_velocity_mps = 0.0;
    back.heading_rate_rps = 0.0;
    back.acceleration_mps2 = 0.0;
  }

  return partial_trajectory;
}

Trajectory create_trajectory(
  const PoseStamped & current_pose, const PlannerWaypoints & planner_waypoints,
  const double & velocity)
{
  Trajectory trajectory;
  trajectory.header = planner_waypoints.header;

  const double speed = std::abs(velocity / 3.6);  // km/h -> m/s

  for (const auto & awp : planner_waypoints.waypoints) {
    TrajectoryPoint point;

    point.pose = awp.pose.pose;
    point.pose.position.z = current_pose.pose.position.z;  // height = const
    point.longitudinal_velocity_mps = (awp.is_back ? -speed : speed);
    point.lateral_velocity_mps = 0.0;
    point.heading_rate_rps = 0.0;
    point.acceleration_mps2 = 0.0;

    trajectory.points.push_back(point);
  }

  // Ensure contiguous unidirectional runs: no single-point opposite signs.
  if (trajectory.points.size() >= 3) {
    for (size_t i = 1; i + 1 < trajectory.points.size(); ++i) {
      const double v_prev = trajectory.points[i - 1].longitudinal_velocity_mps;
      const double v = trajectory.points[i].longitudinal_velocity_mps;
      const double v_next = trajectory.points[i + 1].longitudinal_velocity_mps;
      if (v * v_prev < 0.0 && v * v_next < 0.0 && v_prev * v_next > 0.0) {
        trajectory.points[i].longitudinal_velocity_mps = v_prev;
      }
    }
  }

  return trajectory;
}

Trajectory create_stop_trajectory(
  const PoseStamped & current_pose, const rclcpp::Clock::SharedPtr clock)
{
  PlannerWaypoints waypoints;
  PlannerWaypoint waypoint;

  waypoints.header.stamp = clock->now();
  waypoints.header.frame_id = current_pose.header.frame_id;
  waypoint.pose.header = waypoints.header;
  waypoint.pose.pose = current_pose.pose;
  waypoint.is_back = false;
  waypoints.waypoints.push_back(waypoint);

  return create_trajectory(current_pose, waypoints, 0.0);
}

Trajectory create_stop_trajectory(const Trajectory & trajectory)
{
  Trajectory stop_trajectory = trajectory;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    stop_trajectory.points.at(i).longitudinal_velocity_mps = 0.0;
  }
  return stop_trajectory;
}

bool is_stopped(
  const std::deque<Odometry::ConstSharedPtr> & odom_buffer, const double th_stopped_velocity_mps)
{
  const double th_stopped_velocity_sq = th_stopped_velocity_mps * th_stopped_velocity_mps;
  for (const auto & odom : odom_buffer) {
    const auto & lin = odom->twist.twist.linear;
    const double velocity_sq = lin.x * lin.x + lin.y * lin.y + lin.z * lin.z;
    if (velocity_sq > th_stopped_velocity_sq) {
      return false;
    }
  }
  return true;
}

bool is_near_target(const Pose & target_pose, const Pose & current_pose, const double th_distance_m)
{
  const auto pose_dev = autoware_utils::calc_pose_deviation(target_pose, current_pose);
  return abs(pose_dev.yaw) < M_PI_2 && abs(pose_dev.longitudinal) < th_distance_m &&
         abs(pose_dev.lateral) < th_distance_m;
}
}  // namespace autoware::moveit_freespace_planner::utils
