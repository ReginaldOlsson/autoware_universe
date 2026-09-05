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

#include <tf2/utils.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

namespace
{
using autoware::freespace_planning_algorithms::PlannerCommonParam;
using autoware::freespace_planning_algorithms::VehicleShape;
using autoware::moveit_freespace_planning_algorithms::MoveItAckermannParam;
using autoware::moveit_freespace_planning_algorithms::MoveItAckermannPlanner;

geometry_msgs::msg::Pose make_pose(double x, double y, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation.z = std::sin(yaw * 0.5);
  pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

nav_msgs::msg::OccupancyGrid make_blocked_costmap()
{
  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header.frame_id = "map";
  const double resolution = 0.3;
  const int width = 80;
  const int height = 80;
  costmap.info.resolution = resolution;
  costmap.info.width = width;
  costmap.info.height = height;
  costmap.info.origin.orientation.w = 1.0;
  costmap.data.assign(static_cast<size_t>(width * height), 0);

  // Vertical wall across most of the map, leaving a gap at the top to detour.
  // Blocks the straight start->goal line at x ≈ 12 m.
  for (int i = 0; i < height; ++i) {
    const double y = i * resolution;
    if (y > 20.0) {
      continue;
    }
    for (int j = 0; j < width; ++j) {
      const double x = j * resolution;
      if (x >= 11.4 && x <= 13.2) {
        costmap.data[i * width + j] = 100;
      }
    }
  }
  return costmap;
}

PlannerCommonParam make_common_param()
{
  PlannerCommonParam p;
  p.time_limit = 5000.0;
  p.theta_size = 72;
  p.curve_weight = 0.5;
  p.reverse_weight = 0.7;
  p.direction_change_weight = 0.0;
  p.lateral_goal_range = 0.6;
  p.longitudinal_goal_range = 0.6;
  p.angle_goal_range = 15.0;
  p.max_turning_ratio = 1.0;
  p.turning_steps = 1;
  p.obstacle_threshold = 100;
  return p;
}

MoveItAckermannParam make_moveit_param()
{
  MoveItAckermannParam p;
  p.planner_id = "RRTConnect";
  p.turning_radius = 3.5;
  p.valid_segment_length = 0.3;
  p.longest_valid_segment_fraction = 0.05;
  p.interpolate_resolution = 0.3;
  p.interpolate_count = 0;
  p.simplify = true;
  // Analytic-first would have accepted a 5 m-sampled RS through the wall.
  p.try_analytic_reeds_shepp = true;
  p.planner_range = 4.0;
  p.max_planning_time = 5.0;
  return p;
}
}  // namespace

TEST(MoveItAckermannPlanner, DoesNotPlanThroughCostmapObstacle)
{
  const auto vehicle = VehicleShape(4.0, 1.8, 2.5, 0.6, 0.9);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  MoveItAckermannPlanner planner(make_common_param(), vehicle, make_moveit_param(), clock);

  const auto costmap = make_blocked_costmap();
  planner.setMap(costmap);

  const auto start = make_pose(4.0, 12.0, 0.0);
  const auto goal = make_pose(20.0, 12.0, 0.0);

  const bool ok = planner.makePlan(start, goal);
  if (!ok) {
    SUCCEED();
    return;
  }

  geometry_msgs::msg::PoseArray trajectory;
  trajectory.header = planner.getWaypoints().header;
  for (const auto & wp : planner.getWaypoints().waypoints) {
    trajectory.poses.push_back(wp.pose.pose);
  }

  EXPECT_GE(trajectory.poses.size(), 2u);
  EXPECT_FALSE(planner.hasObstacleOnTrajectory(trajectory));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
