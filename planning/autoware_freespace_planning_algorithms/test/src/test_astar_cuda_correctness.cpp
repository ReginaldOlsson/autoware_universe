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
#include "autoware/freespace_planning_algorithms/astar_search.hpp"
#include "autoware/freespace_planning_algorithms/astar_search_cuda.hpp"
#include "autoware/freespace_planning_algorithms/reeds_shepp.hpp"
#include "autoware/freespace_planning_algorithms/reeds_shepp_device.cuh"

#include <tf2/utils.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

namespace fpa = autoware::freespace_planning_algorithms;

namespace
{
const double length_lexus = 5.5;
const double width_lexus = 2.75;
const double base_length_lexus = 3.0;
const double max_steering_lexus = 0.7;
const fpa::VehicleShape vehicle_shape =
  fpa::VehicleShape(length_lexus, width_lexus, base_length_lexus, max_steering_lexus, 1.5);
constexpr double pi = 3.1415926;
const std::array<double, 3> start_pose{5.5, 4., pi * 0.5};
const std::array<double, 3> goal_pose1{8.0, 26.3, pi * 1.5};
const std::array<double, 3> goal_pose4{25.0, 26.3, pi * 1.5};

geometry_msgs::msg::Pose create_pose_msg(std::array<double, 3> pose3d)
{
  geometry_msgs::msg::Pose pose{};
  tf2::Quaternion quat{};
  quat.setRPY(0, 0, pose3d[2]);
  tf2::convert(quat, pose.orientation);
  pose.position.x = pose3d[0];
  pose.position.y = pose3d[1];
  pose.position.z = 0.0;
  return pose;
}

nav_msgs::msg::OccupancyGrid construct_cost_map(
  size_t width, size_t height, double resolution, size_t n_padding)
{
  nav_msgs::msg::OccupancyGrid costmap_msg{};
  costmap_msg.info.width = width;
  costmap_msg.info.height = height;
  costmap_msg.info.resolution = resolution;
  costmap_msg.data.assign(width * height, 0);
  for (size_t i = 0; i < n_padding; ++i) {
    for (size_t j = width * i; j <= width * (i + 1) && j < costmap_msg.data.size(); ++j) {
      costmap_msg.data[j] = 100;
    }
    for (size_t j = width * (height - n_padding + i);
         j <= width * (height - n_padding + i + 1) && j < costmap_msg.data.size(); ++j) {
      costmap_msg.data[j] = 100;
    }
  }
  for (size_t i = 0; i < height; ++i) {
    for (size_t j = 0; j < n_padding && j < width; ++j) {
      costmap_msg.data[i * width + j] = 100;
      costmap_msg.data[i * width + (width - 1 - j)] = 100;
    }
  }
  for (size_t i = 0; i < height; ++i) {
    for (size_t j = 0; j < width; ++j) {
      const double x = j * resolution;
      const double y = i * resolution;
      if (8.0 < x && x < 28.0 && 9.0 < y && y < 9.5) {
        costmap_msg.data[i * width + j] = 100;
      }
      if (10.0 < x && x < 10.0 + width_lexus && 22.0 < y && y < 22.0 + length_lexus) {
        costmap_msg.data[i * width + j] = 100;
      }
      if (13.5 < x && x < 13.5 + width_lexus && 22.0 < y && y < 22.0 + length_lexus) {
        costmap_msg.data[i * width + j] = 100;
      }
      if (20.0 < x && x < 20.0 + width_lexus && 22.0 < y && y < 22.0 + length_lexus) {
        costmap_msg.data[i * width + j] = 100;
      }
      if (10.0 < x && x < 10.0 + width_lexus && 10.0 < y && y < 10.0 + length_lexus) {
        costmap_msg.data[i * width + j] = 100;
      }
    }
  }
  return costmap_msg;
}

fpa::PlannerCommonParam get_default_planner_params()
{
  return fpa::PlannerCommonParam{
    30 * 1000.0, 144, 0.5, 1.0, 1.5, 0.5, 2.0, 6.0, 0.5, 1, 100};
}

fpa::AstarParam get_astar_param()
{
  return fpa::AstarParam{
    "forward", false, true, true, 0.4, 4.0, 2.0, 0.5, 1.7, 1.0};
}
}  // namespace

TEST(AstarCuda, DeviceAvailable)
{
  ASSERT_TRUE(fpa::AstarSearchCuda::isCudaAvailable());
}

TEST(AstarCuda, ReedsSheppParity)
{
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist_xy(-20.0, 20.0);
  std::uniform_real_distribution<double> dist_yaw(-M_PI, M_PI);
  const double rho = 5.0;
  fpa::ReedsSheppStateSpace cpu(rho);
  for (int i = 0; i < 50; ++i) {
    fpa::ReedsSheppStateSpace::StateXYT s0{dist_xy(rng), dist_xy(rng), dist_yaw(rng)};
    fpa::ReedsSheppStateSpace::StateXYT s1{dist_xy(rng), dist_xy(rng), dist_yaw(rng)};
    const double d_cpu = cpu.distance(s0, s1);
    const double d_gpu = fpa::reeds_shepp_device::distance(
      {s0.x, s0.y, s0.yaw}, {s1.x, s1.y, s1.yaw}, rho);
    EXPECT_NEAR(d_cpu, d_gpu, 1e-4) << "pair " << i;
  }
}

TEST(AstarCuda, BatchReedsSheppKernel)
{
  std::vector<fpa::GpuPose> starts(32);
  for (size_t i = 0; i < starts.size(); ++i) {
    starts[i] = {static_cast<float>(i) * 0.5f, 1.0f, 0.2f};
  }
  const fpa::GpuPose goal{10.0f, 10.0f, 1.0f};
  std::vector<float> distances(starts.size());
  fpa::cuda_batch_reeds_shepp_distance(starts.data(), static_cast<int>(starts.size()), goal, 5.0f,
                                       distances.data());
  fpa::ReedsSheppStateSpace cpu(5.0);
  for (size_t i = 0; i < starts.size(); ++i) {
    const double d_cpu = cpu.distance(
      {starts[i].x, starts[i].y, starts[i].yaw}, {goal.x, goal.y, goal.yaw});
    EXPECT_NEAR(d_cpu, distances[i], 1e-3);
  }
}

TEST(AstarCuda, PlanEasyAndHardGoals)
{
  if (!fpa::AstarSearchCuda::isCudaAvailable()) {
    GTEST_SKIP() << "No CUDA device";
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  const auto params = get_default_planner_params();
  const auto astar_param = get_astar_param();
  auto cpu = std::make_unique<fpa::AstarSearch>(params, vehicle_shape, astar_param, clock);
  auto gpu = std::make_unique<fpa::AstarSearchCuda>(params, vehicle_shape, astar_param, clock);
  const auto map = construct_cost_map(150, 150, 0.2, 10);

  for (const auto & goal : {goal_pose1, goal_pose4}) {
    cpu->setMap(map);
    gpu->setMap(map);
    bool cpu_ok = true;
    bool gpu_ok = true;
    try {
      cpu->makePlan(create_pose_msg(start_pose), create_pose_msg(goal));
    } catch (...) {
      cpu_ok = false;
    }
    try {
      gpu->makePlan(create_pose_msg(start_pose), create_pose_msg(goal));
    } catch (...) {
      gpu_ok = false;
    }
    EXPECT_EQ(cpu_ok, gpu_ok) << "success mismatch for goal";
    if (!gpu_ok) {
      continue;
    }
    geometry_msgs::msg::PoseArray trajectory;
    for (const auto & wp : gpu->getWaypoints().waypoints) {
      trajectory.poses.push_back(wp.pose.pose);
    }
    EXPECT_FALSE(gpu->hasObstacleOnTrajectory(trajectory));
    EXPECT_GT(gpu->getWaypoints().waypoints.size(), 1u);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
