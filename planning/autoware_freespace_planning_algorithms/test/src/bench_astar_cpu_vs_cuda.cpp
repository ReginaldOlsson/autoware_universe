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

#include "autoware/freespace_planning_algorithms/astar_search.hpp"
#include "autoware/freespace_planning_algorithms/astar_search_cuda.hpp"

#include <tf2/utils.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace fpa = autoware::freespace_planning_algorithms;

namespace
{
const double length_lexus = 5.5;
const double width_lexus = 2.75;
const fpa::VehicleShape vehicle_shape =
  fpa::VehicleShape(length_lexus, width_lexus, 3.0, 0.7, 1.5);
constexpr double pi = 3.1415926;
const std::array<double, 3> start_pose{5.5, 4., pi * 0.5};
const std::array<double, 3> goal_easy{8.0, 26.3, pi * 1.5};
const std::array<double, 3> goal_hard{25.0, 26.3, pi * 1.5};

geometry_msgs::msg::Pose create_pose_msg(std::array<double, 3> pose3d)
{
  geometry_msgs::msg::Pose pose{};
  tf2::Quaternion quat{};
  quat.setRPY(0, 0, pose3d[2]);
  tf2::convert(quat, pose.orientation);
  pose.position.x = pose3d[0];
  pose.position.y = pose3d[1];
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
  for (size_t i = 0; i < height; ++i) {
    for (size_t j = 0; j < width; ++j) {
      if (j < n_padding || j + n_padding >= width || i < n_padding || i + n_padding >= height) {
        costmap_msg.data[i * width + j] = 100;
      }
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

fpa::PlannerCommonParam params()
{
  return fpa::PlannerCommonParam{30000.0, 144, 0.5, 1.0, 1.5, 0.5, 2.0, 6.0, 0.5, 1, 100};
}

fpa::AstarParam astar_param()
{
  return fpa::AstarParam{"forward", false, true, true, 0.4, 4.0, 2.0, 0.5, 1.7, 1.0};
}

double percentile(std::vector<double> v, double p)
{
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const size_t idx = static_cast<size_t>(p * (v.size() - 1));
  return v[idx];
}

struct RunResult
{
  bool success{false};
  double set_map_ms{0.0};
  double plan_ms{0.0};
  size_t expansions{0};
  double path_length{0.0};
};

template <typename Algo>
RunResult run_once(
  Algo & algo, const nav_msgs::msg::OccupancyGrid & map, const std::array<double, 3> & goal,
  bool measure_set_map)
{
  RunResult r;
  using Clock = std::chrono::steady_clock;
  if (measure_set_map) {
    const auto t0 = Clock::now();
    algo.setMap(map);
    r.set_map_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
  }
  const auto t1 = Clock::now();
  try {
    r.success = algo.makePlan(create_pose_msg(start_pose), create_pose_msg(goal));
  } catch (...) {
    r.success = false;
  }
  r.plan_ms = std::chrono::duration<double, std::milli>(Clock::now() - t1).count();
  if (r.success) {
    r.path_length = algo.getWaypoints().compute_length();
  }
  return r;
}
}  // namespace

int main()
{
  if (!fpa::AstarSearchCuda::isCudaAvailable()) {
    std::cerr << "No CUDA device; aborting bench\n";
    return 1;
  }

  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  const auto p = params();
  const auto ap = astar_param();
  fpa::AstarSearch cpu(p, vehicle_shape, ap, clock);
  fpa::AstarSearchCuda gpu(p, vehicle_shape, ap, clock);

  const auto map = construct_cost_map(150, 150, 0.2, 10);
  const auto map_dense = construct_cost_map(300, 300, 0.1, 20);

  struct Scenario
  {
    std::string name;
    nav_msgs::msg::OccupancyGrid map;
    std::array<double, 3> goal;
    int replans;
  };

  // S3 dense map skipped by default (very expensive); enable with ASTAR_BENCH_FULL=1
  std::vector<Scenario> scenarios = {
    {"S1_easy", map, goal_easy, 1},
    {"S2_hard", map, goal_hard, 1},
    {"S4_replan", map, goal_hard, 3},
  };
  if (std::getenv("ASTAR_BENCH_FULL")) {
    scenarios.insert(scenarios.begin() + 2, {"S3_dense", map_dense, goal_hard, 1});
  }

  std::ofstream csv("/tmp/fpalgos-astar-cuda-bench.csv");
  csv << "backend,scenario,set_map_ms_median,plan_ms_median,plan_ms_p95,expansions,success,path_"
         "length_m\n";

  std::cout << "backend,scenario,set_map_ms_med,plan_ms_med,plan_ms_p95,expansions,success,path_"
               "len\n";

  for (const auto & sc : scenarios) {
    for (const char * backend : {"cpu", "cuda"}) {
      std::vector<double> set_map_samples;
      std::vector<double> plan_samples;
      size_t expansions = 0;
      double path_len = 0.0;
      bool success = true;

      // warmup
      for (int w = 0; w < 1; ++w) {
        if (std::string(backend) == "cpu") {
          run_once(cpu, sc.map, sc.goal, true);
        } else {
          auto r = run_once(gpu, sc.map, sc.goal, true);
          expansions = gpu.last_expansions();
          (void)r;
        }
      }

      // Keep default bench short enough for CI / interactive use (override with ASTAR_BENCH_RUNS)
      const char * runs_env = std::getenv("ASTAR_BENCH_RUNS");
      const int kRuns = runs_env ? std::max(1, std::atoi(runs_env)) : 3;
      for (int i = 0; i < kRuns; ++i) {
        if (std::string(backend) == "cpu") {
          auto r = run_once(cpu, sc.map, sc.goal, true);
          set_map_samples.push_back(r.set_map_ms);
          for (int k = 0; k < sc.replans; ++k) {
            auto pr = run_once(cpu, sc.map, sc.goal, false);
            plan_samples.push_back(pr.plan_ms);
            success = success && pr.success;
            path_len = pr.path_length;
          }
        } else {
          auto r = run_once(gpu, sc.map, sc.goal, true);
          set_map_samples.push_back(r.set_map_ms);
          expansions = gpu.last_expansions();
          for (int k = 0; k < sc.replans; ++k) {
            auto pr = run_once(gpu, sc.map, sc.goal, false);
            plan_samples.push_back(pr.plan_ms);
            success = success && pr.success;
            path_len = pr.path_length;
            expansions = gpu.last_expansions();
          }
        }
      }

      const double set_med = percentile(set_map_samples, 0.5);
      const double plan_med = percentile(plan_samples, 0.5);
      const double plan_p95 = percentile(plan_samples, 0.95);
      std::cout << backend << "," << sc.name << "," << set_med << "," << plan_med << "," << plan_p95
                << "," << expansions << "," << success << "," << path_len << "\n";
      csv << backend << "," << sc.name << "," << set_med << "," << plan_med << "," << plan_p95 << ","
          << expansions << "," << success << "," << path_len << "\n";
    }
  }

  std::cout << "Wrote /tmp/fpalgos-astar-cuda-bench.csv\n";
  return 0;
}
