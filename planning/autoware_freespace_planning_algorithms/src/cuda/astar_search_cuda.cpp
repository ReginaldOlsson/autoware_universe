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

#include "autoware/freespace_planning_algorithms/astar_search_cuda.hpp"

#include "autoware/freespace_planning_algorithms/kinematic_bicycle_model.hpp"
#include "autoware/freespace_planning_algorithms/reeds_shepp.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <tf2/utils.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::freespace_planning_algorithms
{
namespace
{
using Clock = std::chrono::steady_clock;

geometry_msgs::msg::Pose calcRelativePose(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform tf_transform;
  tf2::convert(base_pose, tf_transform);
  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_transform.inverse());
  geometry_msgs::msg::PoseStamped transformed;
  geometry_msgs::msg::PoseStamped pose_orig;
  pose_orig.pose = pose;
  tf2::doTransform(pose_orig, transformed, transform);
  return transformed.pose;
}
}  // namespace

AstarSearchCuda::AstarSearchCuda(
  const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
  const AstarParam & astar_param, const rclcpp::Clock::SharedPtr & clock)
: AbstractPlanningAlgorithm(planner_common_param, clock, collision_vehicle_shape),
  astar_param_(astar_param)
{
  if (!cuda_device_available()) {
    throw std::runtime_error("AstarSearchCuda requires a CUDA-capable device");
  }

  steering_resolution_ =
    collision_vehicle_shape_.max_steering / planner_common_param_.turning_steps;
  heading_resolution_ = 2.0 * M_PI / planner_common_param_.theta_size;
  const double avg_steering =
    steering_resolution_ + (collision_vehicle_shape_.max_steering - steering_resolution_) / 2.0;
  avg_turning_radius_ =
    kinematic_bicycle_model::getTurningRadius(collision_vehicle_shape_.base_length, avg_steering);
  is_backward_search_ = astar_param_.search_method == "backward";
  min_expansion_dist_ = astar_param_.expansion_distance;
  max_expansion_dist_ = collision_vehicle_shape_.base_length * base_length_max_expansion_factor_;
  near_goal_dist_ =
    std::max(astar_param.near_goal_distance, planner_common_param.longitudinal_goal_range);
}

AstarSearchCuda::~AstarSearchCuda()
{
  cuda_free_buffers(buffers_);
}

void AstarSearchCuda::setMap(const nav_msgs::msg::OccupancyGrid & costmap)
{
  const auto t0 = Clock::now();
  AbstractPlanningAlgorithm::setMap(costmap);

  min_expansion_dist_ = std::max(astar_param_.expansion_distance, 1.5 * costmap_.info.resolution);
  max_expansion_dist_ = std::max(
    collision_vehicle_shape_.base_length * base_length_max_expansion_factor_, min_expansion_dist_);

  const size_t grid_size = costmap_.data.size();
  is_obstacle_u8_.resize(grid_size);
  for (size_t i = 0; i < grid_size; ++i) {
    is_obstacle_u8_[i] = is_obstacle_table_[i] ? 1 : 0;
  }

  edt_gpu_.resize(grid_size);
  for (size_t i = 0; i < grid_size; ++i) {
    edt_gpu_[i].distance = static_cast<float>(edt_map_[i].distance);
    edt_gpu_[i].angle = static_cast<float>(edt_map_[i].angle);
  }

  // Optionally recompute EDT on GPU for parity / timing (overwrite host EDT copy).
  cuda_compute_edt(
    is_obstacle_u8_.data(), static_cast<int>(costmap_.info.width),
    static_cast<int>(costmap_.info.height), static_cast<float>(costmap_.info.resolution),
    edt_gpu_.data());
  for (size_t i = 0; i < grid_size; ++i) {
    edt_map_[i].distance = edt_gpu_[i].distance;
    edt_map_[i].angle = edt_gpu_[i].angle;
  }

  max_coll_indexes_ = 0;
  for (const auto & indexes : coll_indexes_table_) {
    max_coll_indexes_ = std::max(max_coll_indexes_, static_cast<int>(indexes.size()));
  }
  const int theta_size = planner_common_param_.theta_size;
  coll_indexes_flat_.assign(static_cast<size_t>(theta_size * max_coll_indexes_), GpuIndexXY{0, 0});
  coll_counts_.assign(theta_size, 0);
  for (int t = 0; t < theta_size; ++t) {
    coll_counts_[t] = static_cast<int>(coll_indexes_table_[t].size());
    for (int i = 0; i < coll_counts_[t]; ++i) {
      const auto & idx = coll_indexes_table_[t][i];
      coll_indexes_flat_[t * max_coll_indexes_ + i] = GpuIndexXY{idx.x, idx.y};
    }
  }

  const size_t graph_size = grid_size * static_cast<size_t>(theta_size);
  const int turn_span = 2 * planner_common_param_.turning_steps + 1;
  // Batch up to 64 open nodes per wavefront kernel
  constexpr int kMaxBatchNodes = 64;
  const size_t max_successors = static_cast<size_t>(turn_span * 2 * kMaxBatchNodes);

  cuda_allocate_buffers(
    buffers_, grid_size, graph_size,
    static_cast<size_t>(theta_size * max_coll_indexes_), static_cast<size_t>(theta_size),
    max_successors);

  // Fix coll_counts allocation inside upload
  cuda_upload_map(
    buffers_, is_obstacle_u8_.data(), edt_gpu_.data(), grid_size, coll_indexes_flat_.data(),
    coll_counts_.data(), static_cast<size_t>(theta_size * max_coll_indexes_),
    static_cast<size_t>(theta_size));

  last_set_map_ms_ =
    std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

void AstarSearchCuda::resetData()
{
  openlist_ = {};
  const size_t grid_size = costmap_.info.width * costmap_.info.height;
  const size_t graph_size = grid_size * static_cast<size_t>(planner_common_param_.theta_size);
  graph_.assign(graph_size, GpuAstarNode{});
  col_free_distance_map_.assign(grid_size, std::numeric_limits<float>::max());
  cuda_reset_graph(buffers_);
}

GpuAstarParam AstarSearchCuda::makeGpuParam() const
{
  GpuAstarParam p{};
  p.distance_heuristic_weight = static_cast<float>(astar_param_.distance_heuristic_weight);
  p.smoothness_weight = static_cast<float>(astar_param_.smoothness_weight);
  p.obstacle_distance_weight = static_cast<float>(astar_param_.obstacle_distance_weight);
  p.goal_lat_distance_weight = static_cast<float>(astar_param_.goal_lat_distance_weight);
  p.expansion_distance = static_cast<float>(astar_param_.expansion_distance);
  p.near_goal_distance = static_cast<float>(near_goal_dist_);
  p.curve_weight = static_cast<float>(planner_common_param_.curve_weight);
  p.reverse_weight = static_cast<float>(planner_common_param_.reverse_weight);
  p.direction_change_weight = static_cast<float>(planner_common_param_.direction_change_weight);
  p.avg_turning_radius = static_cast<float>(avg_turning_radius_);
  p.steering_resolution = static_cast<float>(steering_resolution_);
  p.min_expansion_dist = static_cast<float>(min_expansion_dist_);
  p.max_expansion_dist = static_cast<float>(max_expansion_dist_);
  p.lateral_goal_range = static_cast<float>(planner_common_param_.lateral_goal_range);
  p.longitudinal_goal_range = static_cast<float>(planner_common_param_.longitudinal_goal_range);
  p.angle_goal_range_rad =
    static_cast<float>(autoware_utils_math::deg2rad(planner_common_param_.angle_goal_range / 2.0));
  p.cost_free_obs_dist = static_cast<float>(cost_free_obs_dist_);
  p.resolution = static_cast<float>(costmap_.info.resolution);
  p.width = static_cast<int>(costmap_.info.width);
  p.height = static_cast<int>(costmap_.info.height);
  p.theta_size = planner_common_param_.theta_size;
  p.turning_steps = planner_common_param_.turning_steps;
  p.max_coll_indexes = max_coll_indexes_;
  p.use_back = astar_param_.use_back ? 1 : 0;
  p.adapt_expansion_distance = astar_param_.adapt_expansion_distance ? 1 : 0;
  p.only_behind_solutions = astar_param_.only_behind_solutions ? 1 : 0;
  p.is_backward_search = is_backward_search_ ? 1 : 0;
  p.use_reeds_shepp = 1;
  return p;
}

GpuVehicleShape AstarSearchCuda::makeGpuVehicle() const
{
  GpuVehicleShape v{};
  v.length = static_cast<float>(collision_vehicle_shape_.length);
  v.width = static_cast<float>(collision_vehicle_shape_.width);
  v.base_length = static_cast<float>(collision_vehicle_shape_.base_length);
  v.max_steering = static_cast<float>(collision_vehicle_shape_.max_steering);
  v.base2back = static_cast<float>(collision_vehicle_shape_.base2back);
  v.min_dimension = static_cast<float>(collision_vehicle_shape_.min_dimension);
  v.max_dimension = static_cast<float>(collision_vehicle_shape_.max_dimension);
  return v;
}

void AstarSearchCuda::setCollisionFreeDistanceMap()
{
  using Entry = std::pair<IndexXY, float>;
  struct CompareEntry
  {
    bool operator()(const Entry & a, const Entry & b) const { return a.second > b.second; }
  };
  std::priority_queue<Entry, std::vector<Entry>, CompareEntry> heap;
  std::vector<uint8_t> closed(col_free_distance_map_.size(), 0);
  auto goal_index = pose2index(costmap_, goal_pose_, planner_common_param_.theta_size);
  col_free_distance_map_[indexToId(goal_index)] = 0.0f;
  heap.push({IndexXY{goal_index.x, goal_index.y}, 0.0f});

  const std::array<int, 3> offsets = {1, 0, -1};
  while (!heap.empty()) {
    const auto current = heap.top();
    heap.pop();
    const int id = indexToId(current.first);
    if (closed[id]) {
      continue;
    }
    closed[id] = 1;
    for (const auto offset_x : offsets) {
      for (const auto offset_y : offsets) {
        const IndexXY n_index{current.first.x + offset_x, current.first.y + offset_y};
        const double offset = std::abs(offset_x) + std::abs(offset_y);
        if (isOutOfRange(n_index) || isObs(n_index) || offset < 1) {
          continue;
        }
        if (getObstacleEDT(n_index).distance < 0.5 * collision_vehicle_shape_.width) {
          continue;
        }
        const int n_id = indexToId(n_index);
        const float dist =
          current.second + static_cast<float>(std::sqrt(offset) * costmap_.info.resolution);
        if (closed[n_id] || col_free_distance_map_[n_id] < dist) {
          continue;
        }
        col_free_distance_map_[n_id] = dist;
        heap.push({n_index, dist});
      }
    }
  }
  cuda_upload_col_free_distance(buffers_, col_free_distance_map_.data(), col_free_distance_map_.size());
}

void AstarSearchCuda::setStartNode()
{
  const auto index = pose2index(costmap_, start_pose_, planner_common_param_.theta_size);
  const int key = indexToId(index) * planner_common_param_.theta_size + index.theta;
  GpuAstarNode node{};
  node.x = static_cast<float>(start_pose_.position.x);
  node.y = static_cast<float>(start_pose_.position.y);
  node.theta = static_cast<float>(tf2::getYaw(start_pose_.orientation));
  node.gc = 0.0f;
  const float cf = col_free_distance_map_[indexToId(index)];
  const ReedsSheppStateSpace space(avg_turning_radius_);
  const float rs = static_cast<float>(space.distance(
    {start_pose_.position.x, start_pose_.position.y, tf2::getYaw(start_pose_.orientation)},
    {goal_pose_.position.x, goal_pose_.position.y, tf2::getYaw(goal_pose_.orientation)}));
  node.fc = static_cast<float>(astar_param_.distance_heuristic_weight) * std::max(cf, rs);
  node.dir_distance = 0.0f;
  node.dist_to_goal = static_cast<float>(autoware_utils_geometry::calc_distance2d(start_pose_, goal_pose_));
  node.dist_to_obs = static_cast<float>(getObstacleEDT(index).distance);
  node.steering_index = 0;
  node.parent_key = -1;
  node.status = static_cast<uint8_t>(GpuNodeStatus::Open);
  node.is_back = 0;
  graph_[key] = node;
  openlist_.push({key, node.fc});
}

bool AstarSearchCuda::makePlan(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose)
{
  const auto t0 = Clock::now();
  resetData();
  start_pose_ = global2local(costmap_, start_pose);
  goal_pose_ = global2local(costmap_, goal_pose);
  if (detectCollision(start_pose_) || detectCollision(goal_pose_)) {
    throw std::logic_error("Invalid start or goal pose");
  }
  if (is_backward_search_) {
    std::swap(start_pose_, goal_pose_);
  }
  setCollisionFreeDistanceMap();
  setStartNode();
  const bool ok = search();
  last_plan_ms_ = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
  if (!ok) {
    throw std::logic_error("CUDA HA* failed to find path to goal");
  }
  return true;
}

bool AstarSearchCuda::makePlan(
  const geometry_msgs::msg::Pose & start_pose,
  const std::vector<geometry_msgs::msg::Pose> & goal_candidates)
{
  if (goal_candidates.empty()) {
    return false;
  }
  return makePlan(start_pose, goal_candidates.front());
}

bool AstarSearchCuda::isGoal(const GpuAstarNode & node) const
{
  const double lateral_goal_range = planner_common_param_.lateral_goal_range / 2.0;
  const double longitudinal_goal_range = planner_common_param_.longitudinal_goal_range / 2.0;
  const double goal_angle =
    autoware_utils_math::deg2rad(planner_common_param_.angle_goal_range / 2.0);

  geometry_msgs::msg::Pose node_pose;
  node_pose.position.x = node.x;
  node_pose.position.y = node.y;
  node_pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(node.theta);

  const auto node_index = pose2index(costmap_, node_pose, planner_common_param_.theta_size);
  const auto goal_index = pose2index(costmap_, goal_pose_, planner_common_param_.theta_size);
  if (node_index == goal_index) {
    return true;
  }

  const auto relative_pose = calcRelativePose(goal_pose_, node_pose);
  const bool is_behind_goal = relative_pose.position.x <= 0.0;
  if (astar_param_.only_behind_solutions && !is_behind_goal) {
    return false;
  }
  if (
    std::fabs(relative_pose.position.x) > longitudinal_goal_range ||
    std::fabs(relative_pose.position.y) > lateral_goal_range) {
    return false;
  }
  const auto angle_diff =
    autoware_utils_math::normalize_radian(tf2::getYaw(relative_pose.orientation));
  return std::abs(angle_diff) <= goal_angle;
}

void AstarSearchCuda::setPath(int goal_key)
{
  std_msgs::msg::Header header;
  header.stamp = clock_->now();
  header.frame_id = costmap_.header.frame_id;

  std::vector<PlannerWaypoint> waypoints;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;

  int key = goal_key;
  while (key >= 0) {
    const auto & node = graph_[key];
    pose.pose.position.x = node.x;
    pose.pose.position.y = node.y;
    pose.pose.position.z = goal_pose_.position.z;
    pose.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(node.theta);
    pose.pose = local2global(costmap_, pose.pose);
    waypoints.push_back({pose, node.is_back != 0});
    key = node.parent_key;
  }
  if (!is_backward_search_) {
    std::reverse(waypoints.begin(), waypoints.end());
  }
  waypoints_.header = header;
  waypoints_.waypoints = waypoints;
}

bool AstarSearchCuda::search()
{
  const auto begin = Clock::now();
  last_expansions_ = 0;
  const auto gpu_param = makeGpuParam();
  const auto gpu_vehicle = makeGpuVehicle();
  const GpuPose goal{
    static_cast<float>(goal_pose_.position.x), static_cast<float>(goal_pose_.position.y),
    static_cast<float>(tf2::getYaw(goal_pose_.orientation))};

  std::vector<GpuSuccessor> successors(buffers_.max_successors);
  constexpr int kBatchSize = 64;
  std::vector<GpuAstarNode> batch_nodes;
  std::vector<int> batch_keys;
  batch_nodes.reserve(kBatchSize);
  batch_keys.reserve(kBatchSize);

  while (!openlist_.empty()) {
    const double msec =
      std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    if (msec > planner_common_param_.time_limit) {
      return false;
    }

    batch_nodes.clear();
    batch_keys.clear();
    while (!openlist_.empty() && static_cast<int>(batch_nodes.size()) < kBatchSize) {
      const auto current_entry = openlist_.top();
      openlist_.pop();
      auto & current = graph_[current_entry.key];
      if (current.status == static_cast<uint8_t>(GpuNodeStatus::Closed)) {
        continue;
      }
      current.status = static_cast<uint8_t>(GpuNodeStatus::Closed);
      ++last_expansions_;
      if (isGoal(current)) {
        setPath(current_entry.key);
        return true;
      }
      batch_nodes.push_back(current);
      batch_keys.push_back(current_entry.key);
    }
    if (batch_nodes.empty()) {
      continue;
    }

    int num_succ = 0;
    cuda_expand_nodes_batch(
      buffers_, gpu_param, gpu_vehicle, goal, batch_nodes.data(), batch_keys.data(),
      static_cast<int>(batch_nodes.size()), successors.data(), num_succ);

    for (int i = 0; i < num_succ; ++i) {
      const auto & s = successors[i];
      auto & next = graph_[s.key];
      if (next.status == static_cast<uint8_t>(GpuNodeStatus::Closed)) {
        continue;
      }
      if (next.status == static_cast<uint8_t>(GpuNodeStatus::None) || next.fc > s.fc) {
        next.x = s.x;
        next.y = s.y;
        next.theta = s.theta;
        next.gc = s.gc;
        next.fc = s.fc;
        next.dir_distance = s.dir_distance;
        next.dist_to_goal = s.dist_to_goal;
        next.dist_to_obs = s.dist_to_obs;
        next.steering_index = s.steering_index;
        next.parent_key = s.parent_key;
        next.is_back = s.is_back;
        next.status = static_cast<uint8_t>(GpuNodeStatus::Open);
        openlist_.push({s.key, s.fc});
      }
    }
  }
  return false;
}

}  // namespace autoware::freespace_planning_algorithms
