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

#ifndef AUTOWARE__MOVEIT_FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
#define AUTOWARE__MOVEIT_FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_

#include "autoware_utils/ros/logger_level_configure.hpp"

#include "autoware/moveit_freespace_planner/utils.hpp"

#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>
#include <autoware/moveit_freespace_planning_algorithms/moveit_ackermann_planner.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/scenario.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::moveit_freespace_planner
{
using autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm;
using autoware::freespace_planning_algorithms::PlannerCommonParam;
using autoware::freespace_planning_algorithms::VehicleShape;
using autoware::moveit_freespace_planning_algorithms::MoveItAckermannPlanner;
using autoware_internal_planning_msgs::msg::Scenario;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;

struct NodeParam
{
  std::string planning_algorithm;
  double waypoints_velocity;
  double update_rate;
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
  double th_course_out_distance_m;
  double th_obstacle_time_sec;
  double vehicle_shape_margin_m;
  bool replan_when_obstacle_found;
  bool replan_when_course_out;
};

class FreespacePlannerNode : public rclcpp::Node
{
public:
  explicit FreespacePlannerNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_pose_array_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_partial_pose_array_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parking_state_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    processing_time_pub_;

  rclcpp::Subscription<LaneletRoute>::SharedPtr route_sub_;

  autoware_utils::InterProcessPollingSubscriber<OccupancyGrid> occupancy_grid_sub_{
    this, "~/input/occupancy_grid"};
  autoware_utils::InterProcessPollingSubscriber<Scenario> scenario_sub_{this, "~/input/scenario"};
  autoware_utils::InterProcessPollingSubscriber<Odometry, autoware_utils::polling_policy::All>
    odom_sub_{this, "~/input/odometry", rclcpp::QoS{100}};

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  NodeParam node_param_;
  VehicleShape vehicle_shape_;

  std::unique_ptr<AbstractPlanningAlgorithm> algo_;
  PoseStamped current_pose_;
  PoseStamped goal_pose_;

  Trajectory trajectory_;
  Trajectory partial_trajectory_;
  std::vector<size_t> reversing_indices_;
  size_t prev_target_index_{0};
  size_t target_index_{0};
  bool is_completed_{false};
  bool reset_in_progress_{false};
  bool is_new_parking_cycle_{true};
  std::optional<rclcpp::Time> obs_found_time_;

  LaneletRoute::ConstSharedPtr route_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_;
  Scenario::ConstSharedPtr scenario_;
  Odometry::ConstSharedPtr odom_;

  std::deque<Odometry::ConstSharedPtr> odom_buffer_;

  PlannerCommonParam getPlannerCommonParam();

  void onRoute(const LaneletRoute::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onTimer();
  void updateData();
  void reset();
  void planTrajectory();
  void initializePlanningAlgorithm();
  bool isDataReady();
  bool isPlanRequired();
  void updateTargetIndex();
  bool checkCurrentTrajectoryCollision();
  TransformStamped getTransform(const std::string & from, const std::string & to);

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;
};
}  // namespace autoware::moveit_freespace_planner

#endif  // AUTOWARE__MOVEIT_FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
