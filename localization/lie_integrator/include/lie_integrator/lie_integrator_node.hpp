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

#ifndef LIE_INTEGRATOR__LIE_INTEGRATOR_NODE_HPP_
#define LIE_INTEGRATOR__LIE_INTEGRATOR_NODE_HPP_

#include "lie_integrator/lie_integrator_3d.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <optional>
#include <string>

namespace lie_integrator
{

class LieIntegratorNode : public rclcpp::Node
{
public:
  explicit LieIntegratorNode(const rclcpp::NodeOptions & options);

private:
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  geometry_msgs::msg::PoseStamped poseToMsg(
    const Eigen::Matrix4d & pose, const rclcpp::Time & stamp) const;
  nav_msgs::msg::Odometry odomToMsg(
    const Eigen::Matrix4d & pose, const Eigen::Vector3d & v_body, double yaw_rate,
    const rclcpp::Time & stamp) const;

  LieIntegrator3D integrator_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

  std::string map_frame_;
  std::string base_frame_;
  double max_dt_;
  bool publish_odometry_;

  std::optional<rclcpp::Time> last_stamp_;
};

}  // namespace lie_integrator

#endif  // LIE_INTEGRATOR__LIE_INTEGRATOR_NODE_HPP_
