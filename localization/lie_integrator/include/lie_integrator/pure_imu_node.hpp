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

#ifndef LIE_INTEGRATOR__PURE_IMU_NODE_HPP_
#define LIE_INTEGRATOR__PURE_IMU_NODE_HPP_

#include "lie_integrator/pure_imu_odometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <optional>
#include <string>

namespace lie_integrator
{

class PureImuNode : public rclcpp::Node
{
public:
  explicit PureImuNode(const rclcpp::NodeOptions & options);

private:
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  PureImuOdometry odometry_;
  PureImuParams params_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr debug_pose_pub_;

  std::string base_frame_;
  std::string map_frame_;
  bool publish_debug_pose_;
  double max_dt_;
  double twist_vx_cov_;
  double twist_wz_cov_;

  std::optional<rclcpp::Time> last_stamp_;
  bool init_logged_{false};
};

}  // namespace lie_integrator

#endif  // LIE_INTEGRATOR__PURE_IMU_NODE_HPP_
