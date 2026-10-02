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

#include "lie_integrator/pure_imu_node.hpp"

#include <Eigen/Geometry>

#include <array>
#include <functional>

namespace lie_integrator
{

PureImuNode::PureImuNode(const rclcpp::NodeOptions & options)
: Node("pure_imu_odometry", options)
{
  params_.gravity = declare_parameter<double>("gravity", 9.80665);
  params_.init_seconds = declare_parameter<double>("init_seconds", 2.0);
  params_.expected_imu_hz = declare_parameter<double>("expected_imu_hz", 200.0);
  params_.zupt_window = static_cast<size_t>(declare_parameter<int>("zupt_window", 50));
  params_.zupt_gyro_var_thresh = declare_parameter<double>("zupt_gyro_var_thresh", 1e-4);
  params_.zupt_accel_var_thresh = declare_parameter<double>("zupt_accel_var_thresh", 0.05);
  params_.nhc_lateral_decay = declare_parameter<double>("nhc_lateral_decay", 0.1);
  params_.nhc_vertical_decay = declare_parameter<double>("nhc_vertical_decay", 0.1);

  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  publish_debug_pose_ = declare_parameter<bool>("publish_debug_pose", true);
  max_dt_ = declare_parameter<double>("max_dt", 0.05);
  twist_vx_cov_ = declare_parameter<double>("twist_vx_covariance", 0.1);
  twist_wz_cov_ = declare_parameter<double>("twist_wz_covariance", 0.05);

  odometry_ = PureImuOdometry(params_);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "~/input/imu", rclcpp::SensorDataQoS(),
    std::bind(&PureImuNode::onImu, this, std::placeholders::_1));

  twist_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "~/output/twist_with_covariance", rclcpp::QoS{10});

  if (publish_debug_pose_) {
    debug_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/output/debug_pose_with_covariance", rclcpp::QoS{10});
  }

  RCLCPP_INFO(
    get_logger(),
    "pure_imu_odometry ready (init=%.1fs). Feed twist to autoware_ekf_localizer; GNSS → EKF pose.",
    params_.init_seconds);
}

void PureImuNode::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  const Eigen::Vector3d accel(
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d gyro(
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  double dt = 1.0 / params_.expected_imu_hz;
  if (last_stamp_.has_value()) {
    dt = (stamp - *last_stamp_).seconds();
  }
  last_stamp_ = stamp;

  if (dt <= 0.0) {
    return;
  }
  if (dt > max_dt_) {
    dt = max_dt_;
  }

  const bool updated = odometry_.process(stamp.seconds(), accel, gyro, dt);
  if (!odometry_.isInitialized()) {
    return;
  }
  if (!init_logged_) {
    RCLCPP_INFO(get_logger(), "IMU static initialization complete.");
    init_logged_ = true;
  }
  if (!updated && !odometry_.state().initialized) {
    return;
  }

  const Eigen::Vector3d v_body = odometry_.bodyVelocity();
  const Eigen::Vector3d w_body = odometry_.bodyAngularVelocity(gyro);

  geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.stamp = stamp;
  twist_msg.header.frame_id = base_frame_;
  twist_msg.twist.twist.linear.x = v_body.x();
  twist_msg.twist.twist.linear.y = v_body.y();
  twist_msg.twist.twist.linear.z = v_body.z();
  twist_msg.twist.twist.angular.x = w_body.x();
  twist_msg.twist.twist.angular.y = w_body.y();
  twist_msg.twist.twist.angular.z = w_body.z();
  twist_msg.twist.covariance.fill(0.0);
  twist_msg.twist.covariance[0] = twist_vx_cov_;   // vx
  twist_msg.twist.covariance[7] = twist_vx_cov_;   // vy
  twist_msg.twist.covariance[14] = twist_vx_cov_;  // vz
  twist_msg.twist.covariance[35] = twist_wz_cov_;  // wz
  twist_pub_->publish(twist_msg);

  if (debug_pose_pub_) {
    const auto & s = odometry_.state();
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.pose.position.x = s.p.x();
    pose_msg.pose.pose.position.y = s.p.y();
    pose_msg.pose.pose.position.z = s.p.z();
    const Eigen::Quaterniond q(s.R);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();
    pose_msg.pose.covariance.fill(0.0);
    pose_msg.pose.covariance[0] = 1.0;
    pose_msg.pose.covariance[7] = 1.0;
    pose_msg.pose.covariance[14] = 1.0;
    pose_msg.pose.covariance[35] = 0.5;
    debug_pose_pub_->publish(pose_msg);
  }
}

}  // namespace lie_integrator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lie_integrator::PureImuNode)
