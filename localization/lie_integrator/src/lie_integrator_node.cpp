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

#include "lie_integrator/lie_integrator_node.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <functional>
#include <memory>

namespace lie_integrator
{

LieIntegratorNode::LieIntegratorNode(const rclcpp::NodeOptions & options)
: Node("lie_integrator", options), integrator_()
{
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  max_dt_ = declare_parameter<double>("max_dt", 0.1);
  publish_odometry_ = declare_parameter<bool>("publish_odometry", true);
  const double gravity = declare_parameter<double>("gravity", 9.80665);

  const double init_x = declare_parameter<double>("initial_pose.x", 0.0);
  const double init_y = declare_parameter<double>("initial_pose.y", 0.0);
  const double init_z = declare_parameter<double>("initial_pose.z", 0.0);
  const double init_yaw = declare_parameter<double>("initial_pose.yaw", 0.0);

  integrator_.setGravity(gravity);

  Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
  init_pose(0, 3) = init_x;
  init_pose(1, 3) = init_y;
  init_pose(2, 3) = init_z;
  const Eigen::AngleAxisd yaw_aa(init_yaw, Eigen::Vector3d::UnitZ());
  init_pose.block<3, 3>(0, 0) = yaw_aa.toRotationMatrix();
  integrator_.setPose(init_pose);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "~/input/imu", rclcpp::SensorDataQoS(),
    std::bind(&LieIntegratorNode::onImu, this, std::placeholders::_1));

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/output/pose", rclcpp::QoS{10});
  twist_pub_ =
    create_publisher<geometry_msgs::msg::TwistStamped>("~/output/twist", rclcpp::QoS{10});
  if (publish_odometry_) {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/output/odometry", rclcpp::QoS{10});
  }

  RCLCPP_INFO(
    get_logger(),
    "lie_integrator node started (map_frame=%s, base_frame=%s). Waiting for IMU on ~/input/imu",
    map_frame_.c_str(), base_frame_.c_str());
}

void LieIntegratorNode::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  if (!last_stamp_.has_value()) {
    last_stamp_ = stamp;
    return;
  }

  double dt = (stamp - *last_stamp_).seconds();
  last_stamp_ = stamp;

  if (dt <= 0.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Non-positive IMU dt (%.6f), skip", dt);
    return;
  }
  if (dt > max_dt_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "IMU dt %.3f > max_dt %.3f, clamping", dt, max_dt_);
    dt = max_dt_;
  }

  const Eigen::Vector3d gyro(
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  const Eigen::Vector3d accel(
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

  const Eigen::Matrix4d pose = integrator_.stepImu(gyro, accel, dt);
  const Eigen::Vector3d & v_body = integrator_.getBodyVelocity();

  pose_pub_->publish(poseToMsg(pose, stamp));

  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = stamp;
  twist_msg.header.frame_id = base_frame_;
  twist_msg.twist.linear.x = v_body.x();
  twist_msg.twist.linear.y = v_body.y();
  twist_msg.twist.linear.z = v_body.z();
  twist_msg.twist.angular.x = gyro.x();
  twist_msg.twist.angular.y = gyro.y();
  twist_msg.twist.angular.z = gyro.z();
  twist_pub_->publish(twist_msg);

  if (odom_pub_) {
    odom_pub_->publish(odomToMsg(pose, v_body, gyro.z(), stamp));
  }
}

geometry_msgs::msg::PoseStamped LieIntegratorNode::poseToMsg(
  const Eigen::Matrix4d & pose, const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = map_frame_;

  msg.pose.position.x = pose(0, 3);
  msg.pose.position.y = pose(1, 3);
  msg.pose.position.z = pose(2, 3);

  const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  return msg;
}

nav_msgs::msg::Odometry LieIntegratorNode::odomToMsg(
  const Eigen::Matrix4d & pose, const Eigen::Vector3d & v_body, double yaw_rate,
  const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = map_frame_;
  msg.child_frame_id = base_frame_;

  msg.pose.pose.position.x = pose(0, 3);
  msg.pose.pose.position.y = pose(1, 3);
  msg.pose.pose.position.z = pose(2, 3);

  const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  msg.twist.twist.linear.x = v_body.x();
  msg.twist.twist.linear.y = v_body.y();
  msg.twist.twist.linear.z = v_body.z();
  msg.twist.twist.angular.z = yaw_rate;
  return msg;
}

}  // namespace lie_integrator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lie_integrator::LieIntegratorNode)
