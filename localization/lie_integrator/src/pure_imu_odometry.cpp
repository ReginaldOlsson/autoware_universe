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

#include "lie_integrator/pure_imu_odometry.hpp"

#include "lie_integrator/lie_algebra.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace lie_integrator
{
namespace
{
Eigen::Vector3d variance3(const std::deque<Eigen::Vector3d> & buf)
{
  if (buf.size() < 2) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & v : buf) {
    mean += v;
  }
  mean /= static_cast<double>(buf.size());

  Eigen::Vector3d var = Eigen::Vector3d::Zero();
  for (const auto & v : buf) {
    const Eigen::Vector3d d = v - mean;
    var += d.cwiseProduct(d);
  }
  var /= static_cast<double>(buf.size() - 1);
  return var;
}
}  // namespace

PureImuOdometry::PureImuOdometry(const PureImuParams & params) : params_(params)
{
  init_needed_ = std::max(
    1, static_cast<int>(std::lround(params_.init_seconds * params_.expected_imu_hz)));
}

bool PureImuOdometry::process(
  double /*stamp*/, const Eigen::Vector3d & accel, const Eigen::Vector3d & gyro, double dt)
{
  if (!state_.initialized) {
    accumulateInit(accel, gyro);
    return tryFinishInit();
  }

  if (dt <= 0.0) {
    return false;
  }

  pushZuptSample(accel, gyro);
  state_.zupt_active = detectZupt();

  const Eigen::Vector3d w = gyro - state_.bg;
  const Eigen::Vector3d a = accel - state_.ba;

  // Attitude: R <- R * exp(h * w)
  state_.R = state_.R * expSO3(dt * w);

  const Eigen::Vector3d g_world(0.0, 0.0, params_.gravity);
  const Eigen::Vector3d a_world = state_.R * a - g_world;

  if (state_.zupt_active) {
    state_.v.setZero();
  } else {
    state_.v += a_world * dt;
    applyNhc();
  }

  state_.p += state_.v * dt;
  return true;
}

Eigen::Vector3d PureImuOdometry::bodyVelocity() const
{
  return state_.R.transpose() * state_.v;
}

Eigen::Vector3d PureImuOdometry::bodyAngularVelocity(const Eigen::Vector3d & gyro) const
{
  return gyro - state_.bg;
}

void PureImuOdometry::accumulateInit(const Eigen::Vector3d & accel, const Eigen::Vector3d & gyro)
{
  init_gyro_sum_ += gyro;
  init_accel_sum_ += accel;
  ++init_count_;
}

bool PureImuOdometry::tryFinishInit()
{
  if (init_count_ < init_needed_) {
    return false;
  }

  state_.bg = init_gyro_sum_ / static_cast<double>(init_count_);
  const Eigen::Vector3d a_mean = init_accel_sum_ / static_cast<double>(init_count_);

  // Align body Z with measured gravity (upright start). Measured accel ≈ +g in up direction.
  Eigen::Vector3d g_body = a_mean.normalized();
  if (g_body.norm() < 1e-6) {
    g_body = Eigen::Vector3d::UnitZ();
  }

  // Build R such that R * g_body ≈ world Z (up)
  const Eigen::Vector3d z_world = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d axis = g_body.cross(z_world);
  const double axis_norm = axis.norm();
  if (axis_norm < 1e-8) {
    // Already aligned or anti-aligned
    if (g_body.dot(z_world) < 0.0) {
      state_.R = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    } else {
      state_.R = Eigen::Matrix3d::Identity();
    }
  } else {
    axis /= axis_norm;
    const double angle = std::atan2(axis_norm, g_body.dot(z_world));
    state_.R = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
  }

  // Accel bias: residual after removing gravity in body frame
  const Eigen::Vector3d g_in_body = state_.R.transpose() * Eigen::Vector3d(0.0, 0.0, params_.gravity);
  state_.ba = a_mean - g_in_body;

  state_.v.setZero();
  state_.p.setZero();
  state_.initialized = true;
  return true;
}

void PureImuOdometry::pushZuptSample(const Eigen::Vector3d & accel, const Eigen::Vector3d & gyro)
{
  zupt_gyro_buf_.push_back(gyro);
  zupt_accel_buf_.push_back(accel);
  while (zupt_gyro_buf_.size() > params_.zupt_window) {
    zupt_gyro_buf_.pop_front();
  }
  while (zupt_accel_buf_.size() > params_.zupt_window) {
    zupt_accel_buf_.pop_front();
  }
}

bool PureImuOdometry::detectZupt() const
{
  if (zupt_gyro_buf_.size() < params_.zupt_window) {
    return false;
  }
  const Eigen::Vector3d gyro_var = variance3(zupt_gyro_buf_);
  const Eigen::Vector3d accel_var = variance3(zupt_accel_buf_);
  return gyro_var.maxCoeff() < params_.zupt_gyro_var_thresh &&
         accel_var.maxCoeff() < params_.zupt_accel_var_thresh;
}

void PureImuOdometry::applyNhc()
{
  Eigen::Vector3d v_body = state_.R.transpose() * state_.v;
  v_body.y() *= params_.nhc_lateral_decay;
  v_body.z() *= params_.nhc_vertical_decay;
  state_.v = state_.R * v_body;
}

}  // namespace lie_integrator
