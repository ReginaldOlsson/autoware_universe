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

#ifndef LIE_INTEGRATOR__PURE_IMU_ODOMETRY_HPP_
#define LIE_INTEGRATOR__PURE_IMU_ODOMETRY_HPP_

#include <Eigen/Dense>

#include <deque>
#include <utility>

namespace lie_integrator
{

struct PureImuParams
{
  double gravity{9.80665};
  double init_seconds{2.0};
  double expected_imu_hz{200.0};
  size_t zupt_window{50};
  double zupt_gyro_var_thresh{1e-4};
  double zupt_accel_var_thresh{0.05};
  double nhc_lateral_decay{0.1};
  double nhc_vertical_decay{0.1};
};

struct PureImuState
{
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};  // world
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};  // world
  Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba{Eigen::Vector3d::Zero()};
  bool initialized{false};
  bool zupt_active{false};
};

/**
 * @brief Strapdown IMU odometry with static init, ZUPT, and NHC.
 * Publishes body twist for autoware_ekf_localizer (no Madgwick / no Kalman).
 */
class PureImuOdometry
{
public:
  explicit PureImuOdometry(const PureImuParams & params = PureImuParams());

  /// Feed one IMU sample. Returns true when a usable state update was produced.
  bool process(
    double stamp, const Eigen::Vector3d & accel, const Eigen::Vector3d & gyro, double dt);

  const PureImuState & state() const { return state_; }
  bool isInitialized() const { return state_.initialized; }

  /// Body-frame linear velocity after NHC (for EKF twist).
  Eigen::Vector3d bodyVelocity() const;

  /// Debiased body angular velocity.
  Eigen::Vector3d bodyAngularVelocity(const Eigen::Vector3d & gyro) const;

private:
  void accumulateInit(const Eigen::Vector3d & accel, const Eigen::Vector3d & gyro);
  bool tryFinishInit();
  void pushZuptSample(const Eigen::Vector3d & accel, const Eigen::Vector3d & gyro);
  bool detectZupt() const;
  void applyNhc();

  PureImuParams params_;
  PureImuState state_;

  Eigen::Vector3d init_gyro_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d init_accel_sum_{Eigen::Vector3d::Zero()};
  int init_count_{0};
  int init_needed_{400};

  std::deque<Eigen::Vector3d> zupt_gyro_buf_;
  std::deque<Eigen::Vector3d> zupt_accel_buf_;
};

}  // namespace lie_integrator

#endif  // LIE_INTEGRATOR__PURE_IMU_ODOMETRY_HPP_
