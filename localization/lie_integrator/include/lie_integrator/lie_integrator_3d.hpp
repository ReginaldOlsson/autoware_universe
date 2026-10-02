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

#ifndef LIE_INTEGRATOR__LIE_INTEGRATOR_3D_HPP_
#define LIE_INTEGRATOR__LIE_INTEGRATOR_3D_HPP_

#include <Eigen/Dense>

namespace lie_integrator
{

/**
 * @brief SE(3) kinematic integrator with geometric "gravity" (terrain contact).
 *
 * Gravity is enforced as a constraint: orientation is rebuilt so body Z aligns
 * with the terrain normal, and height is snapped to the surface. Body twist is
 * then integrated with the SE(3) exponential map.
 *
 * Two input modes:
 * - Ackermann: step(v, phi) with fixed internal dt
 * - IMU: stepImu(gyro, accel, dt) strapdown dead-reckoning on the terrain
 */
class LieIntegrator3D
{
public:
  LieIntegrator3D();
  LieIntegrator3D(double wheelbase, double h);

  /// One Ackermann integration step from rear-wheel speed [m/s] and steer [rad].
  Eigen::Matrix4d step(double v_k, double phi_k);

  /**
   * @brief One IMU integration step.
   * @param gyro  Body angular velocity ω [rad/s] (IMU angular_velocity)
   * @param accel Body specific force [m/s^2] (IMU linear_acceleration, includes gravity)
   * @param dt    Timestep [s]
   */
  Eigen::Matrix4d stepImu(
    const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt);

  void setGravity(double gravity) { gravity_ = gravity; }
  void setPose(const Eigen::Matrix4d & pose);
  void setBodyVelocity(const Eigen::Vector3d & v_body) { v_body_ = v_body; }

  const Eigen::Matrix4d & getPose() const { return current_pose_; }
  const Eigen::Vector3d & getBodyVelocity() const { return v_body_; }
  double getYaw() const { return yaw_; }

private:
  /// Align body Z to terrain normal, preserve heading, snap height.
  void applyTerrainConstraint();

  Eigen::Matrix4d current_pose_;
  Eigen::Vector3d v_body_;  // body-frame linear velocity [m/s]
  double L_;                // wheelbase [m] (Ackermann mode)
  double h_;                // fixed timestep [s] (Ackermann mode)
  double yaw_;              // heading used to rebuild R after terrain alignment
  double gravity_;          // |g| [m/s^2]
};

}  // namespace lie_integrator

#endif  // LIE_INTEGRATOR__LIE_INTEGRATOR_3D_HPP_
