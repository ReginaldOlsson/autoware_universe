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

#include "lie_integrator/lie_integrator_3d.hpp"

#include "lie_integrator/lie_algebra.hpp"
#include "lie_integrator/terrain.hpp"

#include <cmath>

namespace lie_integrator
{
namespace
{
constexpr double kDegenerateEps = 1e-9;
}  // namespace

LieIntegrator3D::LieIntegrator3D()
: LieIntegrator3D(2.8, 0.1)
{
}

LieIntegrator3D::LieIntegrator3D(double wheelbase, double h)
: v_body_(Eigen::Vector3d::Zero()),
  L_(wheelbase),
  h_(h),
  yaw_(0.0),
  gravity_(9.80665)
{
  current_pose_ = Eigen::Matrix4d::Identity();
  current_pose_(2, 3) = getTerrainHeight(0.0, 0.0);
}

void LieIntegrator3D::setPose(const Eigen::Matrix4d & pose)
{
  current_pose_ = pose;
  // Extract yaw from current rotation (atan2 of forward projected to XY)
  const Eigen::Vector3d forward = current_pose_.block<3, 1>(0, 0);
  yaw_ = std::atan2(forward.y(), forward.x());
  applyTerrainConstraint();
}

void LieIntegrator3D::applyTerrainConstraint()
{
  const double x = current_pose_(0, 3);
  const double y = current_pose_(1, 3);
  const Eigen::Vector3d normal = getTerrainNormal(x, y);
  const double height = getTerrainHeight(x, y);

  Eigen::Vector3d forward_yaw(std::cos(yaw_), std::sin(yaw_), 0.0);
  Eigen::Vector3d forward_hill = forward_yaw - (forward_yaw.dot(normal)) * normal;
  if (forward_hill.norm() < kDegenerateEps) {
    Eigen::Vector3d fallback =
      (std::abs(normal.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    forward_hill = fallback - (fallback.dot(normal)) * normal;
  }
  forward_hill.normalize();

  // Body Y (left): Z × X
  const Eigen::Vector3d left_hill = normal.cross(forward_hill);

  Eigen::Matrix3d R_constrained;
  R_constrained.col(0) = forward_hill;
  R_constrained.col(1) = left_hill;
  R_constrained.col(2) = normal;

  current_pose_.block<3, 3>(0, 0) = R_constrained;
  current_pose_(2, 3) = height;

  // Keep body velocity tangent to the ground (no lift-off / sink)
  v_body_.z() = 0.0;
}

Eigen::Matrix4d LieIntegrator3D::step(double v_k, double phi_k)
{
  const double omega_z_flat = (v_k / L_) * std::tan(phi_k);
  yaw_ += h_ * omega_z_flat;

  applyTerrainConstraint();

  v_body_ = Eigen::Vector3d(v_k, 0.0, 0.0);

  Vector6d xi = Vector6d::Zero();
  xi(0) = v_k;
  xi(5) = omega_z_flat;

  current_pose_ = current_pose_ * expSE3(h_ * xi);
  return current_pose_;
}

Eigen::Matrix4d LieIntegrator3D::stepImu(
  const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt)
{
  if (dt <= 0.0) {
    return current_pose_;
  }

  // 1. Integrate heading from body yaw rate (terrain-following vehicle)
  yaw_ += gyro.z() * dt;

  // 2. Gravity constraint: align to terrain, snap height
  applyTerrainConstraint();

  // 3. Remove gravity from specific force (ROS IMU: +g along up when stationary)
  //    After terrain alignment, body Z ≈ surface normal (up).
  const Eigen::Vector3d accel_lin = accel - Eigen::Vector3d(0.0, 0.0, gravity_);

  // 4. Integrate body velocity; keep it in the tangent plane
  v_body_ += accel_lin * dt;
  v_body_.z() = 0.0;

  // 5. SE(3) twist: linear from integrated velocity, angular from gyro
  //    ω_x/ω_y are zeroed after terrain rebuild (attitude is constrained);
  //    only yaw rate drives heading between constraint applications.
  Vector6d xi = Vector6d::Zero();
  xi.head<3>() = v_body_;
  xi(5) = gyro.z();

  // 6. Lie-group integration: g_{k+1} = g_k * exp(dt * ξ)
  current_pose_ = current_pose_ * expSE3(dt * xi);

  // Re-snap after integration so numerical drift does not accumulate in Z / tilt
  applyTerrainConstraint();

  return current_pose_;
}

}  // namespace lie_integrator
