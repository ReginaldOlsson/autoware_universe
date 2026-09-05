#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <functional>
#include <string>
#include <vector>

namespace ackermann_ompl_plugins
{

struct OmplWaypoint
{
  geometry_msgs::msg::Pose pose;
  bool is_back{false};
};

struct OmplSolveRequest
{
  geometry_msgs::msg::Pose start_pose;
  geometry_msgs::msg::Pose goal_pose;
  std::string planner_id{"RRTConnect"};
  double allowed_planning_time{2.0};
  double turning_radius{6.0};
  double longest_valid_segment_fraction{0.05};
  /// Collision-check spacing along Reeds-Shepp motions [m]. When > 0, overrides
  /// longest_valid_segment_fraction using state-space maximum extent.
  double valid_segment_length{-1.0};
  /// Target spacing [m] when densifying the path (preferred over fixed count).
  double interpolate_resolution{0.5};
  int interpolate_count{0};  // 0 => use interpolate_resolution
  bool simplify{true};
  bool try_analytic_reeds_shepp{true};
  double planner_range{8.0};
  double goal_xy_tolerance{1.0};
  double goal_yaw_tolerance{0.2};  // radians (~11 deg)
  // Costmap / world bounds in planning frame (meters)
  double bounds_low_x{0.0};
  double bounds_low_y{0.0};
  double bounds_high_x{100.0};
  double bounds_high_y{100.0};
};

struct OmplSolveResult
{
  bool success{false};
  std::string message;
  std::vector<OmplWaypoint> waypoints;
};

using StateValidityFn = std::function<bool(double x, double y, double yaw)>;

/**
 * @brief Standalone OMPL Ackermann planner used by Autoware AbstractPlanningAlgorithm adapter.
 * Does not require MoveIt; collision checking is provided via StateValidityFn.
 */
class AckermannOmplSolver
{
public:
  OmplSolveResult solve(const OmplSolveRequest & request, const StateValidityFn & is_valid) const;
};

}  // namespace ackermann_ompl_plugins
