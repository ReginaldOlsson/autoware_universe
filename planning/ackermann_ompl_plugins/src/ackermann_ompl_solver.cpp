#include "ackermann_ompl_plugins/ackermann_ompl_solver.hpp"
#include "ackermann_ompl_plugins/ackermann_state_space.hpp"

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/RandomNumbers.h>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ackermann_ompl_plugins
{
namespace
{

geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

class SE2PoseGoal : public ob::GoalSampleableRegion
{
public:
  SE2PoseGoal(
    const ob::SpaceInformationPtr & si, double x, double y, double yaw, double xy_tol,
    double yaw_tol)
  : GoalSampleableRegion(si), x_(x), y_(y), yaw_(yaw), xy_tol_(xy_tol), yaw_tol_(yaw_tol)
  {
    // Satisfied when distanceGoal() == 0 (both xy and yaw within tolerance).
    setThreshold(0.0);
  }

  double distanceGoal(const ob::State * st) const override
  {
    const auto * se2 = st->as<ob::SE2StateSpace::StateType>();
    const double dx = se2->getX() - x_;
    const double dy = se2->getY() - y_;
    const double xy = std::hypot(dx, dy);
    const double dyaw = std::abs(normalizeAngle(se2->getYaw() - yaw_));
    const double xy_err = std::max(0.0, xy - xy_tol_);
    const double yaw_err = std::max(0.0, dyaw - yaw_tol_);
    return xy_err + yaw_err;
  }

  // Required by RRTConnect so it can grow a second tree from the goal.
  void sampleGoal(ob::State * st) const override
  {
    auto * se2 = st->as<ob::SE2StateSpace::StateType>();
    const double r = xy_tol_ * std::sqrt(rng_.uniform01());
    const double ang = rng_.uniformReal(-M_PI, M_PI);
    se2->setX(x_ + r * std::cos(ang));
    se2->setY(y_ + r * std::sin(ang));
    se2->setYaw(normalizeAngle(yaw_ + rng_.uniformReal(-yaw_tol_, yaw_tol_)));
  }

  unsigned int maxSampleCount() const override { return 1000; }

private:
  double x_;
  double y_;
  double yaw_;
  double xy_tol_;
  double yaw_tol_;
  mutable ompl::RNG rng_;
};

void assignReverseFlags(std::vector<OmplWaypoint> & waypoints)
{
  if (waypoints.size() < 2) {
    return;
  }

  // Signed motion relative to heading for each edge.
  std::vector<int> edge_dir(waypoints.size() - 1, 1);
  std::vector<double> edge_len(waypoints.size() - 1, 0.0);
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const double yaw = tf2::getYaw(waypoints[i].pose.orientation);
    const double dx = waypoints[i + 1].pose.position.x - waypoints[i].pose.position.x;
    const double dy = waypoints[i + 1].pose.position.y - waypoints[i].pose.position.y;
    edge_len[i] = std::hypot(dx, dy);
    if (edge_len[i] < 1e-4) {
      edge_dir[i] = (i > 0) ? edge_dir[i - 1] : 1;
      continue;
    }
    const double forward_dot = std::cos(yaw) * dx + std::sin(yaw) * dy;
    edge_dir[i] = (forward_dot >= 0.0) ? 1 : -1;
  }

  // Merge short opposite runs so tiny Reeds-Shepp cusps / interpolation noise
  // do not create fake reverse partials (velocity artifacts).
  constexpr double k_min_segment_length = 0.6;  // [m]
  size_t run_start = 0;
  while (run_start < edge_dir.size()) {
    size_t run_end = run_start + 1;
    while (run_end < edge_dir.size() && edge_dir[run_end] == edge_dir[run_start]) {
      ++run_end;
    }
    double run_length = 0.0;
    for (size_t i = run_start; i < run_end; ++i) {
      run_length += edge_len[i];
    }
    if (run_length < k_min_segment_length && run_start > 0) {
      const int absorb = edge_dir[run_start - 1];
      for (size_t i = run_start; i < run_end; ++i) {
        edge_dir[i] = absorb;
      }
    } else if (run_length < k_min_segment_length && run_end < edge_dir.size()) {
      const int absorb = edge_dir[run_end];
      for (size_t i = run_start; i < run_end; ++i) {
        edge_dir[i] = absorb;
      }
    }
    run_start = run_end;
  }

  // Point i inherits direction of outgoing edge; last point keeps previous.
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    waypoints[i].is_back = edge_dir[i] < 0;
  }
  waypoints.back().is_back = waypoints[waypoints.size() - 2].is_back;
}

std::vector<OmplWaypoint> pathToWaypoints(const og::PathGeometric & path)
{
  std::vector<OmplWaypoint> waypoints;
  waypoints.reserve(path.getStateCount());
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
    const auto * se2 = path.getState(i)->as<ob::SE2StateSpace::StateType>();
    OmplWaypoint wp;
    wp.pose.position.x = se2->getX();
    wp.pose.position.y = se2->getY();
    wp.pose.position.z = 0.0;
    wp.pose.orientation = yawToQuat(se2->getYaw());
    waypoints.push_back(wp);
  }
  assignReverseFlags(waypoints);
  return waypoints;
}

void densifyPath(og::PathGeometric & path, const OmplSolveRequest & request)
{
  if (request.interpolate_count > 1) {
    path.interpolate(static_cast<unsigned int>(request.interpolate_count));
    return;
  }
  if (request.interpolate_resolution <= 0.0 || path.getStateCount() < 2) {
    return;
  }
  const double length = path.length();
  if (length <= request.interpolate_resolution) {
    return;
  }
  const unsigned int count =
    static_cast<unsigned int>(std::ceil(length / request.interpolate_resolution)) + 1;
  path.interpolate(std::max(2u, count));
}

}  // namespace

OmplSolveResult AckermannOmplSolver::solve(
  const OmplSolveRequest & request, const StateValidityFn & is_valid) const
{
  OmplSolveResult result;

  if (!is_valid) {
    result.message = "State validity callback is empty";
    return result;
  }

  auto space = std::make_shared<AckermannStateSpace>(request.turning_radius);
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, request.bounds_low_x);
  bounds.setLow(1, request.bounds_low_y);
  bounds.setHigh(0, request.bounds_high_x);
  bounds.setHigh(1, request.bounds_high_y);
  space->setBounds(bounds);

  og::SimpleSetup ss(space);
  auto si = ss.getSpaceInformation();
  si->setStateValidityChecker([is_valid](const ob::State * state) {
    const auto * se2 = state->as<ob::SE2StateSpace::StateType>();
    return is_valid(se2->getX(), se2->getY(), se2->getYaw());
  });
  // Coarser than 0.01: Reeds-Shepp motion checks are otherwise very expensive.
  si->setStateValidityCheckingResolution(
    std::clamp(request.longest_valid_segment_fraction, 0.01, 0.2));
  si->setup();

  const double start_yaw = tf2::getYaw(request.start_pose.orientation);
  const double goal_yaw = tf2::getYaw(request.goal_pose.orientation);

  ob::ScopedState<> start(space);
  start[0] = request.start_pose.position.x;
  start[1] = request.start_pose.position.y;
  start[2] = start_yaw;

  ob::ScopedState<> goal(space);
  goal[0] = request.goal_pose.position.x;
  goal[1] = request.goal_pose.position.y;
  goal[2] = goal_yaw;

  if (!si->isValid(start.get())) {
    result.message = "Start state is in collision or out of bounds";
    return result;
  }
  if (!si->isValid(goal.get())) {
    result.message = "Goal state is in collision or out of bounds";
    return result;
  }

  ss.clear();
  ss.setStartState(start);
  auto goal_region = std::make_shared<SE2PoseGoal>(
    si, goal[0], goal[1], goal[2], request.goal_xy_tolerance, request.goal_yaw_tolerance);
  ss.setGoal(goal_region);

  // 1) Fast path: single analytic Reeds-Shepp segment (no sampling).
  const bool analytic_ok =
    request.try_analytic_reeds_shepp && si->checkMotion(start.get(), goal.get());
  if (analytic_ok) {
    og::PathGeometric path(si, start.get(), goal.get());
    densifyPath(path, request);
    result.waypoints = pathToWaypoints(path);
    result.success = result.waypoints.size() >= 2;
    result.message = result.success ? "Analytic Reeds-Shepp" : "Empty analytic path";
    return result;
  }

  // 2) Sampling planner on Reeds-Shepp metric (needed when obstacles block direct RS).
  ob::PlannerPtr planner;
  if (request.planner_id == "RRTstar") {
    auto rrt = std::make_shared<og::RRTstar>(si);
    rrt->setRange(request.planner_range);
    planner = rrt;
  } else if (request.planner_id == "PRM") {
    planner = std::make_shared<og::PRM>(si);
  } else {
    // RRTConnect needs a GoalSampleableRegion (set above) to grow from the goal.
    auto rrt = std::make_shared<og::RRTConnect>(si);
    rrt->setRange(request.planner_range);
    planner = rrt;
  }
  ss.setPlanner(planner);

  // Give blocked scenes more time than the clear-path analytic case.
  const double budget = std::max(0.5, request.allowed_planning_time);
  const ob::PlannerStatus status = ss.solve(budget);
  if (!status) {
    result.message = "OMPL planning failed within time budget (obstacle likely blocks path)";
    return result;
  }

  auto & path = ss.getSolutionPath();
  if (request.simplify) {
    // Ackermann-safe: uses ReedsShepp distance/interpolate. Never B-spline.
    og::PathSimplifier simplifier(si);
    const double simplify_time = std::min(0.25, 0.25 * budget);
    simplifier.simplify(path, simplify_time);
  }

  densifyPath(path, request);
  result.waypoints = pathToWaypoints(path);
  result.success = result.waypoints.size() >= 2;
  result.message = result.success ? "Success" : "Empty path";
  return result;
}

}  // namespace ackermann_ompl_plugins
