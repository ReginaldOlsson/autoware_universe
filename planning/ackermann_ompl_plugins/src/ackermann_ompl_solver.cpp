#include "ackermann_ompl_plugins/ackermann_ompl_solver.hpp"
#include "ackermann_ompl_plugins/ackermann_state_space.hpp"

#include <ompl/base/MotionValidator.h>
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

  // Signed motion relative to heading for each edge. Keep the real gear so
  // partial splits see +v/-v changes at cusps (do not absorb short runs).
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const double yaw = tf2::getYaw(waypoints[i].pose.orientation);
    const double dx = waypoints[i + 1].pose.position.x - waypoints[i].pose.position.x;
    const double dy = waypoints[i + 1].pose.position.y - waypoints[i].pose.position.y;
    if (std::hypot(dx, dy) < 1e-4) {
      waypoints[i].is_back = (i > 0) ? waypoints[i - 1].is_back : false;
      continue;
    }
    const double forward_dot = std::cos(yaw) * dx + std::sin(yaw) * dy;
    waypoints[i].is_back = forward_dot < 0.0;
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

double motionCheckStep(const OmplSolveRequest & request)
{
  return request.valid_segment_length > 0.0 ? request.valid_segment_length : 0.3;
}

double validityCheckingFraction(const OmplSolveRequest & request, const ob::StateSpacePtr & space)
{
  const double step = motionCheckStep(request);
  const double extent = space->getMaximumExtent();
  if (extent > 0.0) {
    return std::clamp(step / extent, 1e-4, 0.2);
  }
  return std::clamp(request.longest_valid_segment_fraction, 1e-4, 0.2);
}

/// Collision-check Reeds-Shepp motions every `step_m` meters of path length.
/// Does not depend on OMPL's fraction-of-extent DiscreteMotionValidator.
class MetricMotionValidator : public ob::MotionValidator
{
public:
  MetricMotionValidator(const ob::SpaceInformationPtr & si, double step_m)
  : MotionValidator(si), step_m_(std::max(1e-3, step_m))
  {
  }

  bool checkMotion(const ob::State * s1, const ob::State * s2) const override
  {
    if (!si_->isValid(s2)) {
      return false;
    }
    const double dist = si_->distance(s1, s2);
    const int nd = std::max(1, static_cast<int>(std::ceil(dist / step_m_)));
    auto * tmp = si_->allocState();
    bool valid = true;
    for (int i = 1; i < nd && valid; ++i) {
      si_->getStateSpace()->interpolate(s1, s2, static_cast<double>(i) / static_cast<double>(nd), tmp);
      valid = si_->isValid(tmp);
    }
    si_->freeState(tmp);
    return valid;
  }

  bool checkMotion(
    const ob::State * s1, const ob::State * s2, std::pair<ob::State *, double> & last_valid) const
    override
  {
    const double dist = si_->distance(s1, s2);
    const int nd = std::max(1, static_cast<int>(std::ceil(dist / step_m_)));
    auto * tmp = si_->allocState();
    bool valid = true;
    int last_ok = 0;
    for (int i = 1; i <= nd; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(nd);
      si_->getStateSpace()->interpolate(s1, s2, t, tmp);
      if (!si_->isValid(tmp)) {
        valid = false;
        break;
      }
      last_ok = i;
    }
    if (last_valid.first != nullptr && last_ok > 0) {
      const double t = static_cast<double>(last_ok) / static_cast<double>(nd);
      si_->getStateSpace()->interpolate(s1, s2, t, last_valid.first);
      last_valid.second = t;
    }
    si_->freeState(tmp);
    return valid;
  }

private:
  double step_m_;
};

bool poseCollides(const geometry_msgs::msg::Pose & pose, const StateValidityFn & is_valid)
{
  return !is_valid(pose.position.x, pose.position.y, tf2::getYaw(pose.orientation));
}

bool waypointsCollide(
  const std::vector<OmplWaypoint> & waypoints, const StateValidityFn & is_valid, double step_m)
{
  if (waypoints.empty()) {
    return false;
  }
  const double step = std::max(1e-3, step_m);
  for (size_t i = 0; i < waypoints.size(); ++i) {
    if (poseCollides(waypoints[i].pose, is_valid)) {
      return true;
    }
    if (i + 1 >= waypoints.size()) {
      continue;
    }
    const auto & a = waypoints[i].pose.position;
    const auto & b = waypoints[i + 1].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dist = std::hypot(dx, dy);
    const int n = std::max(1, static_cast<int>(std::ceil(dist / step)));
    const double yaw0 = tf2::getYaw(waypoints[i].pose.orientation);
    const double yaw1 = tf2::getYaw(waypoints[i + 1].pose.orientation);
    const double dyaw = normalizeAngle(yaw1 - yaw0);
    for (int k = 1; k < n; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n);
      geometry_msgs::msg::Pose mid;
      mid.position.x = a.x + t * dx;
      mid.position.y = a.y + t * dy;
      mid.orientation = yawToQuat(yaw0 + t * dyaw);
      if (poseCollides(mid, is_valid)) {
        return true;
      }
    }
  }
  return false;
}

bool finalizePath(
  og::PathGeometric & path, const OmplSolveRequest & request, const StateValidityFn & is_valid,
  OmplSolveResult & result, const std::string & success_message)
{
  densifyPath(path, request);
  result.waypoints = pathToWaypoints(path);
  if (result.waypoints.size() < 2) {
    result.success = false;
    result.message = "Empty path";
    return false;
  }
  if (waypointsCollide(result.waypoints, is_valid, motionCheckStep(request))) {
    result.success = false;
    result.waypoints.clear();
    result.message = "Path intersects costmap obstacles";
    return false;
  }
  result.success = true;
  result.message = success_message;
  return true;
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
  const double step_m = motionCheckStep(request);
  si->setStateValidityCheckingResolution(validityCheckingFraction(request, space));
  si->setMotionValidator(std::make_shared<MetricMotionValidator>(si, step_m));
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
    finalizePath(path, request, is_valid, result, "Analytic Reeds-Shepp");
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

  finalizePath(path, request, is_valid, result, "Success");
  return result;
}

}  // namespace ackermann_ompl_plugins
