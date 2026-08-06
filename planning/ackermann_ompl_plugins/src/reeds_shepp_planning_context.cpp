#include "ackermann_ompl_plugins/reeds_shepp_planning_context.hpp"
#include "ackermann_ompl_plugins/ackermann_ompl_solver.hpp"
#include "ackermann_ompl_plugins/ackermann_validity_checker.hpp"

#include <moveit/robot_state/conversions.h>
#include <tf2/utils.h>

namespace ackermann_ompl_plugins
{

ReedsSheppPlanningContext::ReedsSheppPlanningContext(
  const std::string & name, const std::string & group,
  const moveit::core::RobotModelConstPtr & model, double turning_radius)
: planning_interface::PlanningContext(name, group),
  robot_model_(model),
  turning_radius_(turning_radius)
{
}

bool ReedsSheppPlanningContext::solve(planning_interface::MotionPlanResponse & res)
{
  const auto & req = getMotionPlanRequest();
  const auto & scene = getPlanningScene();
  if (!scene) {
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  const auto * jmg = robot_model_->getJointModelGroup(getGroupName());
  if (!jmg || jmg->getJointModels().empty()) {
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }
  const auto * joint_model = jmg->getJointModels()[0];

  moveit::core::RobotState start_state(robot_model_);
  moveit::core::robotStateMsgToRobotState(req.start_state, start_state);
  const double * start_pos = start_state.getJointPositions(joint_model);

  double gx = 0.0;
  double gy = 0.0;
  double gyaw = 0.0;
  if (!extractGoalPose(req, gx, gy, gyaw)) {
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  OmplSolveRequest ompl_req;
  ompl_req.turning_radius = turning_radius_;
  ompl_req.planner_id = req.planner_id.empty() ? "RRTstar" : req.planner_id;
  ompl_req.allowed_planning_time = req.allowed_planning_time;
  ompl_req.start_pose.position.x = start_pos[0];
  ompl_req.start_pose.position.y = start_pos[1];
  ompl_req.start_pose.orientation.z = std::sin(start_pos[2] * 0.5);
  ompl_req.start_pose.orientation.w = std::cos(start_pos[2] * 0.5);
  ompl_req.goal_pose.position.x = gx;
  ompl_req.goal_pose.position.y = gy;
  ompl_req.goal_pose.orientation.z = std::sin(gyaw * 0.5);
  ompl_req.goal_pose.orientation.w = std::cos(gyaw * 0.5);

  // Approximate bounds from start/goal with margin
  const double margin = 50.0;
  ompl_req.bounds_low_x = std::min(start_pos[0], gx) - margin;
  ompl_req.bounds_low_y = std::min(start_pos[1], gy) - margin;
  ompl_req.bounds_high_x = std::max(start_pos[0], gx) + margin;
  ompl_req.bounds_high_y = std::max(start_pos[1], gy) + margin;

  AckermannOmplSolver solver;
  const auto ompl_res = solver.solve(
    ompl_req, [&](double x, double y, double yaw) {
      moveit::core::RobotState rs(robot_model_);
      rs.setToDefaultValues();
      const double vals[3] = {x, y, yaw};
      rs.setJointPositions(joint_model, vals);
      rs.update();
      collision_detection::CollisionRequest creq;
      collision_detection::CollisionResult cres;
      scene->checkCollision(creq, cres, rs);
      return !cres.collision;
    });

  if (!ompl_res.success) {
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  res.trajectory_ =
    std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, getGroupName());
  for (const auto & wp : ompl_res.waypoints) {
    moveit::core::RobotState rs(robot_model_);
    rs.setToDefaultValues();
    const double yaw = tf2::getYaw(wp.pose.orientation);
    const double vals[3] = {wp.pose.position.x, wp.pose.position.y, yaw};
    rs.setJointPositions(joint_model, vals);
    rs.update();
    res.trajectory_->addSuffixWayPoint(rs, 0.1);
  }

  res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

bool ReedsSheppPlanningContext::solve(planning_interface::MotionPlanDetailedResponse & res)
{
  planning_interface::MotionPlanResponse simple_res;
  const bool success = solve(simple_res);
  if (simple_res.trajectory_) {
    res.trajectory_.push_back(simple_res.trajectory_);
  }
  res.error_code_ = simple_res.error_code_;
  return success;
}

bool ReedsSheppPlanningContext::terminate()
{
  return true;
}

void ReedsSheppPlanningContext::clear() {}

bool ReedsSheppPlanningContext::extractGoalPose(
  const planning_interface::MotionPlanRequest & req, double & x, double & y, double & yaw) const
{
  if (req.goal_constraints.empty()) {
    return false;
  }

  const auto & gc = req.goal_constraints[0];
  if (!gc.position_constraints.empty()) {
    const auto & pc = gc.position_constraints[0];
    if (!pc.constraint_region.primitive_poses.empty()) {
      x = pc.constraint_region.primitive_poses[0].position.x;
      y = pc.constraint_region.primitive_poses[0].position.y;
    } else {
      return false;
    }
  } else {
    return false;
  }

  if (!gc.orientation_constraints.empty()) {
    yaw = tf2::getYaw(gc.orientation_constraints[0].orientation);
  } else {
    yaw = 0.0;
  }
  return true;
}

}  // namespace ackermann_ompl_plugins
