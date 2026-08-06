#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ackermann_ompl_plugins
{

class ReedsSheppPlanningContext : public planning_interface::PlanningContext
{
public:
  ReedsSheppPlanningContext(
    const std::string & name, const std::string & group,
    const moveit::core::RobotModelConstPtr & model, double turning_radius);

  bool solve(planning_interface::MotionPlanResponse & res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse & res) override;
  bool terminate() override;
  void clear() override;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  double turning_radius_;

  bool extractGoalPose(
    const planning_interface::MotionPlanRequest & req, double & x, double & y,
    double & yaw) const;
};

}  // namespace ackermann_ompl_plugins
