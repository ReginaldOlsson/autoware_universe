#pragma once

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <ompl/base/StateValidityChecker.h>

namespace ackermann_ompl_plugins
{

class AckermannValidityChecker : public ompl::base::StateValidityChecker
{
public:
  AckermannValidityChecker(
    const ompl::base::SpaceInformationPtr & si,
    const planning_scene::PlanningSceneConstPtr & scene,
    const moveit::core::JointModel * joint_model,
    const moveit::core::RobotModelConstPtr & robot_model);

  bool isValid(const ompl::base::State * state) const override;

private:
  planning_scene::PlanningSceneConstPtr scene_;
  const moveit::core::JointModel * joint_model_;
  mutable moveit::core::RobotState robot_state_;
};

}  // namespace ackermann_ompl_plugins
