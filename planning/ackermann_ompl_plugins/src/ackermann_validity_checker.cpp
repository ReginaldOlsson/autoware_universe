#include "ackermann_ompl_plugins/ackermann_validity_checker.hpp"

#include <ompl/base/spaces/SE2StateSpace.h>

namespace ackermann_ompl_plugins
{

AckermannValidityChecker::AckermannValidityChecker(
  const ompl::base::SpaceInformationPtr & si, const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::JointModel * joint_model,
  const moveit::core::RobotModelConstPtr & robot_model)
: ompl::base::StateValidityChecker(si),
  scene_(scene),
  joint_model_(joint_model),
  robot_state_(robot_model)
{
}

bool AckermannValidityChecker::isValid(const ompl::base::State * state) const
{
  const auto * se2 = state->as<ompl::base::SE2StateSpace::StateType>();

  const double values[3] = {se2->getX(), se2->getY(), se2->getYaw()};
  robot_state_.setJointPositions(joint_model_, values);
  robot_state_.update();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  scene_->checkCollision(req, res, robot_state_);

  return !res.collision && si_->satisfiesBounds(state);
}

}  // namespace ackermann_ompl_plugins
